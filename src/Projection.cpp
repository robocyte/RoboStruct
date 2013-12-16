#include <iostream>

#include "Projection.hpp"
#include "Utilities.hpp"

namespace
{
    struct ProjectionResidual : LMFunctor<double>
    {
        ProjectionResidual(const Point3Vec &points, const Point2Vec &projections)
            : LMFunctor<double>(11, 2 * static_cast<int>(points.size()))
            , m_points(points)
            , m_projections(projections)
        {}

        int operator()(const Vec &x, Vec &fvec) const
        {
            Mat34 P;
            P.row(0) = x.segment(0, 4);
            P.row(1) = x.segment(4, 4);
            P.row(2) << x(8), x(9), x(10), 1.0;

            int position = 0;
            for (int i = 0; i < m_points.size(); ++i)
            {
                Point3 projection = P * EuclideanToHomogenous(m_points[i]);
                projection /= -projection.z();

                fvec(position + 0) = projection.x() - m_projections[i].x();
                fvec(position + 1) = projection.y() - m_projections[i].y();

                position += 2;
            }

            return 0;
        }

        int operator()(const Mat34 &P, Vec &fvec) const
        {
	        int position = 0;
	        for (int i = 0; i < m_points.size(); ++i)
	        {
		        Point3 projection = P * EuclideanToHomogenous(m_points[i]);
		        projection /= -projection.z();

		        fvec(position + 0) = projection.x() - m_projections[i].x();
		        fvec(position + 1) = projection.y() - m_projections[i].y();

		        position += 2;
	        }

	        return 0;
        }

        Point3Vec m_points;
        Point2Vec m_projections;
    };

    std::vector<int> EvaluateProjectionMatrix(const Mat34 &P, const Point3Vec &points, const Point2Vec &projections, double threshold, double *error)
    {
        std::vector<int> inlier_indices;
        *error = 0.0;

        for (int i = 0; i < points.size(); ++i)
        {
            Point3 projection = P * EuclideanToHomogenous(points[i]);
            projection /= -projection.z();

            double dist = (projection.head<2>() - projections[i]).squaredNorm();

            if (dist < threshold)
            {
                inlier_indices.push_back(i);
                *error += dist;
            }
        }

        return inlier_indices;
    }

    Mat34 RefineProjectionMatrixNonlinear(const Mat34& P, const Point3Vec &points, const Point2Vec &projections)
    {
        Vec x(11);

        x.segment(0, 4) = P.row(0);
        x.segment(4, 4) = P.row(1);
        x.segment(8, 3) = P.block<1, 3>(2, 0);

        ProjectionResidual functor(points, projections);
        Eigen::DenseIndex nfev;
        Eigen::LevenbergMarquardt<ProjectionResidual>::lmdif1(functor, x, &nfev, 1.0e-5);

        Mat34 P_refined;
        P_refined.row(0) = x.segment(0, 4);
        P_refined.row(1) = x.segment(4, 4);
        P_refined.row(2) << x(8), x(9), x(10), 1.0;

        return P_refined;
    }
}

Mat34 ComputeProjectionMatrix(const Point3Vec &points, const Point2Vec &projections, bool optimize)
{
    int num_pts((int)points.size());

    Mat A(2 * num_pts, 11); A.setZero();
    Vec b(2 * num_pts);
    Vec x(11);

    for (int i = 0; i < num_pts; i++)
    {
        A.block<1, 3>(2 * i + 0, 0) = points[i];
        A.block<1, 3>(2 * i + 1, 4) = points[i];
        A.block<1, 3>(2 * i + 0, 8) = projections[i].x() * points[i];
        A.block<1, 3>(2 * i + 1, 8) = projections[i].y() * points[i];
        A(2 * i + 0, 3) = 1.0;
        A(2 * i + 1, 7) = 1.0;

        b.segment<2>(2 * i) = -projections[i];
    }

    // Find the least squares result
    x = A.colPivHouseholderQr().solve(b);

    if (optimize)
    {
        ProjectionResidual functor(points, projections);
        Eigen::DenseIndex nfev;
        Eigen::LevenbergMarquardt<ProjectionResidual>::lmdif1(functor, x, &nfev, 1.0e-5);
    }

    Mat34 P;
    P.row(0) = x.segment(0, 4);
    P.row(1) = x.segment(4, 4);
    P.row(2) << x(8), x(9), x(10), 1.0;

    return P;
}

int ComputeProjectionMatrixRansac(const Point3Vec &points, const Point2Vec &projections,
                                    int ransac_rounds, double ransac_threshold,
                                    Mat34 *P)
{
    int num_pts = static_cast<int>(points.size());

    if (num_pts < 6)
    {
        std::cerr << "[ComputeProjectionMatrixRansac] Error: need at least 6 points!";
        return -1;
    }

    Point3Vec final_pts;
    Point2Vec final_projs;

    int max_inliers     = 0;
    double max_error    = std::numeric_limits<double>::max();
    for (int round = 0; round < ransac_rounds; round++)
    {
        Point3Vec sample_pts;
        Point2Vec sample_projs;

        auto sample_indices = util::GetNRandomIndices(6, num_pts);
        for (int i : sample_indices)
        {
            sample_pts.push_back(points[i]);
            sample_projs.push_back(projections[i]);
        }

        // Solve for the parameters
        Mat34 hypothesis = ComputeProjectionMatrix(sample_pts, sample_projs);

        // Count the number of inliers in all correspondences
        double error = 0.0;
        auto inlier_indices = EvaluateProjectionMatrix(hypothesis, points, projections, ransac_threshold, &error);

        int num_inliers = static_cast<int>(inlier_indices.size());
        if (num_inliers < 6) continue;

        if (num_inliers > max_inliers || (num_inliers == max_inliers && error < max_error))
        {
            *P          = hypothesis;
            max_error   = error;
            max_inliers = num_inliers;

            for (int i : inlier_indices)
            {
                final_pts.push_back(points[i]);
                final_projs.push_back(projections[i]);
            }

            // Quick pseudo LO-RANSAC
            double error_i = 0.0;
            Mat34 Pi = ComputeProjectionMatrix(final_pts, final_projs, true);
            auto inlier_indices_i = EvaluateProjectionMatrix(Pi, points, projections, ransac_threshold, &error_i);
            int num_inliers_i = static_cast<int>(inlier_indices_i.size());

            if (num_inliers_i > max_inliers || (num_inliers_i == max_inliers && error_i < max_error))
            {
                *P          = Pi;
                max_error   = error_i;
                max_inliers = num_inliers_i;
            }
        }
    }

    if (max_inliers < 6)
    {
        std::cerr << "[ComputeProjectionMatrixRansac] Error: Too few inliers to continue\n";
        return -1;
    }


    return max_inliers;
}

void DecomposeProjectionMatrix(const Mat &P, Mat3 *K, Mat3 *R, Vec3 *t)
{
    // Decompose using the RQ decomposition HZ A4.1.1 pag.579.
    Mat3 Kp = P.block(0, 0, 3, 3);
    Mat3 Q; Q.setIdentity();

    // Set K(2, 1) to zero.
    if (Kp(2, 1) != 0)
    {
        double c = -Kp(2, 2);
        double s = Kp(2, 1);
        double l = sqrt(c * c + s * s);
        c /= l; s /= l;
        Mat3 Qx;
        Qx <<  1,  0,  0,
               0,  c, -s,
               0,  s,  c;
        Kp = Kp * Qx;
        Q = Qx.transpose() * Q;
    }

    // Set K(2, 0) to zero.
    if (Kp(2, 0) != 0)
    {
        double c = Kp(2, 2);
        double s = Kp(2, 0);
        double l = sqrt(c * c + s * s);
        c /= l; s /= l;
        Mat3 Qy;
        Qy <<  c,  0,  s,
               0,  1,  0,
              -s,  0,  c;
        Kp = Kp * Qy;
        Q = Qy.transpose() * Q;
    }

    // Set K(1, 0) to zero.
    if (Kp(1, 0) != 0)
    {
        double c = -Kp(1, 1);
        double s = Kp(1, 0);
        double l = sqrt(c * c + s * s);
        c /= l; s /= l;
        Mat3 Qz;
        Qz <<  c, -s,  0,
               s,  c,  0,
               0,  0,  1;
        Kp = Kp * Qz;
        Q = Qz.transpose() * Q;
    }

    Mat3 Rp = Q;

    // Ensure that the diagonal is positive and R determinant == 1
    if (Kp(2, 2) < 0)
    {
        Kp = -Kp;
        Rp = -Rp;
    }

    if (Kp(1, 1) < 0)
    {
        Mat3 S;
        S <<  1,  0,  0,
              0, -1,  0,
              0,  0,  1;
        Kp = Kp * S;
        Rp = S  * Rp;
    }

    if (Kp(0, 0) < 0)
    {
        Mat3 S;
        S << -1,  0,  0,
              0,  1,  0,
              0,  0,  1;
        Kp = Kp * S;
        Rp = S  * Rp;
    }

    // Compute translation.
    Vec3 tp = Kp.colPivHouseholderQr().solve(P.col(3));

    if(Rp.determinant() < 0)
    {
        Rp = -Rp;
        tp = -tp;
    }

    // Scale K so that K(2, 2) = 1
    Kp = Kp / Kp(2, 2);

    *K = Kp;
    *R = Rp;
    *t = tp;
}
