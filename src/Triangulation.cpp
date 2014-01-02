#include <iostream>

#include "Triangulation.hpp"
#include "Utilities.hpp"

namespace
{

    struct TriangulationResidual : LMFunctor<double>
    {
        TriangulationResidual(const Observations &observations)
            : LMFunctor<double>(3, 2 * static_cast<int>(observations.size()))
            , m_observations(observations)
        {}

        int operator()(const Vec &x, Vec &fvec) const
        {
            int position = 0;
            for (const auto &observation : m_observations)
            {
                auto projection = Project(x, observation);

                fvec(position + 0) = observation.m_point.x() - projection.x();
                fvec(position + 1) = observation.m_point.y() - projection.y();

                position += 2;
            }

            return 0;
        }

        Observations m_observations;
    };

}

double ComputeRayAngle(Point2 p, Point2 q, const Camera &cam1, const Camera &cam2)
{
    Mat3 K1_inv = cam1.GetIntrinsicMatrix().inverse();
    Mat3 K2_inv = cam2.GetIntrinsicMatrix().inverse();

    Point3 p3n = K1_inv * EuclideanToHomogenous(p);
    Point3 q3n = K2_inv * EuclideanToHomogenous(q);

    Point2 pn = p3n.head<2>() / p3n.z();
    Point2 qn = q3n.head<2>() / q3n.z();

    Point3 p_w = cam1.m_R.transpose() * Point3(pn.x(), pn.y(), -1.0);
    Point3 q_w = cam2.m_R.transpose() * Point3(qn.x(), qn.y(), -1.0);

    // Compute the angle between the rays
    double dot = p_w.dot(q_w);
    double mag = p_w.norm() * q_w.norm();

    return acos(util::clamp((dot / mag), (-1.0 + 1.0e-8), (1.0 - 1.0e-8)));
}

bool CheckCheirality(const Point3 p, const Camera &cam)
{
    Point3 pt = cam.m_R * (p - cam.m_t);
    return (pt.z() < 0.0);
}

Point2 Project(const Point3 &p, const Observation &observation)
{
    Point3 projection = observation.m_R * p + observation.m_t;
    return projection.head<2>() / projection.z();
}

Point3 Triangulate(const Observations &observations, double *error, bool optimize)
{
    int num_points = static_cast<int>(observations.size());

    Mat A(2 * num_points, 3);
    Vec b(2 * num_points);
    Vec x(3);

    for (int i = 0; i < num_points; i++)
    {
        A.row(2 * i + 0) = observations[i].m_R.row(0) - observations[i].m_point.x() * observations[i].m_R.row(2);
        A.row(2 * i + 1) = observations[i].m_R.row(1) - observations[i].m_point.y() * observations[i].m_R.row(2);

        b(2 * i + 0) = observations[i].m_t(2) * observations[i].m_point.x() - observations[i].m_t(0);
        b(2 * i + 1) = observations[i].m_t(2) * observations[i].m_point.y() - observations[i].m_t(1);
    }

    // Find the least squares result
    x = A.colPivHouseholderQr().solve(b);

    TriangulationResidual functor(observations);

    // Run a non-linear optimization to refine the result
    if (optimize)
    {
        Eigen::NumericalDiff<TriangulationResidual> numDiff(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<TriangulationResidual>> lm(numDiff);

        lm.parameters.ftol   = 1.0e-5;
        lm.parameters.xtol   = 1.0e-5;
        lm.parameters.maxfev = 200;

        auto status = lm.minimize(x);
    }

    if (error != nullptr)
    {
        Vec residuals(2 * num_points);
        functor(x, residuals);
        *error = residuals.squaredNorm();
    }

    return x;
}

bool FindExtrinsics(const Mat3 &E, const Point2Vec &pts1, const Point2Vec &pts2, Mat3 *R, Vec3 *t)
{
    int num_correspondences = static_cast<int>(pts1.size());

    // Put first camera at origin
    Mat3 R0;    R0.setIdentity();
    Vec3 t0;    t0.setZero();

    // Find the SVD of E
    Eigen::JacobiSVD<Mat3> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 U  = svd.matrixU();
    Mat3 Vt = svd.matrixV().transpose();

    // Find R and t
    Mat3 D;
    D << 0,  1,  0,
        -1,  0,  0,
         0,  0,  1;

    Mat3 Ra = U * D * Vt;
    Mat3 Rb = U * D.transpose() * Vt;

    if (Ra.determinant() < 0.0) Ra *= -1.0;
    if (Rb.determinant() < 0.0) Rb *= -1.0;

    Vec3 tu = U.col(2);

    // Figure out which configuration is correct using the supplied points
    int c1_pos = 0, c2_pos = 0, c1_neg = 0, c2_neg = 0;

    for (int i = 0; i < num_correspondences; i++)
    {
        Observations observations;
        observations.push_back(Observation(pts1[i], R0, t0));
        observations.push_back(Observation(pts2[i], Ra, tu));

        Point3 Q = Triangulate(observations);
        Point3 PQ = (Ra * Q) + tu;

        if (Q.z() > 0)  c1_pos++;
        else            c1_neg++;
        
        if (PQ.z() > 0) c2_pos++;
        else            c2_neg++;
    }

    if (c1_pos < c1_neg && c2_pos < c2_neg)
    {
        *R =  Ra;
        *t =  tu;
    } else if (c1_pos > c1_neg && c2_pos > c2_neg)
    {
        *R =  Ra;
        *t = -tu;
    } else
    {
        // Triangulate again
        c1_pos = c1_neg = c2_pos = c2_neg = 0;

        for (int i = 0; i < num_correspondences; i++)
        {
            Observations observations;
            observations.push_back(Observation(pts1[i], R0, t0));
            observations.push_back(Observation(pts2[i], Rb, tu));

            Point3 Q = Triangulate(observations);
            Point3 PQ = (Rb * Q) + tu;

            if (Q.z() > 0)  c1_pos++;
            else            c1_neg++;
        
            if (PQ.z() > 0) c2_pos++;
            else            c2_neg++;
        }

        if (c1_pos < c1_neg && c2_pos < c2_neg)
        {
            *R =  Rb;
            *t =  tu;
        } else if (c1_pos > c1_neg && c2_pos > c2_neg)
        {
            *R =  Rb;
            *t = -tu;
        } else
        {
            std::cerr << "[FindExtrinsics] Error: no case found!";
            return false;
        }
    }

    return true;
}
