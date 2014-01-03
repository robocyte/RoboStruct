#include "FivePoint.hpp"
#include "Poly.hpp"
#include "Triangulation.hpp"
#include "Utilities.hpp"

namespace
{

    typedef std::vector<Poly3> PolyVec;

    Mat ComputeNullspaceBasis(const Point2Vec &pts1, const Point2Vec &pts2)
    {
        // Generate the epipolar constraint matrix
        Eigen::Matrix<double, 5, 9> A;
        for (int i = 0; i < 5; i++)
        {
            A(i, 0) = pts1[i].x() * pts2[i].x();
            A(i, 1) = pts1[i].y() * pts2[i].x();
            A(i, 2) = pts2[i].x();

            A(i, 3) = pts1[i].x() * pts2[i].y();
            A(i, 4) = pts1[i].y() * pts2[i].y();
            A(i, 5) = pts2[i].y();

            A(i, 6) = pts1[i].x();
            A(i, 7) = pts1[i].y();
            A(i, 8) = 1.0;
        }

        Eigen::JacobiSVD<Eigen::Matrix<double, 5, 9>> svd;
        return svd.compute(A, Eigen::ComputeFullV).matrixV().transpose().bottomLeftCorner<4, 9>();
    }

    PolyVec ComputeConstraintMatrix(const Mat &basis)
    {
        PolyVec constraints(10);

        // Basis rows are X, Y, Z, W; essential matrix is of form x*X + y*Y + z*Z + W

        // Create pt1 polynomial for each entry of E
        Poly3 polys[9];
        Poly3 poly_term1, poly_term2, poly_term3, poly_det;
        Poly3 poly_EET[6], poly_lambda[6], poly_tr, poly_lambdaE[9];

        for (int i = 0; i < 9; i++) polys[i] = Poly3New(basis(0, i), basis(1, i), basis(2, i), basis(3, i));

        // Create pt1 polynomial from the constraint det(E) = 0
        poly_term1 = Poly3Mult21(Poly3Sub(Poly3Mult11(polys[1], polys[5]), Poly3Mult11(polys[2], polys[4])), polys[6]);
        poly_term2 = Poly3Mult21(Poly3Sub(Poly3Mult11(polys[2], polys[3]), Poly3Mult11(polys[0], polys[5])), polys[7]);
        poly_term3 = Poly3Mult21(Poly3Sub(Poly3Mult11(polys[0], polys[4]), Poly3Mult11(polys[1], polys[3])), polys[8]);
        poly_det   = Poly3Add(poly_term1, Poly3Add(poly_term2, poly_term3));

        // Create polynomials for the singular value constraint
        for (int i = 0; i < 6; i++)
        {
            int r = 0, c = 0, k;
            poly_EET[i] = Poly3New(0.0, 0.0, 0.0, 0.0);

            switch(i)
            {
            case 0:
            case 1:
            case 2:
                r = 0;
                c = i;
                break;
            case 3:
            case 4:
                r = 1;
                c = i-2;
                break;
            case 5:
                r = 2;
                c = 2;
                break;
            }

            for (k = 0; k < 3; k++) poly_EET[i] = Poly3Add(poly_EET[i], Poly3Mult11(polys[r * 3 + k], polys[c * 3 + k]));
        }

        poly_tr = Poly3Add3(poly_EET[0], poly_EET[3], poly_EET[5]);
        poly_tr = Poly3Scale(poly_tr, 0.5);

        poly_lambda[0]  = Poly3Sub(poly_EET[0], poly_tr);
        poly_lambda[1]  = poly_EET[1];
        poly_lambda[2]  = poly_EET[2];
        poly_lambda[3]  = Poly3Sub(poly_EET[3], poly_tr);
        poly_lambda[4]  = poly_EET[4];
        poly_lambda[5]  = Poly3Sub(poly_EET[5], poly_tr);
        poly_lambdaE[0] = Poly3Add3(Poly3Mult(poly_lambda[0], polys[0]), Poly3Mult(poly_lambda[1], polys[3]), Poly3Mult(poly_lambda[2], polys[6]));
        poly_lambdaE[1] = Poly3Add3(Poly3Mult(poly_lambda[0], polys[1]), Poly3Mult(poly_lambda[1], polys[4]), Poly3Mult(poly_lambda[2], polys[7]));
        poly_lambdaE[2] = Poly3Add3(Poly3Mult(poly_lambda[0], polys[2]), Poly3Mult(poly_lambda[1], polys[5]), Poly3Mult(poly_lambda[2], polys[8]));
        poly_lambdaE[3] = Poly3Add3(Poly3Mult21(poly_lambda[1], polys[0]), Poly3Mult21(poly_lambda[3], polys[3]), Poly3Mult21(poly_lambda[4], polys[6]));
        poly_lambdaE[4] = Poly3Add3(Poly3Mult21(poly_lambda[1], polys[1]), Poly3Mult21(poly_lambda[3], polys[4]), Poly3Mult21(poly_lambda[4], polys[7]));
        poly_lambdaE[5] = Poly3Add3(Poly3Mult21(poly_lambda[1], polys[2]), Poly3Mult21(poly_lambda[3], polys[5]), Poly3Mult21(poly_lambda[4], polys[8]));
        poly_lambdaE[6] = Poly3Add3(Poly3Mult21(poly_lambda[2], polys[0]), Poly3Mult21(poly_lambda[4], polys[3]), Poly3Mult21(poly_lambda[5], polys[6]));
        poly_lambdaE[7] = Poly3Add3(Poly3Mult21(poly_lambda[2], polys[1]), Poly3Mult21(poly_lambda[4], polys[4]), Poly3Mult21(poly_lambda[5], polys[7]));
        poly_lambdaE[8] = Poly3Add3(Poly3Mult21(poly_lambda[2], polys[2]), Poly3Mult21(poly_lambda[4], polys[5]), Poly3Mult21(poly_lambda[5], polys[8]));

        for (int i = 0; i < 9; i++) constraints[i] = poly_lambdaE[i];

        constraints[9] = poly_det;

        return constraints;
    }

    Mat ComputeGroebnerBasis(const PolyVec &constraints) 
    {
        Eigen::Matrix<double, 10, 20> A;

        for (int i = 0; i < 10; i++)
        {
            A(i, 0)  = constraints[i].v[coef_xxx];
            A(i, 1)  = constraints[i].v[coef_xxy];
            A(i, 2)  = constraints[i].v[coef_xyy];
            A(i, 3)  = constraints[i].v[coef_yyy];
            A(i, 4)  = constraints[i].v[coef_xxz];
            A(i, 5)  = constraints[i].v[coef_xyz];
            A(i, 6)  = constraints[i].v[coef_yyz];
            A(i, 7)  = constraints[i].v[coef_xzz];
            A(i, 8)  = constraints[i].v[coef_yzz];
            A(i, 9)  = constraints[i].v[coef_zzz];
            A(i, 10) = constraints[i].v[coef_xx];
            A(i, 11) = constraints[i].v[coef_xy];
            A(i, 12) = constraints[i].v[coef_yy];
            A(i, 13) = constraints[i].v[coef_xz];
            A(i, 14) = constraints[i].v[coef_yz];
            A(i, 15) = constraints[i].v[coef_zz];
            A(i, 16) = constraints[i].v[coef_x];
            A(i, 17) = constraints[i].v[coef_y];
            A(i, 18) = constraints[i].v[coef_z];
            A(i, 19) = constraints[i].v[coef_1];
        }

        // Do a full Gaussian elimination
        for (int i = 0; i < 10; ++i)
        {
            A.row(i) /= A(i, i);                                                // Make the leading coefficient of row i = 1
            for (int j = i + 1; j < 10; ++j) A.row(j) -= A.row(i) * A(j, i);    // Subtract from other rows
        }

        // Now, do the back substitution
        for (int i = 9; i >= 0; i--)
        {
            for (int j = 0; j < i; j++) A.row(j) -= A.row(i) * A(j, i);
        }

        return A.topRightCorner<10, 10>();
    }

    Mat ComputeActionMatrix(const Mat &gbasis) 
    {
        Eigen::Matrix<double, 10, 10> action;
        action.setZero();

        action.row(0) = -gbasis.row(0);
        action.row(1) = -gbasis.row(1);
        action.row(2) = -gbasis.row(2);
        action.row(3) = -gbasis.row(4);
        action.row(4) = -gbasis.row(5);
        action.row(5) = -gbasis.row(7);
        action(6, 0)  = 1.0;
        action(7, 1)  = 1.0;
        action(8, 3)  = 1.0;
        action(9, 6)  = 1.0;

        return action;
    }

    Mat3Vec ComputeEssentialMatricesGroebner(const Mat &action, const Mat &basis)
    {
        Mat3Vec solutions;

        Eigen::EigenSolver<Eigen::Matrix<double, 10, 10>> Es(action);

        for (int i = 0; i < 10; ++i)
        {
            auto evec = Es.eigenvectors().col(i);
            if (evec(0).imag() == 0)
            {
                double w_inv = 1.0 / evec(9).real();
                double x = evec(6).real() * w_inv;
                double y = evec(7).real() * w_inv;;
                double z = evec(8).real() * w_inv;;

                auto Et = (basis.row(0) * x) + (basis.row(1) * y) + (basis.row(2) * z) + (basis.row(3));
                auto mult = 1.0 / Et(8);
                auto res = Et * mult;

                Mat3 E;
                E.row(0) = res.segment(0, 3);
                E.row(1) = res.segment(3, 3);
                E.row(2) = res.segment(6, 3);

                solutions.push_back(E);
            }
        }

        return solutions;
    }

    Mat3Vec GenerateEssentialMatrixHypotheses(const Point2Vec &pts1, const Point2Vec &pts2)
    {
        auto basis          = ComputeNullspaceBasis(pts1, pts2);
        auto constraints    = ComputeConstraintMatrix(basis);
        auto gbasis         = ComputeGroebnerBasis(constraints);
        auto action         = ComputeActionMatrix(gbasis);
        return                ComputeEssentialMatricesGroebner(action, basis);
    }

    double FundamentalMatrixComputeResidual(const Mat3 &F, const Point3 &pt1, const Point3 &pt2)
    {
        Point3 Fl = F * pt2;
        Point3 Fr = F * pt1;

        double pt = pt1.dot(Fl);

        return (1.0 / (Fl(0) * Fl(0) + Fl(1) * Fl(1)) + 1.0 / (Fr(0) * Fr(0) + Fr(1) * Fr(1))) * (pt * pt);
    }

    int EvaluateFundamentalMatrix(const Mat3 &F, const Point2Vec &pts1, const Point2Vec &pts2, double thresh_norm, double *score)
    {
        int    num_inliers = 0;
        double min_resid   = std::numeric_limits<double>::min();
        double likelihood  = 0.0;

        for (int i = 0; i < pts1.size(); i++)
        {
            double resid = FundamentalMatrixComputeResidual(F, Point3(pts2[i].x(), pts2[i].y(), 1.0), Point3(pts1[i].x(), pts1[i].y(), 1.0));

            likelihood += log(1.0 + resid * resid / (thresh_norm));

            if (resid < thresh_norm)
            {
                if (resid < min_resid) min_resid = resid;
                num_inliers++;
            }
        }

        *score = likelihood;

        return num_inliers;
    }

}

int ComputeRelativePoseRansac(const Point2Vec &pts1, const Point2Vec &pts2, const Mat &K1, const Mat &K2,
                              double ransac_threshold, int ransac_rounds, Mat3 *R, Vec3 *t)
{
    int num_matches = static_cast<int>(pts1.size());

    Point2Vec pts1_norm;    pts1_norm.reserve(num_matches);
    Point2Vec pts2_norm;    pts2_norm.reserve(num_matches);

    Mat3 K1_inv = K1.inverse();
    Mat3 K2_inv = K2.inverse();

    for (int i = 0; i < num_matches; i++)
    {
        Point3 r_norm = K1_inv * EuclideanToHomogenous(pts1[i]);
        Point3 l_norm = K2_inv * EuclideanToHomogenous(pts2[i]);

        pts1_norm.push_back(-r_norm.head<2>());
        pts2_norm.push_back(-l_norm.head<2>());
    }

    Mat3 E_best;
    int max_inliers = 0;
    auto min_score = std::numeric_limits<double>::max();
    for (int round = 0; round < ransac_rounds; round++)
    {
        Point2Vec sample_pts1, sample_pts2;
        int num_ident = 0;

        auto sample_indices = util::GetNRandomIndices(5, num_matches);
        for (int i : sample_indices)
        {
            sample_pts1.push_back(pts1_norm[i]);
            sample_pts2.push_back(pts2_norm[i]);

            // Check for degeneracy
            if (pts1_norm[i] == pts2_norm[i]) num_ident++;
        }
        if (num_ident >= 3) continue;   // Choose another 5

        auto hypotheses = GenerateEssentialMatrixHypotheses(sample_pts1, sample_pts2);

        for (const auto &candidate : hypotheses)
        {
            Mat3 E2 = candidate;

            E2(0, 0) = -E2(0, 0);
            E2(0, 1) = -E2(0, 1);
            E2(1, 0) = -E2(1, 0);
            E2(1, 1) = -E2(1, 1);
            E2(2, 2) = -E2(2, 2);

            Mat3 F = K2_inv * E2 * K1_inv;

            double score = 0.0;
            int num_inliers = EvaluateFundamentalMatrix(F, pts1, pts2, ransac_threshold * ransac_threshold, &score);

            if (num_inliers > max_inliers || (num_inliers == max_inliers && score < min_score))
            {
                E_best      = candidate;
                min_score   = score;
                max_inliers = num_inliers;
            }
        }
    }

    if (max_inliers > 0) bool success = FindExtrinsics(E_best, pts1_norm, pts2_norm, R, t);

    return max_inliers;
}