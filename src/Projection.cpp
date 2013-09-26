#include <iostream>

#include "Projection.hpp"
#include "Utilities.hpp"

namespace
{
	struct ProjectionResidual : LMFunctor<double>
	{
		ProjectionResidual(const Vec3Vec &points, const Vec2Vec &projections)
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
				Vec3 pr = P * Vec4(m_points[i].x(), m_points[i].y(), m_points[i].z(), 1.0);
				pr /= -pr.z();

				fvec(position + 0) = pr.x() - m_projections[i].x();
				fvec(position + 1) = pr.y() - m_projections[i].y();

				position += 2;
			}

			return 0;
		}

		int operator()(const Mat34 &P, Vec &fvec) const
		{
			int position = 0;
			for (int i = 0; i < m_points.size(); ++i)
			{
				Vec3 pr = P * Vec4(m_points[i].x(), m_points[i].y(), m_points[i].z(), 1.0);
				pr /= -pr.z();

				fvec(position + 0) = pr.x() - m_projections[i].x();
				fvec(position + 1) = pr.y() - m_projections[i].y();

				position += 2;
			}

			return 0;
		}

		Vec3Vec m_points;
		Vec2Vec m_projections;
	};

	std::vector<int> EvaluateProjectionMatrix(const Mat34 &P, const Vec3Vec &points, const Vec2Vec &projections, double threshold, double *error)
	{
		std::vector<int> inlier_indices;
		*error = 0.0;

		for (int i = 0; i < points.size(); ++i)
		{
			Vec3 pr = P * Vec4(points[i].x(), points[i].y(), points[i].z(), 1.0);
			pr /= -pr.z();
			    
			double dist = (pr.head(2) - projections[i]).squaredNorm();

			if (dist < threshold)
			{
				inlier_indices.push_back(i);
				*error += dist;
			}
		}

		return inlier_indices;
	}

	Mat34 RefineProjectionMatrixNonlinear(const Mat34& P, const Vec3Vec &points, const Vec2Vec &projections)
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

Mat34 ComputeProjectionMatrix(const Vec3Vec &points, const Vec2Vec &projections, bool optimize)
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

int ComputeProjectionMatrixRansac(const Vec3Vec &points, const Vec2Vec &projections,
								  int ransac_rounds, double ransac_threshold,
								  Mat34 *P)
{
	int num_pts = static_cast<int>(points.size());

	if (num_pts < 6)
	{
		std::cerr << "[ComputeProjectionMatrixRansac] Error: need at least 6 points!";
		return -1;
	}

	Vec3Vec final_pts;
	Vec2Vec final_projs;

	int max_inliers		= 0;
	double max_error	= std::numeric_limits<double>::max();
	for (int round = 0; round < ransac_rounds; round++)
	{
		Vec3Vec sample_pts;
		Vec2Vec sample_projs;

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
			*P			= hypothesis;
			max_error	= error;
			max_inliers	= num_inliers;

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
				*P			= Pi;
				max_error	= error_i;
				max_inliers	= num_inliers_i;
			}

		}
	}
		
	if (max_inliers < 6)
	{
		std::cerr << "[ComputeProjectionMatrixRansac] Error: Too few inliers to continue\n";
		return -1;
	}
		
	// Re-estimate P from all correspondences classified as inliers and refine the result
	//*P = ComputeProjectionMatrix(final_pts, final_projs, true);

	return max_inliers;
}

void DecomposeProjectionMatrix(const Mat &P, Mat3 *K, Mat3 *R, Vec3 *t)
{
	Eigen::HouseholderQR<Mat3> qr(P.topLeftCorner<3, 3>().inverse());
	*K = qr.matrixQR().triangularView<Eigen::Upper>();
	*R = qr.householderQ();

	*K = K->inverse().eval();
	*R = R->inverse().eval();

	if (R->determinant() < 0)
	{
		K->col(0) *= -1.0;
		R->row(0) *= -1.0;
	}

	*K /= (*K)(2, 2);

	if ((*K)(0, 0) < 0)
	{
		Mat3 D;
		D << -1,  0,  0,
			  0, -1,  0,
			  0,  0,  1;

		*K = *K *  D;
		*R =  D * *R;
	}

	if ((*K)(1, 1) < 0)
	{
		(*K)(1, 1) *= -1;

		Mat3 D;
		D << -1,  0,  0,
			  0,  1,  0,
			  0,  0, -1;

		*R =  D * *R;
	}

	*t = K->partialPivLu().solve(P.col(3));
}
