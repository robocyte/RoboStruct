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

	int EvaluateProjectionMatrix(const Mat34 &P, const Vec3Vec &points, const Vec2Vec &projections, double threshold, double *error)
	{
		int num_inliers	= 0;
		*error			= 0.0;

		for (int i = 0; i < points.size(); ++i)
		{
			Vec3 pr = P * Vec4(points[i].x(), points[i].y(), points[i].z(), 1.0);
			pr /= -pr.z();
			    
			double dist = (pr.head(2) - projections[i]).squaredNorm();

			if (dist < threshold)
			{
				num_inliers++;
				*error += dist;
			}
		}

		return num_inliers;
	}

	int RefineProjectionMatrixNonlinear(const Vec3Vec &points, const Vec2Vec &projections, double *Pin, double *Pout)
	{
		int num_pts = static_cast<int>(points.size());
		if (num_pts < 6)
		{
			std::cerr << "[FindProjectionMatrixNonlinear] Error: need at least 6 points!";
			return -1;
		}

		Vec x(11);
	
		for (int i = 0; i < 11; ++i) x(i) = Pin[i];

		ProjectionResidual functor(points, projections);
		Eigen::DenseIndex nfev;
		Eigen::LevenbergMarquardt<ProjectionResidual>::lmdif1(functor, x, &nfev, 1.0e-5);

		for (int i = 0; i < 11; ++i) Pout[i] = x(i); Pout[11] = 1.0;

		return 0;
	}
}

void ComputeProjectionMatrix(const Vec3Vec &points, const Vec2Vec &projections, double *P)
{
	int num_pts = static_cast<int>(points.size());
	if (num_pts < 6)
	{
		std::cerr << "[FindProjectionMatrix] Error: need at least 6 points!";
		return;
	}

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
	x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	for (int i = 0; i < 11; ++i) P[i] = x(i);
	P[11] = 1.0;
}

int ComputeProjectionMatrixRansac(int num_pts, v3_t *points, v2_t *projections,
								  int ransac_rounds, double ransac_threshold,
								  double *P)
{
	Vec3Vec epoints;
	Vec2Vec eprojections;
	for (int i = 0; i < num_pts; ++i)
	{
		epoints.push_back(Vec3(Vx(points[i]), Vy(points[i]), Vz(points[i])));
		eprojections.push_back(Vec2(Vx(projections[i]), Vy(projections[i])));
	}

	if (num_pts < 6)
	{
		std::cerr << "[ComputeProjectionMatrixRansac] Error: need at least 6 points!";
		return -1;
    }

	double Pbest[12];

	int max_inliers		= 0;
	double max_error	= 0.0;
	for (int round = 0; round < ransac_rounds; round++)
	{
		double error = 0.0;

		Vec3Vec sample_pts;		sample_pts.reserve(6);
		Vec2Vec sample_projs;	sample_projs.reserve(6);

		auto sample_indices = util::GetNRandomIndices(6, num_pts);
		for (int i : sample_indices)
		{
			sample_pts.push_back(epoints[i]);
			sample_projs.push_back(eprojections[i]);
		}

		// Solve for the parameters
		double Ptmp[12];
		ComputeProjectionMatrix(sample_pts, sample_projs, Ptmp);
		Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> Ptmpe(Ptmp);

		// Count the number of inliers in all correspondences
		int num_inliers = EvaluateProjectionMatrix(Ptmpe, epoints, eprojections, ransac_threshold, &error);

		if (num_inliers > max_inliers || (num_inliers == max_inliers && error < max_error))
		{
			memcpy(Pbest, Ptmp, sizeof(double) * 12);
			max_error = error;
			max_inliers = num_inliers;
		}
	}
		
	memcpy(P, Pbest, sizeof(double) * 12);

	if (max_inliers < 6)
	{
		std::cerr << "[ComputeProjectionMatrixRansac] Error: Too few inliers to continue\n";
		return -1;
	}
		
	Vec3Vec pts_final;
	Vec2Vec projs_final;
	for (int i = 0; i < num_pts; i++)
	{
		Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> Pbeste(Pbest);
		Vec3 pr = Pbeste * Vec4(epoints[i].x(), epoints[i].y(), epoints[i].z(), 1.0);
		pr /= -pr.z();
			    
		double dist = (pr.head(2) - eprojections[i]).squaredNorm();

		if (dist < ransac_threshold)
		{
			pts_final.push_back(epoints[i]);
			projs_final.push_back(eprojections[i]);
		}
	}

	// Re-estimate P from all correspondences classified as inliers
	double Plinear[12];
	ComputeProjectionMatrix(pts_final, projs_final, Plinear);

	// Refine the result
	if (max_inliers >= 6)
	{
		RefineProjectionMatrixNonlinear(pts_final, projs_final, Plinear, P);
	} else
	{
		memcpy(P, Plinear, sizeof(double) * 12);
	}

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