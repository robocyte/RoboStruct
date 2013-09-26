#include <iostream>

#include "wx/log.h"

#include "matrix.h"

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
			P(2, 0) = x(8);
			P(2, 1) = x(9);
			P(2, 2) = x(10);
			P(2, 3) = 1.0;

			int position = 0;
			for (int i = 0; i < m_points.size(); ++i)
			{
				Vec3 pr = P * Vec4(m_points[i].x(), m_points[i].y(), m_points[i].z(), 1.0);
				pr.x() /= -pr.z();
				pr.y() /= -pr.z();

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
				pr.x() /= -pr.z();
				pr.y() /= -pr.z();

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
			pr.x() /= -pr.z();
			pr.y() /= -pr.z();
			    
			double dx = pr.x() - projections[i].x();
			double dy = pr.y() - projections[i].y();

			double dist = dx * dx + dy * dy;

			if (dist < threshold)
			{
				num_inliers++;
				*error += dist;
			}
		}

		return num_inliers;
	}
}

// Solve for a 3x4 projection matrix, given a set of 3D points and 2D projections
void FindProjectionMatrix(int num_pts, v3_t *points, v2_t *projections, double *P)
{
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
		A(2 * i, 0)  = Vx(points[i]);
		A(2 * i, 1)  = Vy(points[i]);
		A(2 * i, 2)  = Vz(points[i]);
		A(2 * i, 3)  = 1.0;
		
		A(2 * i, 8)  = Vx(projections[i]) * Vx(points[i]);
		A(2 * i, 9)  = Vx(projections[i]) * Vy(points[i]);
		A(2 * i, 10) = Vx(projections[i]) * Vz(points[i]);
		
		A(2 * i + 1, 4)  = Vx(points[i]);
		A(2 * i + 1, 5)  = Vy(points[i]);
		A(2 * i + 1, 6)  = Vz(points[i]);
		A(2 * i + 1, 7)  = 1.0;	

		A(2 * i + 1, 8)  = Vy(projections[i]) * Vx(points[i]);
		A(2 * i + 1, 9)  = Vy(projections[i]) * Vy(points[i]);
		A(2 * i + 1, 10) = Vy(projections[i]) * Vz(points[i]);
		
		b(2 * i + 0) = -Vx(projections[i]);
		b(2 * i + 1) = -Vy(projections[i]);
	}

	// Find the least squares result
	x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	for (int i = 0; i < 11; ++i) P[i] = x(i);
	P[11] = 1.0;
}

int FindProjectionMatrixNonlinear(int num_pts, v3_t *points, v2_t *projections, double *Pin, double *Pout)
{
    if (num_pts < 6)
	{
		std::cerr << "[FindProjectionMatrixNonlinear] Error: need at least 6 points!";
		return -1;
    }

	Vec3Vec epoints;		epoints.reserve(num_pts);
	Vec2Vec eprojections;	eprojections.reserve(num_pts);
	Vec x(11);
	
	for (int i = 0; i < 11; ++i) x(i) = Pin[i];
	for (int i = 0; i < num_pts; ++i)
	{
		epoints.push_back(Vec3(points[i].p[0], points[i].p[1], points[i].p[2]));
		eprojections.push_back(Vec2(projections[i].p[0], projections[i].p[1]));
	}

	ProjectionResidual functor(epoints, eprojections);
	Eigen::DenseIndex nfev;
	Eigen::LevenbergMarquardt<ProjectionResidual>::lmdif1(functor, x, &nfev, 1.0e-5);

	for (int i = 0; i < 11; ++i) Pout[i] = x(i); Pout[11] = 1.0;

	return 0;
}

int ComputeProjectionMatrixRansac(int num_pts, v3_t *points, v2_t *projections,
								  int ransac_rounds, double ransac_threshold,
								  double *P)
{
    if (num_pts < 6)
	{
		std::cerr << "[ComputeProjectionMatrixRansac] Error: need at least 6 points!";
		return -1;
    }

	double max_error = 0.0;
	double Pbest[12];
	int num_inliers = 0, num_inliers_new = 0;
	double Plinear[12];
	double thresh_sq = ransac_threshold * ransac_threshold;
	double error = 0.0;
	int num_inliers_polished = 0;

	Vec3Vec epoints;
	Vec2Vec eprojections;

	for (int i = 0; i < num_pts; ++i)
	{
		epoints.push_back(Vec3(Vx(points[i]), Vy(points[i]), Vz(points[i])));
		eprojections.push_back(Vec2(Vx(projections[i]), Vy(projections[i])));
	}

	int max_inliers = 0;
	for (int round = 0; round < ransac_rounds; round++)
	{
		std::vector<v3_t> sample_pts;	sample_pts.reserve(6);
		std::vector<v2_t> sample_projs;	sample_projs.reserve(6);

		auto sample_indices = util::GetNRandomIndices(6, num_pts);
		for (int i : sample_indices)
		{
			sample_pts.push_back(points[i]);
			sample_projs.push_back(projections[i]);
		}

		// Solve for the parameters
		double Ptmp[12];
		FindProjectionMatrix(6, sample_pts.data(), sample_projs.data(), Ptmp);
		Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> Ptmpe(Ptmp);

		// Count the number of inliers
		num_inliers = EvaluateProjectionMatrix(Ptmpe, epoints, eprojections, thresh_sq, &error);

		if (num_inliers > max_inliers || (num_inliers == max_inliers && error < max_error))
		{
			memcpy(Pbest, Ptmp, sizeof(double) * 12);
			max_error = error;
			max_inliers = num_inliers;
		}
	}
		
	memcpy(P, Pbest, sizeof(double) * 12);

	//wxLogMessage("[ComputeProjectionMatrixRansac] # of inliers = %i (out of %i)\n", max_inliers, num_pts);
	//wxLogMessage("[ComputeProjectionMatrixRansac] Error = %0.3f\n", sqrt(max_error / max_inliers));

	if (max_inliers < 6)
	{
		std::cerr << "[ComputeProjectionMatrixRansac] Error: Too few inliers to continue\n";
		return -1;
	}
		
	// Do the final least squares minimization
	num_inliers = 0;
	v3_t * pts_final = (v3_t *) malloc(sizeof(v3_t) * max_inliers);
	v2_t *projs_final = (v2_t *) malloc(sizeof(v2_t) * max_inliers);
		
	for (int i = 0; i < num_pts; i++)
	{
		double pt[4] = { Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0 };
		double pr[3];
		double dx, dy, dist;
		    
		matrix_product341(Pbest, pt, pr);

		pr[0] /= -pr[2];
		pr[1] /= -pr[2];
		    
		dx = pr[0] - Vx(projections[i]);
		dy = pr[1] - Vy(projections[i]);

		dist = dx * dx + dy * dy;

		if (dist < thresh_sq)
		{
			pts_final[num_inliers] = points[i];
			projs_final[num_inliers] = projections[i];
			num_inliers++;
		}
	}

	FindProjectionMatrix(max_inliers, pts_final, projs_final, Plinear);

	for (int i = 0; i < num_pts; i++)
	{
		double pt[4] = { Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0 };
		double pr[3];
		double dx, dy, dist;
		    
		matrix_product341(Plinear, pt, pr);

		pr[0] /= -pr[2];
		pr[1] /= -pr[2];
		    
		dx = pr[0] - Vx(projections[i]);
		dy = pr[1] - Vy(projections[i]);

		dist = dx * dx + dy * dy;

		if (dist < thresh_sq) num_inliers_new++;
	}

	if (num_inliers_new < max_inliers)
	{
		std::cerr << "[ComputeProjectionMatrixRansac] Reverting to old solution (" << num_inliers_new << " < " << max_inliers << ")\n";
		memcpy(Plinear, Pbest, 12 * sizeof(double));
	}
		
	error = 0.0;
	for (int i = 0; i < max_inliers; i++)
	{
		double pt[4] = { Vx(pts_final[i]), Vy(pts_final[i]), Vz(pts_final[i]), 1.0 };
		double pr[3];
		double dx, dy, dist;
		    
		matrix_product341(Plinear, pt, pr);
		pr[0] /= pr[2];
		pr[1] /= pr[2];
		    
		dx = pr[0] - Vx(projs_final[i]);
		dy = pr[1] - Vy(projs_final[i]);

		dist = dx * dx + dy * dy;
		error += dist;
	}
		
	//std::cerr << "[ComputeProjectionMatrixRansac] Old error: " << sqrt(error / max_inliers) << std::endl;

	// Polish the result
	if (max_inliers >= 6)
	{
		int num_inliers_polished = 0;
		FindProjectionMatrixNonlinear(max_inliers, pts_final, projs_final, Plinear, P);

		// Check that the number of inliers hasn't gone down
		num_inliers_polished = 0;
		for (int i = 0; i < num_pts; i++)
		{
			double pt[4] = { Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0 };
			double pr[3];
			double dx, dy, dist;
			    
			matrix_product341(P, pt, pr);

			pr[0] /= -pr[2];
			pr[1] /= -pr[2];
			    
			dx = pr[0] - Vx(projections[i]);
			dy = pr[1] - Vy(projections[i]);

			dist = dx * dx + dy * dy;

			if (dist < thresh_sq) num_inliers_polished++;
		}

		if (num_inliers_polished < max_inliers)
		{
			std::cerr << "[ComputeProjectionMatrixRansac] Decreased number of inliers (" << num_inliers_polished << " < " << max_inliers << "), reverting" << std::endl;
			memcpy(P, Plinear, sizeof(double) * 12);		
		}
	} else
	{
		memcpy(P, Plinear, sizeof(double) * 12);
	}

	error = 0.0;
	for (int i = 0; i < max_inliers; i++)
	{
		double pt[4] = { Vx(pts_final[i]), Vy(pts_final[i]), Vz(pts_final[i]), 1.0 };
		double pr[3];
		double dx, dy, dist;
		    
		matrix_product341(P, pt, pr);

		pr[0] /= -pr[2];
		pr[1] /= -pr[2];
		    
		dx = pr[0] - Vx(projs_final[i]);
		dy = pr[1] - Vy(projs_final[i]);

		dist = dx * dx + dy * dy;
		error += dist;
	}
		
	//std::cerr << "[ComputeProjectionMatrixRansac] New error: " << sqrt(error / max_inliers) << std::endl;

	free(pts_final);
	free(projs_final);

	return max_inliers;
}

//int ComputeProjectionMatrixRansac(int num_pts, v3_t *points, v2_t *projections,
//								  int ransac_rounds, double ransac_threshold,
//								  double *P)
//{
//    if (num_pts < 6)
//	{
//		std::cerr << "[ComputeProjectionMatrixRansac] Error: need at least 6 points!";
//		return -1;
//    }
//
//	const int MIN_PTS = 6;
//	int *inliers = (int *) malloc(sizeof(int) * num_pts);
//	int max_inliers = 0;
//	double max_error = 0.0;
//	double Pbest[12];
//	int num_inliers = 0, num_inliers_new = 0;
//	std::vector<v3_t> pts_final;
//	std::vector<v2_t> projs_final;
//	double Plinear[12];
//	double Rinit[9];
//	double triangular[9], orthogonal[9];
//	int neg, sign;
//	double thresh_sq = ransac_threshold * ransac_threshold;
//	double error = 0.0;
//	int num_inliers_polished = 0;
//
//	for (int round = 0; round < ransac_rounds; round++)
//	{
//		num_inliers = 0;
//		std::vector<v3_t> sample_pts;	sample_pts.reserve(MIN_PTS);
//		std::vector<v2_t> sample_projs;	sample_projs.reserve(MIN_PTS);
//
//		auto sample_indices = util::GetNRandomIndices(6, num_pts);
//		for (int idx : sample_indices)
//		{
//			sample_pts.push_back(points[idx]);
//			sample_projs.push_back(projections[idx]);
//		}
//
//		// Solve for the parameters
//		double Ptmp[12];
//		FindProjectionMatrix(MIN_PTS, sample_pts.data(), sample_projs.data(), Ptmp);
//
//		Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> Ptmpe(Ptmp);
//
//		// Fix the sign on the P matrix
//		Mat tri = Ptmpe.topLeftCorner<3, 4>().colwise().reverse().transpose().householderQr().matrixQR().triangularView<Eigen::Upper>();
//		tri.reverseInPlace();
//		tri.transposeInPlace();
//		tri.col(2) *= -1;
//
//		// Check the parity along the diagonal
//		neg = (tri(0, 0) < 0.0) + (tri(1, 1) < 0.0) + (tri(2, 2) < 0.0);
//
//		if ((neg % 2) == 1)	sign = -1;
//		else				sign = 1;
//
//		// Count the number of inliers
//		error = 0.0;
//		std::vector<v3_t> pts_inl;
//		std::vector<v2_t> projs_inl;
//		for (int i = 0; i < num_pts; i++)
//		{
//			Vec4 pt(Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0);
//			Vec3 pr = Ptmpe * pt;
//
//			// Check cheirality
//			if (sign * pr.z() > 0.0) continue;
//
//			pr.x() /= -pr.z();
//			pr.y() /= -pr.z();
//			    
//			double dx = pr.x() - Vx(projections[i]);
//			double dy = pr.y() - Vy(projections[i]);
//			double dist = dx * dx + dy * dy;
//
//			if (dist < thresh_sq)
//			{
//				pts_inl.push_back(points[i]);
//				projs_inl.push_back(projections[i]);
//
//				inliers[num_inliers] = i;
//				num_inliers++;
//				error += dist;
//			}
//		}
//			    
//		if (num_inliers > max_inliers)
//		{
//			pts_final.clear();
//			projs_final.clear();
//			pts_final = pts_inl;
//			projs_final = projs_inl;
//			memcpy(P, Ptmp, sizeof(double) * 12);
//			max_error = error;
//			max_inliers = num_inliers;
//		}
//	}
//		
//	num_inliers = (int)pts_final.size();
//
//	printf("[ComputeProjectionMatrixRansac] # of inliers = %d (out of %d)\n", max_inliers, num_pts);
//	printf("[ComputeProjectionMatrixRansac] Error = %0.3f\n", sqrt(max_error / max_inliers));
//
//	if (max_inliers < 6)
//	{
//		std::cerr << "[ComputeProjectionMatrixRansac] Error: Too few inliers to continue\n";
//		free(inliers);
//		return -1;
//	}
//		
//	// Do the final least squares minimization
//	FindProjectionMatrix(max_inliers, pts_final.data(), projs_final.data(), Plinear);
//
//	// Fix the sign on the P matrix
//	memcpy(Rinit + 0, Plinear + 0, 3 * sizeof(double));
//	memcpy(Rinit + 3, Plinear + 4, 3 * sizeof(double));
//	memcpy(Rinit + 6, Plinear + 8, 3 * sizeof(double));
//
//	dgerqf_driver(3, 3, Rinit, triangular, orthogonal);	    
//		
//	// Check the parity along the diagonal
//	neg =	(triangular[0] < 0.0) + (triangular[4] < 0.0) + (triangular[8] < 0.0);
//
//	if ((neg % 2) == 1)	sign = -1;
//	else				sign = 1;
//
//	for (int i = 0; i < num_pts; i++)
//	{
//		double pt[4] = { Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0 };
//		double pr[3];
//		double dx, dy, dist;
//		    
//		matrix_product341(Plinear, pt, pr);
//
//		if (sign * pr[2] > 0.0) continue;
//
//		pr[0] /= -pr[2];
//		pr[1] /= -pr[2];
//		    
//		dx = pr[0] - Vx(projections[i]);
//		dy = pr[1] - Vy(projections[i]);
//
//		dist = dx * dx + dy * dy;
//
//		if (dist < thresh_sq) num_inliers_new++;
//	}
//
//	if (num_inliers_new < max_inliers)
//	{
//		std::cout << "[ComputeProjectionMatrixRansac] Reverting to old solution\n";
//		memcpy(Plinear, Pbest, 12 * sizeof(double));
//	}
//		
//	error = 0.0;
//	for (int i = 0; i < max_inliers; i++)
//	{
//		double pt[4] = { Vx(pts_final[i]), Vy(pts_final[i]), Vz(pts_final[i]), 1.0 };
//		double pr[3];
//		double dx, dy, dist;
//		    
//		matrix_product341(Plinear, pt, pr);
//		pr[0] /= pr[2];
//		pr[1] /= pr[2];
//		    
//		dx = pr[0] - Vx(projs_final[i]);
//		dy = pr[1] - Vy(projs_final[i]);
//
//		dist = dx * dx + dy * dy;
//		error += dist;
//	}
//		
//	std::cout << "[ComputeProjectionMatrixRansac] Old error: " << sqrt(error / max_inliers) << std::endl;
//
//	// Polish the result
//	if (max_inliers >= 6)
//	{
//		int num_inliers_polished = 0;
//		FindProjectionMatrixNonlinear(max_inliers, pts_final.data(), projs_final.data(), Plinear, P);
//
//		// Fix the sign on the P matrix
//		memcpy(Rinit + 0, P + 0, 3 * sizeof(double));
//		memcpy(Rinit + 3, P + 4, 3 * sizeof(double));
//		memcpy(Rinit + 6, P + 8, 3 * sizeof(double));
//
//		dgerqf_driver(3, 3, Rinit, triangular, orthogonal);	    
//
//		// Check the parity along the diagonal
//		neg =	(triangular[0] < 0.0) + (triangular[4] < 0.0) + (triangular[8] < 0.0);
//
//		if ((neg % 2) == 1)	sign = -1;
//		else				sign = 1;
//
//		// Check that the number of inliers hasn't gone down
//		num_inliers_polished = 0;
//		for (int i = 0; i < num_pts; i++)
//		{
//			double pt[4] = { Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0 };
//			double pr[3];
//			double dx, dy, dist;
//			    
//			matrix_product341(P, pt, pr);
//
//			if (sign * pr[2] > 0.0) continue;
//
//			pr[0] /= -pr[2];
//			pr[1] /= -pr[2];
//			    
//			dx = pr[0] - Vx(projections[i]);
//			dy = pr[1] - Vy(projections[i]);
//
//			dist = dx * dx + dy * dy;
//
//			if (dist < thresh_sq) num_inliers_polished++;
//		}
//
//		if (num_inliers_polished < max_inliers)
//		{
//			std::cout << "[ComputeProjectionMatrixRansac] Decreased number of inliers (" << num_inliers_polished << " < " << max_inliers << "), reverting" << std::endl;
//			memcpy(P, Plinear, sizeof(double) * 12);		
//		}
//	} else
//	{
//		memcpy(P, Plinear, sizeof(double) * 12);
//	}
//
//	error = 0.0;
//	for (int i = 0; i < max_inliers; i++)
//	{
//		double pt[4] = { Vx(pts_final[i]), Vy(pts_final[i]), Vz(pts_final[i]), 1.0 };
//		double pr[3];
//		double dx, dy, dist;
//		    
//		matrix_product341(P, pt, pr);
//
//		pr[0] /= -pr[2];
//		pr[1] /= -pr[2];
//		    
//		dx = pr[0] - Vx(projs_final[i]);
//		dy = pr[1] - Vy(projs_final[i]);
//
//		dist = dx * dx + dy * dy;
//		error += dist;
//	}
//		
//	std::cout << "[ComputeProjectionMatrixRansac] New error: " << sqrt(error / max_inliers) << std::endl;
//
//	free(inliers);
//
//	return max_inliers;
//}

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