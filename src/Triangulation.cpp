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
				auto reprojected_point = Project(x, observation);

				fvec(position + 0) = observation.m_point.x() - reprojected_point.x();
				fvec(position + 1) = observation.m_point.y() - reprojected_point.y();

				position += 2;
			}
			
			return 0;
		}

		Observations m_observations;
	};
}

double ComputeRayAngle(Vec2 p, Vec2 q, const ECamera &cam1, const ECamera &cam2)
{
	Mat3 K1_inv = cam1.GetIntrinsics().inverse();
	Mat3 K2_inv = cam2.GetIntrinsics().inverse();

	Vec3 p3n = K1_inv * Vec3(p.x(), p.y(), 1.0);
	Vec3 q3n = K2_inv * Vec3(q.x(), q.y(), 1.0);

	Vec2 pn(p3n.x() / p3n.z(), p3n.y() / p3n.z());
	Vec2 qn(q3n.x() / q3n.z(), q3n.y() / q3n.z());

	Vec3 p_w = cam1.m_R.transpose() * Vec3(pn.x(), pn.y(), -1.0);
	Vec3 q_w = cam2.m_R.transpose() * Vec3(qn.x(), qn.y(), -1.0);

	// Compute the angle between the rays
	double dot = p_w.dot(q_w);
	double mag = p_w.norm() * q_w.norm();

	return acos(util::clamp((dot / mag), (-1.0 + 1.0e-8), (1.0 - 1.0e-8)));
}

bool CheckCheirality(const Vec3 p, const ECamera &cam)
{
	Vec3 pt = cam.m_R * (p - cam.m_t);
	return (pt.z() < 0.0);
}

Vec2 Project(const Vec3 &p, const Observation &observation)
{
	Vec3 proj = observation.m_R * p + observation.m_t;
	return Vec2(proj.x() / proj.z(), proj.y() / proj.z());
}

Vec3 Triangulate(const Observations &observations, double *error, bool optimize)
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
		Eigen::DenseIndex nfev;
		Eigen::LevenbergMarquardt<TriangulationResidual>::lmdif1(functor, x, &nfev, 1.0e-10);
	}

	if (error != nullptr)
	{
		Vec residuals(2 * num_points);
		functor(x, residuals);
		*error = residuals.squaredNorm();
	}

	return x;
}

bool FindExtrinsics(const Mat3 &E, const Vec2Vec &pts1, const Vec2Vec &pts2, Mat3 *R, Vec3 *t)
{
	int num_correspondences = static_cast<int>(pts1.size());

	// Put first camera at origin
	Mat3 R0;	R0.setIdentity();
	Vec3 t0;	t0.setZero();

	// Find the SVD of E
	Eigen::JacobiSVD<Mat3> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Mat3 U	= svd.matrixU();
	Mat3 Vt	= svd.matrixV().transpose();

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

		Vec3 Q = Triangulate(observations);
		Vec3 PQ = (Ra * Q) + tu;

		if (Q.z() > 0)	c1_pos++;
		else			c1_neg++;
        
		if (PQ.z() > 0)	c2_pos++;
		else			c2_neg++;
	}

	if (c1_pos < c1_neg && c2_pos < c2_neg)
	{
		*R = Ra;
		*t = tu;
	} else if (c1_pos > c1_neg && c2_pos > c2_neg)
	{
		*R = Ra;
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

			Vec3 Q = Triangulate(observations);
			Vec3 PQ = (Rb * Q) + tu;

			if (Q.z() > 0)	c1_pos++;
			else			c1_neg++;
        
			if (PQ.z() > 0)	c2_pos++;
			else			c2_neg++;
		}

		if (c1_pos < c1_neg && c2_pos < c2_neg)
		{
			*R = Rb;
			*t = tu;
		} else if (c1_pos > c1_neg && c2_pos > c2_neg)
		{
			*R = Rb;
			*t = -tu;
		} else
		{
			std::cerr << "[FindExtrinsics] Error: no case found!";
			return false;
		}
	}

	return true;
}
