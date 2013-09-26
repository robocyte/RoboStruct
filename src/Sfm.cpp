#include "Sfm.hpp"

#include <iostream>

namespace
{
	Mat3 AngleAxisToRotationMatrix(const Vec3 &w)
	{
		Mat3 dR;
		double theta = w.squaredNorm();

		if (theta > 0.0)
		{
			theta = sqrt(theta);
			double wx = w.x() / theta, wy = w.y() / theta, wz = w.z() / theta;
			double costh = cos(theta), sinth = sin(theta);

			dR <<		  costh + wx * wx * (1.0 - costh),	wx * wy * (1.0 - costh) - wz * sinth,	 wy * sinth + wx * wz * (1.0 - costh),
					 wz * sinth + wx * wy * (1.0 - costh),		 costh + wy * wy * (1.0 - costh),	-wx * sinth + wy * wz * (1.0 - costh),
					-wy * sinth + wx * wz * (1.0 - costh),  wx * sinth + wy * wz * (1.0 - costh),		  costh + wz * wz * (1.0 - costh);
		} else
		{
			dR <<    1.0,  w.z(), -w.y(),
				  -w.z(),    1.0,  w.x(),
				   w.y(), -w.x(),    1.0;
		}

		return dR;
	}

	struct CameraResidual : LMFunctor<double>
	{
		CameraResidual(const ECamera &camera, const Vec3Vec &points, const Vec2Vec &projections, bool adjust_focal)
			: LMFunctor<double>(adjust_focal ? 9 : 6, adjust_focal ? 2 * points.size() + 3 : 2 * points.size())
			, m_camera(camera)
			, m_points(points)
			, m_projections(projections)
			, m_adjust_focal(adjust_focal)
			, m_constrain_focal_weight(0.0001 * points.size())
			, m_constrain_distortion_weight(100.0 * points.size())
		{}

		int operator()(const Vec &x, Vec &fvec) const
		{
			int num_points = static_cast<int>(m_points.size());

			for (int i = 0; i < num_points; ++i)
			{
				Vec2 projection = ProjectPoint(x, m_points[i]);

				fvec(2 * i + 0) = m_projections[i].x() - projection.x();
				fvec(2 * i + 1) = m_projections[i].y() - projection.y();
			}

			if (m_adjust_focal)
			{
				fvec(2 * num_points + 0) = m_constrain_focal_weight * (m_camera.m_focal_length - x(6));
				fvec(2 * num_points + 1) = -m_constrain_distortion_weight * x(7);
				fvec(2 * num_points + 2) = -m_constrain_distortion_weight * x(8);
			}

			return 0;
		}

		Vec2 ProjectPoint(const Vec &x, const Vec3 &point) const
		{
			double focal_length = 0.0;
			if (m_adjust_focal)	focal_length = x(6);
			else				focal_length = m_camera.m_focal_length;

			// Compute rotation update
			Mat3 Rnew = AngleAxisToRotationMatrix(x.segment(3, 3)) * m_camera.m_R;

			// Project
			Vec3 tmp = Rnew * (point - x.head(3));
			Vec2 projection(tmp.x() / -tmp.z(), tmp.y() / -tmp.z());

			if (m_adjust_focal)
			{
				// Apply radial distortion
				double r = projection.squaredNorm() / (focal_length * focal_length);
				double factor = 1.0 + x(7) * r + x(8) * r * r;

				projection *= factor;
			}

			return projection * focal_length;
		}

		ECamera m_camera;
		Vec3Vec	m_points;
		Vec2Vec	m_projections;

		bool	m_adjust_focal;

		double	m_constrain_focal_weight;
		double	m_constrain_distortion_weight;
	};
}

Mat3 ECamera::GetIntrinsics() const
{
	Mat3 K;
	K.setIdentity();
	K(0, 0) = K(1, 1) = m_focal_length;

	return K;
}

Vec2 SfmProjectFinal(const Vec3 &p, const ECamera &cam)
{
	// HZ p. 153f
	Vec3 tmp = cam.m_R * (p - cam.m_t);
	Vec2 projected(tmp.x() / -tmp.z(), tmp.y() / -tmp.z());

	// Compute radial distortion
	double r2 = projected.squaredNorm() / (cam.m_focal_length * cam.m_focal_length);
	double factor = 1.0 + cam.m_k[0] * r2 + cam.m_k[1] * r2 * r2;	// Taylor expansion for L(r)

    return projected * factor * cam.m_focal_length;
}

Vec2 SfmProjectRD(const Vec3 &p, const ECamera &cam)
{
	// HZ p. 153f
	Vec3 tmp = cam.m_R * (p - cam.m_t);
	Vec2 projected(-tmp.x() * cam.m_focal_length / tmp.z(), -tmp.y() * cam.m_focal_length / tmp.z());

	// Compute radial distortion
	double r2 = projected.squaredNorm() / (cam.m_focal_length * cam.m_focal_length);
	double factor = 1.0 + cam.m_k[0] * r2 + cam.m_k[1] * r2 * r2;	// Taylor expansion for L(r)

    return projected * factor;
}

//void RefineCamera(ECamera *camera, const Vec3Vec &points, const Vec2Vec &projections, bool adjust_focal)
void RefineCamera(int num_points, v3_t *points, v2_t *projs, camera_params_t *camera, bool adjust_focal)
{
	ECamera cam;
	cam.m_focal_length = camera->f;
	cam.m_k << camera->k[0], camera->k[1];
	cam.m_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera->R);
	cam.m_t = Eigen::Map<Vec3>(camera->t);

	Vec3Vec epoints;		epoints.reserve(num_points);
	Vec2Vec eprojections;	eprojections.reserve(num_points);
	for (int i = 0; i < num_points; ++i)
	{
		epoints.push_back(Vec3(points[i].p[0], points[i].p[1], points[i].p[2]));
		eprojections.push_back(Vec2(projs[i].p[0], projs[i].p[1]));
	}

	Vec x(adjust_focal ? 9 : 6);
	if (adjust_focal)	x << camera->t[0], camera->t[1], camera->t[2], 0.0, 0.0, 0.0, camera->f, camera->k[0], camera->k[1];
	else				x << camera->t[0], camera->t[1], camera->t[2], 0.0, 0.0, 0.0;

	CameraResidual functor(cam, epoints, eprojections, adjust_focal);
	Eigen::DenseIndex nfev;
	Eigen::LevenbergMarquardt<CameraResidual>::lmdif1(functor, x, &nfev, 1.0e-12);

	// Copy out the parameters
	double xb[6];
	for (int i = 0; i < 6; ++i) xb[i] = x(i);
	memcpy(camera->t, xb + 0, 3 * sizeof(double));
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Rne(camera->R);
	Rne = AngleAxisToRotationMatrix(x.segment(3, 3)) * cam.m_R;;

	if (adjust_focal)
	{
		camera->f		= x(6);
		camera->k[0]	= x(7);
		camera->k[1]	= x(8);
	}
}
