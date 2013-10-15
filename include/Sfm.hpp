#pragma once

#include "EigenTools.hpp"

struct Camera
{
	Camera()
		: m_focal_length(0.0)
		, m_init_focal_length(0.0)
		, m_R(Mat3::Identity())
		, m_t(Vec3::Zero())
		, m_k(Vec2::Zero())
		, m_k_inv(Vec6::Zero())
	{
		m_k_inv(1) = 1.0;
	}

	double	m_focal_length;			// Focal length in pixels
	double	m_init_focal_length;	// Initial focal length from EXIF
	Mat3	m_R;					// Rotation matrix
	Vec3	m_t;					// Camera translation
	Vec2	m_k;					// Undistortion parameters
	Vec6	m_k_inv;				// Inverse undistortion parameters

	Mat3 GetIntrinsics() const;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

Vec2 SfmProjectFinal(const Vec3 &p, const Camera &cam);

Vec2 SfmProjectRD(const Vec3 &p, const Camera &cam);

Vec6 InvertDistortion(double r0, double r1, const Vec6 &k_in);

Vec2 UndistortNormalizedPoint(const Vec2 &p, const Vec6 & k_inverse);

// Refine the position of a single camera
void RefineCamera(Camera *camera, const Vec3Vec &points, const Vec2Vec &projections, bool adjust_focal);
