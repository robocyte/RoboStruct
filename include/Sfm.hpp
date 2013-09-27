#pragma once

#include "EigenTools.hpp"

typedef struct
{
	double R[9];		// Rotation
	double t[3];		// Translation
	double f;			// Focal length
	double init_f;		// Initial focal length from EXIF
	double k[2];		// Undistortion parameters
	double k_inv[6];	// Inverse undistortion parameters
} camera_params_t;

struct ECamera
{
	ECamera()
		: m_focal_length(0.0)
		, m_R(Mat3::Identity())
		, m_t(Vec3::Zero())
		, m_k(Vec2::Zero())
	{}

	double	m_focal_length;		// Focal length in pixels
	Mat3	m_R;
	Vec3	m_t;
	Vec2	m_k;				// Undistortion parameters

	Mat3 GetIntrinsics() const;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

Vec2 SfmProjectFinal(const Vec3 &p, const ECamera &cam);

Vec2 SfmProjectRD(const Vec3 &p, const ECamera &cam);

#include "vector.h"
#include "matrix.h"

// Refine the position of a single camera
void RefineCamera(int num_points, v3_t *points, v2_t *projs, camera_params_t *camera, bool adjust_focal);
