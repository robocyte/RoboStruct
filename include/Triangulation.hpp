#pragma once

#include "Sfm.hpp"

struct Observation
{
	explicit Observation(const Vec2 &point, const Mat3 &R, const Vec3 &t)
		: m_point(point)
		, m_R(R)
		, m_t(t)
	{}

    explicit Observation(const Vec2 &point, const Camera &camera)
        : m_point(point)
        , m_R(camera.m_R)
        , m_t(-(camera.m_R * camera.m_t))
    {}

	Vec2 m_point;
	Mat3 m_R;
	Vec3 m_t;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

typedef std::vector<Observation>	Observations;

// Compute the angle in radians between two rays
double ComputeRayAngle(Vec2 p, Vec2 q, const Camera &cam1, const Camera &cam2);

// Check if a point p lies in front of a camera
bool CheckCheirality(const Vec3 p, const Camera &cam);

// Project a 3d point p onto an image
Vec2 Project(const Vec3 &p, const Observation &observation);

// Find the point with the smallest squared projection error
Vec3 Triangulate(const Observations &observations, double *error = nullptr, bool optimize = false);

// Given an essential matrix and point correspondences, find R and t
bool FindExtrinsics(const Mat3 &E, const Vec2Vec &pts1, const Vec2Vec &pts2, Mat3 *R, Vec3 *t);
