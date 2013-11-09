#pragma once

#include "Sfm.hpp"

struct Observation
{
    explicit Observation(const Point2 &point, const Mat3 &R, const Vec3 &t)
        : m_point(point)
        , m_R(R)
        , m_t(t)
    {}

    explicit Observation(const Point2 &point, const Camera &camera)
        : m_point(point)
        , m_R(camera.m_R)
        , m_t(-(camera.m_R * camera.m_t))
    {}

    Point2 m_point;
    Mat3 m_R;
    Vec3 m_t;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

typedef std::vector<Observation> Observations;

// Compute the angle in radians between two rays
double ComputeRayAngle(Point2 p, Point2 q, const Camera &cam1, const Camera &cam2);

// Check if a point p lies in front of a camera
bool CheckCheirality(const Point3 p, const Camera &cam);

// Project a 3d point p onto an image
Point2 Project(const Point3 &p, const Observation &observation);

// Find the point with the smallest squared projection error
Point3 Triangulate(const Observations &observations, double *error = nullptr, bool optimize = false);

// Given an essential matrix and point correspondences, find R and t
bool FindExtrinsics(const Mat3 &E, const Point2Vec &pts1, const Point2Vec &pts2, Mat3 *R, Point3 *t);
