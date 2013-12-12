#pragma once

#include "EigenTools.hpp"

struct Camera
{
    Camera()
    {
        m_k_inv(1) = 1.0;
    }

    double  m_focal_length = 0.0;       // Focal length in pixels
    double  m_init_focal_length = 0.0;  // Initial focal length from EXIF
    Mat3    m_R = Mat3::Identity();     // Rotation matrix
    Vec3    m_t = Vec3::Zero();         // Camera translation
    Vec2    m_k = Vec2::Zero();         // Undistortion parameters
    Vec6    m_k_inv = Vec6::Zero();     // Inverse undistortion parameters
    bool    m_adjusted = false;         // Has this camera been adjusted?

    Mat3    GetIntrinsicMatrix() const;
    Mat34   GetProjectionMatrix() const;

    Point2  Project(const Point3 &point) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

Vec6 InvertDistortion(double r0, double r1, const Vec6 &k_in);

Point2 UndistortNormalizedPoint(const Point2 &p, const Vec6 & k_inverse);

// Refine the position of a single camera
void RefineCamera(Camera *camera, const Point3Vec &points, const Point2Vec &projections, bool adjust_focal);
