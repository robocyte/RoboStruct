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
        , m_adjusted(false)
    {
        m_k_inv(1) = 1.0;
    }

    double  m_focal_length;         // Focal length in pixels
    double  m_init_focal_length;    // Initial focal length from EXIF
    Mat3    m_R;                    // Rotation matrix
    Vec3    m_t;                    // Camera translation
    Vec2    m_k;                    // Undistortion parameters
    Vec6    m_k_inv;                // Inverse undistortion parameters
    bool    m_adjusted;             // Has this camera been adjusted?

    Mat3    GetIntrinsicMatrix() const;
    Mat34   GetProjectionMatrix() const;

    Point2  ProjectFinal(const Point3 &point);
    Point2  ProjectRD(const Point3 &point);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

Vec6 InvertDistortion(double r0, double r1, const Vec6 &k_in);

Point2 UndistortNormalizedPoint(const Point2 &p, const Vec6 & k_inverse);

// Refine the position of a single camera
void RefineCamera(Camera *camera, const Point3Vec &points, const Point2Vec &projections, bool adjust_focal);
