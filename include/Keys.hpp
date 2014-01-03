#pragma once

#include "EigenTools.hpp"

struct KeyPoint
{
    KeyPoint() = default;

    KeyPoint(float x, float y, unsigned char r = 0, unsigned char g = 0, unsigned char b = 0)
        : m_coords(Point2(x, y))
        , m_color(Vec3(r, g, b) / 255.0)
    {}

    Point2 m_coords = Point2::Zero();   // Subpixel location
    Vec3   m_color  = Vec3::Zero();     // Color of this key
    int    m_extra  = -1;               // 4 bytes of extra storage
    int    m_track  = -1;               // Track index this point corresponds to

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
