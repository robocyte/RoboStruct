#pragma once

#include "EigenTools.hpp"

struct KeyPoint
{
	KeyPoint()
        : m_coords(Point2::Zero())
        , m_color(Vec3::Zero())
		, m_extra(-1)
		, m_track(-1)
	{}

	KeyPoint(float x, float y, unsigned char r = 0, unsigned char g = 0, unsigned char b = 0)
		: m_coords(Point2(x, y))
		, m_color(Vec3(r, g, b) / 255.0)
		, m_extra(-1)
		, m_track(-1)
	{}

    Point2 m_coords;	   				// Subpixel location
	Vec3 m_color;               	// Color of this key
	int m_extra;					// 4 bytes of extra storage
	int m_track;					// Track index this point corresponds to
};
