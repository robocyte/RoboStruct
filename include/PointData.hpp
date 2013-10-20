#pragma once

#include "EigenTools.hpp"
#include "TrackData.hpp"

struct PointData
{
	PointData()
		: m_pos()
		, m_color()
		, m_views()
	{}

    PointData(const Vec3 &pos, const Vec3 &color, const ImageKeyVector &views)
        : m_pos(pos)
        , m_color(color)
        , m_views(views)
    {}

	Vec3			m_pos;	    // 3D position of the point
	Vec3			m_color;	// Color of the point
	ImageKeyVector	m_views;	// Views/keys corresponding to this point
};
