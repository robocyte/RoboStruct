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

	Vec3			m_pos;	    // 3D position of the point
	Vec3			m_color;	// Color of the point
	ImageKeyVector	m_views;	// Views/keys corresponding to this point
};
