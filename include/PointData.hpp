#pragma once

#include "TrackData.hpp"

struct PointData
{
	PointData()
		: m_pos()
		, m_color()
		, m_views()
	{}

	double			m_pos[3];	// 3D position of the point
	float			m_color[3];	// Color of the point
	ImageKeyVector	m_views;	// Views/keys corresponding to this point
};
