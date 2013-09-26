#pragma once

struct KeyPoint
{
	KeyPoint()
		: m_x(0.0f)
		, m_y(0.0f)
		, m_r(0)
		, m_g(0)
		, m_b(0)
		, m_extra(-1)
		, m_track(-1)
	{}

	KeyPoint(float x, float y, unsigned char r = 0, unsigned char g = 0, unsigned char b = 0)
		: m_x(x)
		, m_y(y)
		, m_r(r)
		, m_g(g)
		, m_b(b)
		, m_extra(-1)
		, m_track(-1)
	{}

	float m_x, m_y;					// Subpixel location
	unsigned char m_r, m_g, m_b;	// Color of this key
	int m_extra;					// 4 bytes of extra storage
	int m_track;					// Track index this point corresponds to
};
