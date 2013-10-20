#pragma once

#include <string>

class CameraData
{
public:
	CameraData();

	bool			m_adjusted;					// Has this camera been adjusted?
	std::string		m_camera_model;				// Model name
	std::string		m_camera_make;				// Maker name
	double			m_ccd_width;				// CCD width (mm)
	double			m_focal;					// Focal length (px)
	int				m_width, m_height;
	double			m_k[2];						// Distortion parameters
	double			m_R[9], m_t[3];				// Extrinsics

	void	SetCCDWidth(double ccd_width);
	double	GetCCDWidth();
	void	Finalize();
};
