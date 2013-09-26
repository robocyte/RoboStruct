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
	double			m_quat[4];					// Camera rotation as a normalized quaternion
	double			m_Pmatrix[12];				// Projection matrix
	float			m_gl_rot_mat[4][4];			// OpenGL rotation matrix
	bool			m_constrained[7];
	double			m_constraints[7];
	double			m_constraint_weights[7];
	double			m_img_upper_left[4], m_img_lower_left[4],
					m_img_upper_right[4], m_img_lower_right[4];

	void	SetCCDWidth(double ccd_width);
	double	GetCCDWidth();
	void	Finalize();
	void	GetIntrinsics(double *K) const;
};
