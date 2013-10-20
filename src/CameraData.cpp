#include "CameraData.hpp"

CameraData::CameraData()
	: m_adjusted(false)
	, m_ccd_width(0)
	, m_focal(0.0)
{
	m_k[0] = m_k[1] = 0.0;
}

void CameraData::SetCCDWidth(double ccd_width)
{
	m_ccd_width = ccd_width;
}

double CameraData::GetCCDWidth()
{
	return m_ccd_width;
}

void CameraData::Finalize()
{
	//// Find the 3d positions of the image corners
	//m_img_upper_left[0] = -0.5 * m_width;
	//m_img_upper_left[1] = 0.5 * m_height;
	//m_img_upper_left[2] = -m_focal;
	//m_img_upper_left[3] = 1.0;

	//m_img_lower_left[0] = -0.5 * m_width;
	//m_img_lower_left[1] = -0.5 * m_height;
	//m_img_lower_left[2] = -m_focal;
	//m_img_lower_left[3] = 1.0;

	//m_img_upper_right[0] = 0.5 * m_width;
	//m_img_upper_right[1] = 0.5 * m_height;
	//m_img_upper_right[2] = -m_focal;
	//m_img_upper_right[3] = 1.0;

	//m_img_lower_right[0] = 0.5 * m_width;
	//m_img_lower_right[1] = -0.5 * m_height;
	//m_img_lower_right[2] = -m_focal;
	//m_img_lower_right[3] = 1.0;

	//matrix_scale(3, 1, m_img_upper_left, 1.0 / m_focal, m_img_upper_left);
 //   matrix_scale(3, 1, m_img_lower_left, 1.0 / m_focal, m_img_lower_left);
 //   matrix_scale(3, 1, m_img_upper_right, 1.0 / m_focal, m_img_upper_right);
 //   matrix_scale(3, 1, m_img_lower_right, 1.0 / m_focal, m_img_lower_right);

	// Compute the camera's rotation as a quaternion
	//matrix_to_quaternion(m_R, m_quat);
}
