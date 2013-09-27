#include "matrix.h"

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
	// Compute projection matrix
	double K[9];
	double Ptmp[12] = 
	{	m_R[0], m_R[1], m_R[2], m_t[0],
		m_R[3], m_R[4], m_R[5], m_t[1],
		m_R[6], m_R[7], m_R[8], m_t[2]};

	this->GetIntrinsics(K);

	matrix_product(3, 3, 3, 4, K, Ptmp, m_Pmatrix);

	// Find the 3d positions of the image corners
	m_img_upper_left[0] = -0.5 * m_width;
	m_img_upper_left[1] = 0.5 * m_height;
	m_img_upper_left[2] = -m_focal;
	m_img_upper_left[3] = 1.0;

	m_img_lower_left[0] = -0.5 * m_width;
	m_img_lower_left[1] = -0.5 * m_height;
	m_img_lower_left[2] = -m_focal;
	m_img_lower_left[3] = 1.0;

	m_img_upper_right[0] = 0.5 * m_width;
	m_img_upper_right[1] = 0.5 * m_height;
	m_img_upper_right[2] = -m_focal;
	m_img_upper_right[3] = 1.0;

	m_img_lower_right[0] = 0.5 * m_width;
	m_img_lower_right[1] = -0.5 * m_height;
	m_img_lower_right[2] = -m_focal;
	m_img_lower_right[3] = 1.0;

	matrix_scale(3, 1, m_img_upper_left, 1.0 / m_focal, m_img_upper_left);
    matrix_scale(3, 1, m_img_lower_left, 1.0 / m_focal, m_img_lower_left);
    matrix_scale(3, 1, m_img_upper_right, 1.0 / m_focal, m_img_upper_right);
    matrix_scale(3, 1, m_img_lower_right, 1.0 / m_focal, m_img_lower_right);

	// Compute the camera's rotation as a quaternion
	matrix_to_quaternion(m_R, m_quat);
}

void CameraData::GetIntrinsics(double *K) const
{
	K[0] = m_focal;	K[1] = 0.0;		K[2] = 0.0;
	K[3] = 0.0;		K[4] = m_focal;	K[5] = 0.0;
	K[6] = 0.0;		K[7] = 0.0;		K[8] = 1.0;
}
