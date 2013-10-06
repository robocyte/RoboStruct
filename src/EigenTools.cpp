#include "EigenTools.hpp"

Mat3 AngleAxisToRotationMatrix(const Vec3 &w)
{
	Mat3 dR;
	double theta = w.squaredNorm();

	if (theta > 0.0)
	{
		theta = sqrt(theta);
		double wx = w.x() / theta, wy = w.y() / theta, wz = w.z() / theta;
		double costh = cos(theta), sinth = sin(theta);

		dR <<	      costh + wx * wx * (1.0 - costh),	wx * wy * (1.0 - costh) - wz * sinth,	 wy * sinth + wx * wz * (1.0 - costh),
				 wz * sinth + wx * wy * (1.0 - costh),		 costh + wy * wy * (1.0 - costh),	-wx * sinth + wy * wz * (1.0 - costh),
				-wy * sinth + wx * wz * (1.0 - costh),  wx * sinth + wy * wz * (1.0 - costh),		  costh + wz * wz * (1.0 - costh);
	} else
	{
		dR <<    1.0,  w.z(), -w.y(),
				-w.z(),    1.0,  w.x(),
				w.y(), -w.x(),    1.0;
	}

	return dR;
}

//Vec3 RotationMatrixToAngleAxis(const Mat3 &R)
//{
//
//}