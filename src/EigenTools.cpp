#include "EigenTools.hpp"
#include "Utilities.hpp"

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

Vec3 RotationMatrixToAngleAxis(const Mat3 &R)
{
	Vec3 angle_axis;
	angle_axis(0) = R(2, 1) - R(1, 2);
	angle_axis(1) = R(0, 2) - R(2, 0);
	angle_axis(2) = R(1, 0) - R(0, 1);

	double costheta = std::min(std::max((R.trace() - 1.0) / 2.0, -1.0), 1.0);
	double sintheta = std::min(angle_axis.norm() / 2.0, 1.0);

	double theta = atan2(sintheta, costheta);
	double threshold = 1e-12;

	// Case 1: sin(theta) is large
	if ((sintheta > threshold) || (sintheta < -threshold))
	{
		return angle_axis * theta / (2.0 * sintheta);
	}

	// Case 2: theta ~ 0
	if (costheta > 0.0)
	{
		return angle_axis * 0.5;
	}

	// Case 3: theta ~ pi
	double inv_one_minus_costheta = 1.0 / (1.0 - costheta);
	for (int i = 0; i < 3; ++i)
	{
		angle_axis[i] = theta * sqrt((R(i, i) - costheta) * inv_one_minus_costheta);
		if (((sintheta < 0.0) && (angle_axis[i] > 0.0)) || ((sintheta > 0.0) && (angle_axis[i] < 0.0)))
		{
			angle_axis[i] = -angle_axis[i];
		}
	}

	return angle_axis;
}

Vec3 EuclideanToHomogenous(const Vec2 &x)
{
    return Vec3(x.x(), x.y(), 1.0);
}

Vec4 EuclideanToHomogenous(const Vec3 &x)
{
    return Vec4(x.x(), x.y(), x.z(), 1.0);
}

Vec2 HomogenousToEuclidean(const Vec3 &x)
{
    return x.head<2>() / x.z();
}