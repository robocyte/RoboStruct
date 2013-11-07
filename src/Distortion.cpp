#include <math.h>

#include "EigenTools.hpp"

Vec6 InvertDistortion(double r0, double r1, const Vec6 &k_in)
{
	int num_eqns = 20;
	int num_vars = 6;

	Eigen::Matrix<double, 20, 6>	A;
	Eigen::Matrix<double, 20, 1>	b;
	Eigen::Matrix<double, 6, 1>		x;

	for (int i = 0; i < num_eqns; i++)
	{
		double t = r0 + i * (r1 - r0) / (num_eqns - 1);
		double p = 1.0;
		double a = 0.0;
		for (int j = 0; j < 6; j++)
		{
			a += p * k_in(j);
			p *= t;
		}

		double ap = 1.0;
		for (int j = 0; j < 6; j++)
		{
			A(i, j) = ap;
			ap *= a;
		}

		b(i) = t;
	}

	return A.colPivHouseholderQr().solve(b);
}

Point2 UndistortNormalizedPoint(const Point2 &p, const Vec6 & k_inverse)
{
	double r = p.norm();
	if (r == 0.0) return p;

	double t = 1.0;
	double a = 0.0;

	for (int i = 0; i < 6; i++)
	{
		a += t * k_inverse(i);
		t *= r;
	}

	return (a / r) * p;
}
