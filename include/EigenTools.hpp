#pragma once

#include <vector>

#include "Eigen/Dense"
#include "unsupported/Eigen/NonLinearOptimization"

typedef Eigen::MatrixXd							Mat;
typedef Eigen::Matrix3d							Mat3;
typedef Eigen::Matrix<double, 3, 4>				Mat34;
typedef Eigen::VectorXd							Vec;
typedef Eigen::Vector2d							Vec2;
typedef Eigen::Vector3d							Vec3;
typedef Eigen::Vector4d							Vec4;
typedef Eigen::Matrix<double, 6, 1, 0, 6, 1>	Vec6;

typedef std::vector<Mat>	MatVec;
typedef std::vector<Mat3>	Mat3Vec;
typedef std::vector<Vec>	VecVec;
typedef std::vector<Vec2>	Vec2Vec;
typedef std::vector<Vec3>	Vec3Vec;

// The base class for functors when using the (unsupported) NonLinearOptimization module
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct LMFunctor
{
	typedef _Scalar Scalar;

	enum
	{
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};

	typedef Eigen::Matrix<Scalar, InputsAtCompileTime,1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime,1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	const int m_inputs, m_values;

	LMFunctor()
		: m_inputs(InputsAtCompileTime)
		, m_values(ValuesAtCompileTime)
	{}
		
	LMFunctor(int inputs, int values)
		: m_inputs(inputs)
		, m_values(values)
	{}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

	// You should define that in the subclass:
	// int operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

Mat3 AngleAxisToRotationMatrix(const Vec3 &w);

Vec3 RotationMatrixToAngleAxis(const Mat3 &R);

Vec3 EuclideanToHomogenous(const Vec2 &x);

Vec4 EuclideanToHomogenous(const Vec3 &x);

Vec2 HomogenousToEuclidean(const Vec3 &x);