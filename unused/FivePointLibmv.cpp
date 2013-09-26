namespace
{
	enum
	{
		ecoef_xxx,
		ecoef_xxy,
		ecoef_xyy,
		ecoef_yyy,
		ecoef_xxz,
		ecoef_xyz,
		ecoef_yyz,
		ecoef_xzz,
		ecoef_yzz,
		ecoef_zzz,
		ecoef_xx,
		ecoef_xy,
		ecoef_yy,
		ecoef_xz,
		ecoef_yz,
		ecoef_zz,
		ecoef_x,
		ecoef_y,
		ecoef_z,
		ecoef_1
	};

	typedef Eigen::VectorXd Vec;
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	typedef Eigen::Vector4d Vec4;
	typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
	typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
	
	template<typename TMatX, typename TMatA>
	inline void EncodeEpipolarEquation(const TMatX &x1, const TMatX &x2, TMatA *A)
	{
		for (int i = 0; i < x1.cols(); ++i)
		{
			(*A)(i, 0) = x2(0, i) * x1(0, i);  // 0 represents x coords,
			(*A)(i, 1) = x2(0, i) * x1(1, i);  // 1 represents y coords.
			(*A)(i, 2) = x2(0, i);
			(*A)(i, 3) = x2(1, i) * x1(0, i);
			(*A)(i, 4) = x2(1, i) * x1(1, i);
			(*A)(i, 5) = x2(1, i);
			(*A)(i, 6) = x1(0, i);
			(*A)(i, 7) = x1(1, i);
			(*A)(i, 8) = 1.0;
		}
	}

	Mat FivePointsNullspaceBasis(const Mat2X &x1, const Mat2X &x2)
	{
		Eigen::Matrix<double, 5, 9> A;
		A.setZero();  // Make A square until Eigen supports rectangular SVD.
		EncodeEpipolarEquation(x1, x2, &A);
		Eigen::JacobiSVD<Eigen::Matrix<double, 5, 9> > svd;
		return svd.compute(A, Eigen::ComputeFullV).matrixV().topRightCorner<9,4>();
	}
	
	Vec o1(const Vec &a, const Vec &b)
	{
		Vec res = Vec::Zero(20);

		res(ecoef_xx)	= a(ecoef_x) * b(ecoef_x);
		res(ecoef_xy)	= a(ecoef_x) * b(ecoef_y)
						+ a(ecoef_y) * b(ecoef_x);
		res(ecoef_xz)	= a(ecoef_x) * b(ecoef_z)
						+ a(ecoef_z) * b(ecoef_x);
		res(ecoef_yy)	= a(ecoef_y) * b(ecoef_y);
		res(ecoef_yz)	= a(ecoef_y) * b(ecoef_z)
						+ a(ecoef_z) * b(ecoef_y);
		res(ecoef_zz)	= a(ecoef_z) * b(ecoef_z);
		res(ecoef_x)		= a(ecoef_x) * b(ecoef_1)
						+ a(ecoef_1) * b(ecoef_x);
		res(ecoef_y)		= a(ecoef_y) * b(ecoef_1)
						+ a(ecoef_1) * b(ecoef_y);
		res(ecoef_z)		= a(ecoef_z) * b(ecoef_1)
						+ a(ecoef_1) * b(ecoef_z);
		res(ecoef_1)		= a(ecoef_1) * b(ecoef_1);

		return res;
	}

	Vec o2(const Vec &a, const Vec &b)
	{
		Vec res(20);

		res(ecoef_xxx) = a(ecoef_xx) * b(ecoef_x);
		res(ecoef_xxy) = a(ecoef_xx) * b(ecoef_y)
					+ a(ecoef_xy) * b(ecoef_x);
		res(ecoef_xxz) = a(ecoef_xx) * b(ecoef_z)
					+ a(ecoef_xz) * b(ecoef_x);
		res(ecoef_xyy) = a(ecoef_xy) * b(ecoef_y)
					+ a(ecoef_yy) * b(ecoef_x);
		res(ecoef_xyz) = a(ecoef_xy) * b(ecoef_z)
					+ a(ecoef_yz) * b(ecoef_x)
					+ a(ecoef_xz) * b(ecoef_y);
		res(ecoef_xzz) = a(ecoef_xz) * b(ecoef_z)
					+ a(ecoef_zz) * b(ecoef_x);
		res(ecoef_yyy) = a(ecoef_yy) * b(ecoef_y);
		res(ecoef_yyz) = a(ecoef_yy) * b(ecoef_z)
					+ a(ecoef_yz) * b(ecoef_y);
		res(ecoef_yzz) = a(ecoef_yz) * b(ecoef_z)
					+ a(ecoef_zz) * b(ecoef_y);
		res(ecoef_zzz) = a(ecoef_zz) * b(ecoef_z);
		res(ecoef_xx)  = a(ecoef_xx) * b(ecoef_1)
					+ a(ecoef_x)  * b(ecoef_x);
		res(ecoef_xy)  = a(ecoef_xy) * b(ecoef_1)
					+ a(ecoef_x)  * b(ecoef_y)
					+ a(ecoef_y)  * b(ecoef_x);
		res(ecoef_xz)  = a(ecoef_xz) * b(ecoef_1)
					+ a(ecoef_x)  * b(ecoef_z)
					+ a(ecoef_z)  * b(ecoef_x);
		res(ecoef_yy)  = a(ecoef_yy) * b(ecoef_1)
					+ a(ecoef_y)  * b(ecoef_y);
		res(ecoef_yz)  = a(ecoef_yz) * b(ecoef_1)
					+ a(ecoef_y)  * b(ecoef_z)
					+ a(ecoef_z)  * b(ecoef_y);
		res(ecoef_zz)  = a(ecoef_zz) * b(ecoef_1)
					+ a(ecoef_z)  * b(ecoef_z);
		res(ecoef_x)   = a(ecoef_x)  * b(ecoef_1)
					+ a(ecoef_1)  * b(ecoef_x);
		res(ecoef_y)   = a(ecoef_y)  * b(ecoef_1)
					+ a(ecoef_1)  * b(ecoef_y);
		res(ecoef_z)   = a(ecoef_z)  * b(ecoef_1)
					+ a(ecoef_1)  * b(ecoef_z);
		res(ecoef_1)   = a(ecoef_1)  * b(ecoef_1);

		return res;
	}

	// Builds the polynomial constraint matrix M.
	Mat FivePointsPolynomialConstraints(const Mat &E_basis)
	{
		// Build the polynomial form of E (equation (8) in Stewenius et al. [1])
		Vec E[3][3];
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				E[i][j] = Vec::Zero(20);
				E[i][j](ecoef_x) = E_basis(3 * i + j, 0);
				E[i][j](ecoef_y) = E_basis(3 * i + j, 1);
				E[i][j](ecoef_z) = E_basis(3 * i + j, 2);
				E[i][j](ecoef_1) = E_basis(3 * i + j, 3);
			}
		}

		// The constraint matrix.
		Mat M(10, 20);
		int mrow = 0;

		// Determinant constraint det(E) = 0; equation (19) of Nister [2].
		M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0]) + 
						o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1]) + 
						o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

		// Cubic singular values constraint.
		// Equation (20).
		Vec EET[3][3];
		for (int i = 0; i < 3; ++i)
		{    // Since EET is symmetric, we only compute
			for (int j = 0; j < 3; ++j)
			{  // its upper triangular part.
				if (i <= j)
				{
					EET[i][j]	= o1(E[i][0], E[j][0])
								+ o1(E[i][1], E[j][1])
								+ o1(E[i][2], E[j][2]);
				} else
				{
					EET[i][j] = EET[j][i];
				}
			}
		}

		// Equation (21).
		Vec (&L)[3][3] = EET;
		Vec trace  = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
		for (int i = 0; i < 3; ++i)
		{
			L[i][i] -= trace;
		}

		// Equation (23).
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				Vec LEij	= o2(L[i][0], E[0][j])
							+ o2(L[i][1], E[1][j])
							+ o2(L[i][2], E[2][j]);
				M.row(mrow++) = LEij;
			}
		}

		return M;
	}

	// Gauss--Jordan elimination for the constraint matrix.
	void FivePointsGaussJordan(Mat *Mp)
	{
		Mat &M = *Mp;

		// Gauss Elimination.
		for (int i = 0; i < 10; ++i)
		{
			M.row(i) /= M(i,i);
			for (int j = i + 1; j < 10; ++j)
			{
				M.row(j) = M.row(j) / M(j,i) - M.row(i);
			}
		}

		// Backsubstitution.
		for (int i = 9; i >= 0; --i)
		{
			for (int j = 0; j < i; ++j)
			{
				M.row(j) = M.row(j) - M(j,i) * M.row(i);
			}
		}
	}	

	std::vector<Mat3> GenerateEssentialMatrixHypotheses2(v2_t *r_pts, v2_t *l_pts)
	{
		std::vector<Mat3> Es;
		// Step 1: Nullspace extraction.
		Eigen::Matrix<double, 2, 5> x1, x2;
		for (int i = 0; i < 5; i++)
		{
			x1(0, i) = r_pts[i].p[0]; x1(1, i) = r_pts[i].p[1];
			x2(0, i) = l_pts[i].p[0]; x2(1, i) = l_pts[i].p[1];
		}

		Mat E_basis = FivePointsNullspaceBasis(x1, x2);

		// Step 2: Constraint expansion.
		Mat M = FivePointsPolynomialConstraints(E_basis);

		// Step 3: Gauss-Jordan elimination.
		FivePointsGaussJordan(&M);

		// For the next steps, follow the matlab code given in Stewenius et al [1].

		// Build the action matrix.
		Mat B = M.topRightCorner<10,10>();
		Mat At = Mat::Zero(10,10);
		At.row(0) = -B.row(0);
		At.row(1) = -B.row(1);
		At.row(2) = -B.row(2);
		At.row(3) = -B.row(4);
		At.row(4) = -B.row(5);
		At.row(5) = -B.row(7);
		At(6,0) = 1;
		At(7,1) = 1;
		At(8,3) = 1;
		At(9,6) = 1;

		// Compute the solutions from action matrix's eigenvectors.
		Eigen::EigenSolver<Mat> es(At);
		typedef Eigen::EigenSolver<Mat>::EigenvectorsType Matc;
		Matc V = es.eigenvectors();
		Matc solutions(4, 10);
		solutions.row(0) = V.row(6).array() / V.row(9).array();
		solutions.row(1) = V.row(7).array() / V.row(9).array();
		solutions.row(2) = V.row(8).array() / V.row(9).array();
		solutions.row(3).setOnes();

		// Get the ten candidate E matrices in vector form.
		Matc Evec = E_basis * solutions;

		// Build the essential matrices for the real solutions.
		Es.reserve(10);
		for (int s = 0; s < 10; ++s)
		{
			Evec.col(s) /= Evec.col(s).norm();
			bool is_real = true;
			for (int i = 0; i < 9; ++i)
			{
				if (Evec(i, s).imag() != 0)
				{
					is_real = false;
					break;
				}
			}
		
			if (is_real)
			{
				Mat3 E;
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < 3; ++j)
					{
						E(i, j) = Evec(3 * i + j, s).real();
					}
				}
				Es.push_back(E);
			}
		}
		return Es;
	}

}
