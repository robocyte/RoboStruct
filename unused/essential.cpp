	std::vector<int> GetRandomSequenceFromArray(int amount, int min, int max)
	{
		std::vector<int> indices(max);
		std::iota(indices.begin(), indices.end(), min);

		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(indices.begin(), indices.end(), g);
		indices.erase(indices.begin() + amount, indices.end());

		return indices;
	}

	void ComputeNullspaceBasis(std::vector<Eigen::Vector2d> l_pts, std::vector<Eigen::Vector2d> r_pts, double *basis)
	{
		double *Q, *S, *U, VT[81];
		int max_dim;
		int i;

		max_dim = 9;

		Q = (double*)malloc(sizeof(double) * max_dim * max_dim);
		S = (double*)malloc(sizeof(double) * max_dim);
		U = (double*)malloc(sizeof(double) * max_dim * max_dim);

		// create the 5x9 epipolar constraint matrix
		for (i = 0; i < 5; i++)
		{
			double *row = Q + i * 9;

			row[0] = l_pts[i].x() * r_pts[i].x();
			row[1] = l_pts[i].y() * r_pts[i].x();
			row[2] = r_pts[i].x();

			row[3] = l_pts[i].x() * r_pts[i].y();
			row[4] = l_pts[i].y() * r_pts[i].y();
			row[5] = r_pts[i].y();

			row[6] = l_pts[i].x();
			row[7] = l_pts[i].y();
			row[8] = 1.0;
		}

		// find four vectors that span the right nullspace of the matrix
		dgesvd_driver(5, 9, Q, U, S, VT);

		memcpy(basis, VT + 5 * 9, 36 * sizeof(double));

		free(Q);
		free(S);
		free(U);
	}

	void ComputeConstraintMatrix(double *basis, Poly3 *constraints)
	{
		// Basis rows are X, Y, Z, W; essential matrix is or form x*X + y*Y + z*Z + W

		// Create a polynomial for each entry of E
		Poly3 polys[9];
		Poly3 poly_term1, poly_term2, poly_term3, poly_det;
		Poly3 poly_EET[6], poly_lambda[6], poly_tr, poly_lambdaE[9];

		int i;

		for (i=0; i < 9; i++) polys[i] = Poly3New(basis[i], basis[9+i], basis[18+i], basis[27+i]);

		// create a polynormial from the constraint det(E) = 0
		poly_term1 = Poly3Mult21( Poly3Sub( Poly3Mult11(polys[1], polys[5]), Poly3Mult11(polys[2], polys[4]) ), polys[6] );
		poly_term2 = Poly3Mult21( Poly3Sub( Poly3Mult11(polys[2], polys[3]), Poly3Mult11(polys[0], polys[5]) ), polys[7] );
		poly_term3 = Poly3Mult21( Poly3Sub( Poly3Mult11(polys[0], polys[4]), Poly3Mult11(polys[1], polys[3]) ), polys[8] );
		poly_det = Poly3Add(poly_term1, Poly3Add(poly_term2, poly_term3));

		// create polynomials for the singular value constraint
		for (i = 0; i < 6; i++)
		{
			int r = 0, c = 0, k;
			poly_EET[i] = Poly3New(0.0, 0.0, 0.0, 0.0);

			switch(i)
			{
			case 0:
			case 1:
			case 2:
				r = 0;
				c = i;
				break;
			case 3:
			case 4:
				r = 1;
				c = i-2;
				break;
			case 5:
				r = 2;
				c = 2;
				break;
			}

			for (k = 0; k < 3; k++)
			{
				poly_EET[i] = Poly3Add(poly_EET[i], Poly3Mult11(polys[r*3+k], polys[c*3+k]));
			}
		}

		poly_tr = Poly3Add3(poly_EET[0], poly_EET[3], poly_EET[5]);
		poly_tr = Poly3Scale(poly_tr, 0.5);

		poly_lambda[0] = Poly3Sub(poly_EET[0], poly_tr);
		poly_lambda[1] = poly_EET[1];
		poly_lambda[2] = poly_EET[2];
		poly_lambda[3] = Poly3Sub(poly_EET[3], poly_tr);
		poly_lambda[4] = poly_EET[4];
		poly_lambda[5] = Poly3Sub(poly_EET[5], poly_tr);
		poly_lambdaE[0] = Poly3Add3(Poly3Mult(poly_lambda[0], polys[0]),
									 Poly3Mult(poly_lambda[1], polys[3]),
									 Poly3Mult(poly_lambda[2], polys[6]));
		poly_lambdaE[1] = Poly3Add3(Poly3Mult(poly_lambda[0], polys[1]),
									 Poly3Mult(poly_lambda[1], polys[4]),
									 Poly3Mult(poly_lambda[2], polys[7]));
		poly_lambdaE[2] = Poly3Add3(Poly3Mult(poly_lambda[0], polys[2]),
									 Poly3Mult(poly_lambda[1], polys[5]),
									 Poly3Mult(poly_lambda[2], polys[8]));
		poly_lambdaE[3] = Poly3Add3(Poly3Mult21(poly_lambda[1], polys[0]),
									 Poly3Mult21(poly_lambda[3], polys[3]),
									 Poly3Mult21(poly_lambda[4], polys[6]));
		poly_lambdaE[4] = Poly3Add3(Poly3Mult21(poly_lambda[1], polys[1]),
									 Poly3Mult21(poly_lambda[3], polys[4]),
									 Poly3Mult21(poly_lambda[4], polys[7]));
		poly_lambdaE[5] = Poly3Add3(Poly3Mult21(poly_lambda[1], polys[2]),
									 Poly3Mult21(poly_lambda[3], polys[5]),
									 Poly3Mult21(poly_lambda[4], polys[8]));
		poly_lambdaE[6] = Poly3Add3(Poly3Mult21(poly_lambda[2], polys[0]),
									 Poly3Mult21(poly_lambda[4], polys[3]),
									 Poly3Mult21(poly_lambda[5], polys[6]));
		poly_lambdaE[7] = Poly3Add3(Poly3Mult21(poly_lambda[2], polys[1]),
									 Poly3Mult21(poly_lambda[4], polys[4]),
									 Poly3Mult21(poly_lambda[5], polys[7]));
		poly_lambdaE[8] = Poly3Add3(Poly3Mult21(poly_lambda[2], polys[2]),
									 Poly3Mult21(poly_lambda[4], polys[5]),
									 Poly3Mult21(poly_lambda[5], polys[8]));

		for (i=0; i < 9; i++) constraints[i] = poly_lambdaE[i];

		constraints[9] = poly_det;
	}

	void ComputeGrabnerBasis(Poly3 *constraints, double *Gbasis)
	{
		double A[200];
		int i, j;

		for (i = 0; i < 10; i++)
		{
			double *row = A + 20 * i;

			row[0] = constraints[i].v[POLY3_X3];
			row[1] = constraints[i].v[POLY3_X2Y];
			row[2] = constraints[i].v[POLY3_XY2];
			row[3] = constraints[i].v[POLY3_Y3];
			row[4] = constraints[i].v[POLY3_X2Z];
			row[5] = constraints[i].v[POLY3_XYZ];
			row[6] = constraints[i].v[POLY3_Y2Z];
			row[7] = constraints[i].v[POLY3_XZ2];
			row[8] = constraints[i].v[POLY3_YZ2];
			row[9] = constraints[i].v[POLY3_Z3];
			row[10] = constraints[i].v[POLY3_X2];
			row[11] = constraints[i].v[POLY3_XY];
			row[12] = constraints[i].v[POLY3_Y2];
			row[13] = constraints[i].v[POLY3_XZ];
			row[14] = constraints[i].v[POLY3_YZ];
			row[15] = constraints[i].v[POLY3_Z2];
			row[16] = constraints[i].v[POLY3_X];
			row[17] = constraints[i].v[POLY3_Y];
			row[18] = constraints[i].v[POLY3_Z];
			row[19] = constraints[i].v[POLY3_UNIT];
		}

		// Do a full Gaussian elimination
		for (i = 0; i < 10; i++)
		{
			// Make the leading coefficient of row i = 1
			double leading = A[20 * i + i];
			matrix_scale(20, 1, A + 20 * i, 1.0 / leading, A + 20 * i);

			// Subtract from other rows
			for (j = i+1; j < 10; j++)
			{
				double leading2 = A[20 * j + i];
				double scaled_row[20];
				matrix_scale(20, 1, A + 20 * i, leading2, scaled_row);
				matrix_diff(20, 1, 20, 1, A + 20 * j, scaled_row, A + 20 * j);
			}
		}

		// Now, do the back substitution
		for (i = 9; i >= 0; i--)
		{
			for (j = 0; j < i; j++)
			{
				double scale = A[20 * j + i];
				double scaled_row[20];
				matrix_scale(20, 1, A + 20 * i, scale, scaled_row);
				matrix_diff(20, 1, 20, 1, A + 20 * j, scaled_row, A + 20 * j);
			}
		}

		// copy out results
		for (i = 0; i < 10; i++) memcpy(Gbasis + i * 10, A + i * 20 + 10, sizeof(double) * 10);
	}

	void ComputeActionMatrix(double *Gbasis, double *At) 
	{
		int i;
		for (i = 0; i < 100; i++) At[i] = 0.0;

		matrix_scale(10, 1, Gbasis +  0, -1.0, At +  0);
		matrix_scale(10, 1, Gbasis + 10, -1.0, At + 10);
		matrix_scale(10, 1, Gbasis + 20, -1.0, At + 20);
		matrix_scale(10, 1, Gbasis + 40, -1.0, At + 30);
		matrix_scale(10, 1, Gbasis + 50, -1.0, At + 40);
		matrix_scale(10, 1, Gbasis + 70, -1.0, At + 50);

		At[6 * 10 + 0] = 1.0;
		At[7 * 10 + 1] = 1.0;
		At[8 * 10 + 3] = 1.0;
		At[9 * 10 + 6] = 1.0;
	}

	std::vector<Eigen::Matrix3d> ComputeEssentialMatricesGrabner(double *At, double *basis)
	{
		double evec[100], eval[10];
		int real = dgeev_driver(10, At, evec, eval);
		double E[90];
		std::vector<Eigen::Matrix3d> solutions;

		for (int i = 0; i < real; i++)
		{
			double X[9], Y[9], Z[9], tmp1[9], tmp2[9];

			double x = evec[10 * i + 6];
			double y = evec[10 * i + 7];
			double z = evec[10 * i + 8];
			double w = evec[10 * i + 9];
			double w_inv = 1.0 / w;

			x = x * w_inv;
			y = y * w_inv;
			z = z * w_inv;

			matrix_scale(9, 1, basis + 0, x, X);
			matrix_scale(9, 1, basis + 9, y, Y);
			matrix_scale(9, 1, basis + 18, z, Z);

			matrix_sum(9, 1, 9, 1, X, Y, tmp1);
			matrix_sum(9, 1, 9, 1, Z, basis + 27, tmp2);
			matrix_sum(9, 1, 9, 1, tmp1, tmp2, E + 9 * i);

			matrix_scale(9, 1, E + 9 * i, 1.0 / E[9 * i + 8], E + 9 * i);
		}

		for (int i = 0; i < 10; ++i)
		{
			Eigen::Matrix3d candidate;
			candidate <<	E[9 * i], E[9 * i + 1], E[9 * i + 2],
							E[9 * i + 3], E[9 * i + 4], E[9 * i + 5],
							E[9 * i + 6], E[9 * i + 7], E[9 * i + 8];
			solutions.push_back(candidate);
		}

		return solutions;
	}

	void ComputeEssentialMatrices(int n, double *roots, double *basis, Poly1 p1, Poly1 p2, Poly1 p3, double *E)
	{
		int i;

		for (i = 0; i < n; i++)
		{
			double z = roots[i];
			double den = Poly1Eval(p3, z);
			double den_inv = 1.0 / den;

			double x = Poly1Eval(p1, z) * den_inv;
			double y = Poly1Eval(p2, z) * den_inv;

			double X[9], Y[9], Z[9], tmp1[9], tmp2[9];

			matrix_scale(9, 1, basis + 0, x, X);
			matrix_scale(9, 1, basis + 9, y, Y);
			matrix_scale(9, 1, basis + 18, z, Z);

			matrix_sum(9, 1, 9, 1, X, Y, tmp1);
			matrix_sum(9, 1, 9, 1, Z, basis + 27, tmp2);
			matrix_sum(9, 1, 9, 1, tmp1, tmp2, E + 9 * i);
		}
	}

	double FundamentalMatrixComputeResidual(double *F, v3_t r, v3_t l)
	{
		double Fl[3], Fr[3], pt;    

		Fl[0] = F[0] * Vx(l) + F[1] * Vy(l) + F[2] * Vz(l);
		Fl[1] = F[3] * Vx(l) + F[4] * Vy(l) + F[5] * Vz(l);
		Fl[2] = F[6] * Vx(l) + F[7] * Vy(l) + F[8] * Vz(l);

		Fr[0] = F[0] * Vx(r) + F[3] * Vy(r) + F[6] * Vz(r);
		Fr[1] = F[1] * Vx(r) + F[4] * Vy(r) + F[7] * Vz(r);
		Fr[2] = F[2] * Vx(r) + F[5] * Vy(r) + F[8] * Vz(r);

		pt = Vx(r) * Fl[0] + Vy(r) * Fl[1] + Vz(r) * Fl[2];

		return (1.0 / (Fl[0] * Fl[0] + Fl[1] * Fl[1]) + 1.0 / (Fr[0] * Fr[0] + Fr[1] * Fr[1])) * (pt * pt);
	}

	std::vector<Eigen::Matrix3d> GenerateEssentialMatrixHypotheses(std::vector<Eigen::Vector2d> l_pts, std::vector<Eigen::Vector2d> r_pts)
	{
		double basis[36], Gbasis[100], At[100];
		Poly3 constraints[10];

		if (l_pts.size() < 5)
		{
			std::cerr << "[GenerateEssentialMatrixHypotheses] n must be >= 5";
			return std::vector<Eigen::Matrix3d>(1);
		}

		// generate the nullspace basis of the epipolar constraint matrix
		ComputeNullspaceBasis(r_pts, l_pts, basis);
		ComputeConstraintMatrix(basis, constraints);
		ComputeGrabnerBasis(constraints, Gbasis);
		ComputeActionMatrix(Gbasis, At);
		return ComputeEssentialMatricesGrabner(At, basis);
	}

	int EvaluateEssentialMatrix(std::vector<KeyPoint> l_pts, std::vector<KeyPoint> r_pts, double thresh_norm, const Eigen::Matrix3d &F, int &best_inlier, double &score)
	{
		int num_inliers = 0;
		int i;
		double min_resid = 1.0e20;
		double likelihood = 0.0;

		for (i = 0; i < r_pts.size(); i++)
		{
			v3_t r = v3_new(r_pts[i].m_x, r_pts[i].m_y, 1.0);
			v3_t l = v3_new(l_pts[i].m_x, l_pts[i].m_y, 1.0);

			double Fc[9];
			Fc[0] = F(0, 0);
			Fc[1] = F(0, 1);
			Fc[2] = F(0, 2);

			Fc[3] = F(1, 0);
			Fc[4] = F(1, 1);
			Fc[5] = F(1, 2);

			Fc[6] = F(2, 0);
			Fc[7] = F(2, 1);
			Fc[8] = F(2, 2);
			double resid = FundamentalMatrixComputeResidual(Fc, l, r);

			likelihood += log(1.0 + resid * resid / (thresh_norm));

			if (resid < thresh_norm)
			{
				num_inliers++;

				if (resid < min_resid)
				{
					min_resid = resid;
					best_inlier = i;
				}
			}
		}

		score = likelihood;

		return num_inliers;
	}

	bool FindExtrinsicsWithEssential(const Eigen::Matrix3d &E, std::vector<Eigen::Vector2d> l_pts, std::vector<Eigen::Vector2d> r_pts, Eigen::Matrix3d &R_out, Eigen::Vector3d &t_out)
	{
		double Eb[9];
		Eb[0] = E(0, 0);
		Eb[1] = E(0, 1);
		Eb[2] = E(0, 2);

		Eb[3] = E(1, 0);
		Eb[4] = E(1, 1);
		Eb[5] = E(1, 2);

		Eb[6] = E(2, 0);
		Eb[7] = E(2, 1);
		Eb[8] = E(2, 2);
		double tmp[9], tmp2[3], Qv[3], R[9], t[3];
		double U[9], S[3], VT[9];
		double tu[3], Ra[9], Rb[9];

		double D[9] = 
		{  0.0, 1.0, 0.0,
		  -1.0, 0.0, 0.0,
		   0.0, 0.0, 1.0 };

		double DT[9] = 
		{  0.0, -1.0, 0.0,
		   1.0,  0.0, 0.0,
		   0.0,  0.0, 1.0 };

		double I[9] = 
		{  1.0, 0.0, 0.0, 
		   0.0, 1.0, 0.0,
		   0.0, 0.0, 1.0 };
    
		double t0[3] = { 0.0, 0.0, 0.0 };

		v3_t Q, PQ;
		double c1, c2;
		double error;

		int i;
		int c1_pos = 0, c1_neg = 0;
		int c2_pos = 0, c2_neg = 0;

		/* Now find the SVD of E */
		dgesvd_driver(3, 3, Eb, U, S, VT);

		/* Now find R and t */
		tu[0] = U[2];  tu[1] = U[5];  tu[2] = U[8];

		matrix_product33(U, D, tmp);
		matrix_product33(tmp, VT, Ra);
		matrix_product33(U, DT, tmp);
		matrix_product33(tmp, VT, Rb);

		if (matrix_determinant3(Ra) < 0.0) matrix_scale(3, 3, Ra, -1.0, Ra);
		if (matrix_determinant3(Rb) < 0.0) matrix_scale(3, 3, Rb, -1.0, Rb);

		/* Figure out which configuration is correct using the supplied points */
		for (i = 0; i < l_pts.size(); i++)
		{
			v2_t p1 = v2_new(l_pts[i].x(), l_pts[i].y());
			v2_t p2 = v2_new(r_pts[i].x(), r_pts[i].y());

			Q = triangulate(p1, p2, I, t0, Ra, tu, &error);
			Qv[0] = Vx(Q), Qv[1] = Vy(Q), Qv[2] = Vz(Q);
			matrix_product331(Ra, Qv, tmp);
			matrix_sum(3, 1, 3, 1, tmp, tu, tmp2);
			PQ = v3_new(tmp2[0], tmp2[1], tmp2[2]);

			c1 = Vz(Q);
			c2 = Vz(PQ);

			if (c1 > 0)	c1_pos++;
			else		c1_neg++;
        
			if (c2 > 0)	c2_pos++;
			else		c2_neg++;
		}

		if (c1_pos < c1_neg && c2_pos < c2_neg)
		{
			memcpy(R, Ra, 9 * sizeof(double));
			t[0] = tu[0]; t[1] = tu[1]; t[2] = tu[2];
		} else if (c1_pos > c1_neg && c2_pos > c2_neg)
		{
			memcpy(R, Ra, 9 * sizeof(double));
			t[0] = -tu[0]; t[1] = -tu[1]; t[2] = -tu[2];
		} else
		{
			/* Triangulate again */
			c1_pos = c1_neg = c2_pos = c2_neg = 0;

			for (i = 0; i < l_pts.size(); i++)
			{
				v2_t p1 = v2_new(l_pts[i].x(), l_pts[i].y());
				v2_t p2 = v2_new(r_pts[i].x(), r_pts[i].y());
				Q = triangulate(p1, p2, I, t0, Rb, tu, &error);
				Qv[0] = Vx(Q), Qv[1] = Vy(Q), Qv[2] = Vz(Q);
				matrix_product331(Rb, Qv, tmp);
				matrix_sum(3, 1, 3, 1, tmp, tu, tmp2);
				PQ = v3_new(tmp2[0], tmp2[1], tmp2[2]);

				c1 = Vz(Q);
				c2 = Vz(PQ);

				if (c1 > 0)	c1_pos++;
				else		c1_neg++;
        
				if (c2 > 0)	c2_pos++;
				else		c2_neg++;
			}

			if (c1_pos < c1_neg && c2_pos < c2_neg)
			{
				memcpy(R, Rb, 9 * sizeof(double));
				t[0] = tu[0]; t[1] = tu[1]; t[2] = tu[2];
			} else if (c1_pos > c1_neg && c2_pos > c2_neg)
			{
				memcpy(R, Rb, 9 * sizeof(double));
				t[0] = -tu[0]; t[1] = -tu[1]; t[2] = -tu[2];
			} else
			{
				fprintf(stderr, "[find_extrinsics] Error: no case found!\n");
				return false;
			}

			R_out = Eigen::Matrix3d();
			R_out << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
			t_out = Eigen::Vector3d();
			t_out << t[0], t[1], t[2];
		}

		return true;
	}


	
namespace
{
	typedef Eigen::MatrixXd Mat;
	typedef Eigen::VectorXd Vec;
	typedef Eigen::Matrix<double, 3, 3> Mat3;
	typedef Eigen::Matrix<double, 3, 4> Mat34;
	typedef Eigen::Matrix<double, 4, 4> Mat4;
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	typedef Eigen::Vector4d Vec4;

	template <typename TMat, typename TVec>
	double Nullspace(TMat *A, TVec *nullspace)
	{
		Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
		(*nullspace) = svd.matrixV().col(A->cols()-1);
		if (A->rows() >= A->cols())	return svd.singularValues()(A->cols()-1);
		else return 0.0;
	}
	
	void P_From_KRt(const Eigen::Matrix3d &K, const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix<double, 3, 4> *P)
	{
		P->block<3, 3>(0, 0) = R;
		P->col(3) = t;
		(*P) = K * (*P);
	}

	double Depth(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const Eigen::Vector3d &X)
	{
		return (R*X)(2) + t(2);
	}

	void HomogeneousToEuclidean(const Vec4 &H, Vec3 *X)
	{
		double w = H(3);
		*X << H(0) / w, H(1) / w, H(2) / w;
	}

	void TriangulateDLT(const Mat34 &P1, const Vec2 &x1, const Mat34 &P2, const Vec2 &x2, Vec4 *X_homogeneous)
	{
		Mat4 design;
	
		for (int i = 0; i < 4; ++i)
		{
			design(0,i) = x1(0) * P1(2,i) - P1(0,i);
			design(1,i) = x1(1) * P1(2,i) - P1(1,i);
			design(2,i) = x2(0) * P2(2,i) - P2(0,i);
			design(3,i) = x2(1) * P2(2,i) - P2(1,i);
		}
	
		Nullspace(&design, X_homogeneous);
	}

	void TriangulateDLT(const Mat34 &P1, const Vec2 &x1, const Mat34 &P2, const Vec2 &x2, Vec3 *X_euclidean)
	{
		Vec4 X_homogeneous;
		TriangulateDLT(P1, x1, P2, x2, &X_homogeneous);
		HomogeneousToEuclidean(X_homogeneous, X_euclidean);
	}

	void MotionFromEssential(const Eigen::Matrix3d &E, std::vector<Eigen::Matrix3d> *Rs, std::vector<Eigen::Vector3d> *ts)
	{
		Eigen::JacobiSVD<Eigen::Matrix3d> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U =  USV.matrixU();
		Eigen::Vector3d d =  USV.singularValues();
		Eigen::Matrix3d Vt = USV.matrixV().transpose();

		// Last column of U is undetermined since d = (a a 0).
		if (U.determinant() < 0) U.col(2) *= -1;

		// Last row of Vt is undetermined since d = (a a 0).
		if (Vt.determinant() < 0) Vt.row(2) *= -1;

		Eigen::Matrix3d W;
		W <<	0, -1,  0,
				1,  0,  0,
				0,  0,  1;

		Eigen::Matrix3d U_W_Vt = U * W * Vt;
		Eigen::Matrix3d U_Wt_Vt = U * W.transpose() * Vt;

		Rs->resize(4);
		(*Rs)[0] = U_W_Vt;
		(*Rs)[1] = U_W_Vt;
		(*Rs)[2] = U_Wt_Vt;
		(*Rs)[3] = U_Wt_Vt;

		ts->resize(4);
		(*ts)[0] = U.col(2);
		(*ts)[1] = -U.col(2);
		(*ts)[2] =  U.col(2);
		(*ts)[3] = -U.col(2);
	}

	int MotionFromEssentialChooseSolution(const std::vector<Eigen::Matrix3d> &Rs, const std::vector<Eigen::Vector3d> &ts, const Eigen::Matrix3d &K1, const Vec2 &x1, const Eigen::Matrix3d &K2, const Vec2 &x2)
	{
		Eigen::Matrix<double, 3, 4> P1, P2;
		Eigen::Matrix3d R1;
		Eigen::Vector3d t1;

		R1.setIdentity();
		t1.setZero();

		P_From_KRt(K1, R1, t1, &P1);
		
		for (int i = 0; i < 4; ++i)
		{
			const Eigen::Matrix3d &R2 = Rs[i];
			const Eigen::Vector3d &t2 = ts[i];
			P_From_KRt(K2, R2, t2, &P2);
			Eigen::Vector3d X;

			TriangulateDLT(P1, x1, P2, x2, &X);
			double d1 = Depth(R1, t1, X);
			double d2 = Depth(R2, t2, X);

			// Test if point is front to the two cameras.
			if (d1 > 0 && d2 > 0) return i;
		}

		return -1;
	}

	bool MotionFromEssentialAndCorrespondence(const Eigen::Matrix3d &E, const Eigen::Matrix3d &K1, const Eigen::Vector2d &x1, const Eigen::Matrix3d &K2, const Eigen::Vector2d &x2, Eigen::Matrix3d *R, Eigen::Vector3d *t)
	{
		std::vector<Eigen::Matrix3d> Rs;
		std::vector<Eigen::Vector3d> ts;
		MotionFromEssential(E, &Rs, &ts);

		//for (auto &R : Rs) std::cerr << "\nR:\n" << R;
		//for (auto &t : ts) std::cerr << "\nt: \n" << t;

		int solution = MotionFromEssentialChooseSolution(Rs, ts, K1, x1, K2, x2);
		if (solution >= 0)
		{
			*R = Rs[solution];
			*t = ts[solution];
			return true;
		} else
		{
			return false;
		}
	}
}
	
	
	// TESTING
	auto Fmat = GetFundamentalMatrix(i1, i2);
	Eigen::Matrix3d F;
	cv::cv2eigen(Fmat, F);

	Eigen::Matrix3d KE1, KE2, EEE;
	KE1 << K1[0], K1[1], K1[2], K1[3], K1[4], K1[5], K1[6], K1[7], K1[8];
	KE2	<< K2[0], K2[1], K2[2], K2[3], K2[4], K2[5], K2[6], K2[7], K2[8];
	auto E = KE2.transpose() * F * KE1;

	Mat3 R;
	Vec3 t;

	Eigen::Vector2d r(k1_pts[1].p[0], k1_pts[1].p[1]);
	Eigen::Vector2d l(k2_pts[1].p[0], k2_pts[1].p[1]);

	bool is_motion_ok = MotionFromEssentialAndCorrespondence(E, KE1, r, KE2, l, &R, &t);
	std::cerr << "\nLibmv R: \n" << R << std::endl;
	std::cerr << "Bundler R: \n";
	std::cerr << R0[0] << " " << R0[1] << " " << R0[2] << std::endl;
	std::cerr << R0[3] << " " << R0[4] << " " << R0[5] << std::endl;
	std::cerr << R0[6] << " " << R0[7] << " " << R0[8] << std::endl;
	std::cerr << "Libmv t: \n" << t << std::endl;
	std::cerr << "Bundler t: \n";
	std::cerr << t0[0] << "\n" << t0[1] << "\n" << t0[2] << std::endl;

	R0[0] = R(0, 0);
	R0[1] = R(0, 1);
	R0[2] = -R(0, 2);

	R0[3] = R(1, 0);
	R0[4] = R(1, 1);
	R0[5] = -R(1, 2);

	R0[6] = -R(2, 0);
	R0[7] = -R(2, 1);
	R0[8] = R(2, 2);

	t0[0] = t(0);
	t0[1] = t(1);
	t0[2] = -t(2);
	// TESTING
