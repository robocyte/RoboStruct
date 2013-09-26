#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>

#include "FivePoint.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#include "matrix.h"

#include "Poly.hpp"

#include "vector.h"
#include "triangulate.h"

namespace
{
	void Compute_nullspace_basis(int n, v2_t *a, v2_t *b, double *basis)
	{
		double *Q, *S, *U, VT[81];
		int max_dim;

		if (n < 5)
		{
			fprintf(stderr, "[compute_nullspace_basis] n must be >= 5\n");
			return;
		}

		max_dim = MAX(9, n);

		Q = (double*)malloc(sizeof(double) * max_dim * max_dim);
		S = (double*)malloc(sizeof(double) * max_dim);
		U = (double*)malloc(sizeof(double) * max_dim * max_dim);

		// create the 5x9 epipolar constraint matrix
		for (int i = 0; i < 5; i++)
		{
			double *row = Q + i * 9;

			row[0] = a[i].p[0] * b[i].p[0];
			row[1] = a[i].p[1] * b[i].p[0];
			row[2] = b[i].p[0];

			row[3] = a[i].p[0] * b[i].p[1];
			row[4] = a[i].p[1] * b[i].p[1];
			row[5] = b[i].p[1];

			row[6] = a[i].p[0];
			row[7] = a[i].p[1];
			row[8] = 1.0;
		}

		Eigen::Matrix<double, 5, 9> A;
		A.setZero();
		for (int i = 0; i < 5; i++)
		{
			A(i, 0) = a[i].p[0] * b[i].p[0];
			A(i, 1) = a[i].p[1] * b[i].p[0];
			A(i, 2) = b[i].p[0];

			A(i, 3) = a[i].p[0] * b[i].p[1];
			A(i, 4) = a[i].p[1] * b[i].p[1];
			A(i, 5) = b[i].p[1];

			A(i, 6) = a[i].p[0];
			A(i, 7) = a[i].p[1];
			A(i, 8) = 1.0;
		}

		Eigen::JacobiSVD<Eigen::Matrix<double, 5, 9>> svd;
		auto Ebasis = svd.compute(A, Eigen::ComputeFullV).matrixV().transpose().bottomLeftCorner<4, 9>();

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				basis[9 * i + j] = Ebasis(i, j);
				basis[9 * i + j] = Ebasis(i, j);
				basis[9 * i + j] = Ebasis(i, j);

				basis[9 * i + j] = Ebasis(i, j);
				basis[9 * i + j] = Ebasis(i, j);
				basis[9 * i + j] = Ebasis(i, j);

				basis[9 * i + j] = Ebasis(i, j);
				basis[9 * i + j] = Ebasis(i, j);
				basis[9 * i + j] = Ebasis(i, j);
			}
		}

		//std::cerr << "Ebasis:\n" << Ebasis << std::endl;

		//// Find four vectors that span the right nullspace of the matrix
		//dgesvd_driver(n, 9, Q, U, S, VT);

		////memcpy(basis, VT /*+ 5 * 9*/, 36 * sizeof(double));

		//std::cerr << "Bbasis:\n" << std::endl;
		//for (int i = 0; i < 36; ++i) std::cerr << basis[i] << " ";
		////std::cerr << "\nBVT:\n:" << std::endl;
		////for (int i = 0; i < 81; ++i) std::cerr << VT[i] << " ";
		//std::cerr << std::endl;

		free(Q);
		free(S);
		free(U);
	}

	void Compute_constraint_matrix(double *basis, Poly3 *constraints)
	{
		// Basis rows are X, Y, Z, W; essential matrix is or form x*X + y*Y + z*Z + W

		// Create a polynomial for each entry of E
		Poly3 polys[9];
		Poly3 poly_term1, poly_term2, poly_term3, poly_det;
		Poly3 poly_EET[6], poly_lambda[6], poly_tr, poly_lambdaE[9];

		int i;

		for (i=0; i < 9; i++)
		{
			polys[i] = Poly3New(basis[i], basis[9+i], basis[18+i], basis[27+i]);
		}

		// Create a polynormial from the constraint det(E) = 0
		poly_term1 = Poly3Mult21( Poly3Sub( Poly3Mult11(polys[1], polys[5]), Poly3Mult11(polys[2], polys[4]) ), polys[6] );
		poly_term2 = Poly3Mult21( Poly3Sub( Poly3Mult11(polys[2], polys[3]), Poly3Mult11(polys[0], polys[5]) ), polys[7] );
		poly_term3 = Poly3Mult21( Poly3Sub( Poly3Mult11(polys[0], polys[4]), Poly3Mult11(polys[1], polys[3]) ), polys[8] );
		poly_det = Poly3Add(poly_term1, Poly3Add(poly_term2, poly_term3));

		// Create polynomials for the singular value constraint
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

	void Compute_B_matrix(Poly3 *constraints, Poly1 *B) 
	{
		Poly3 e = constraints[4];
		Poly3 f = constraints[5];
		Poly3 g = constraints[6];
		Poly3 h = constraints[7];
		Poly3 i = constraints[8];
		Poly3 j = constraints[9];

		B[0] = Poly1New3(-Poly3Get(f, POLY3_XZ2),
						  Poly3Get(e, POLY3_XZ2) - Poly3Get(f, POLY3_XZ),
						  Poly3Get(e, POLY3_XZ) - Poly3Get(f, POLY3_X),
						  Poly3Get(e, POLY3_X));
		B[1] = Poly1New3(-Poly3Get(f, POLY3_YZ2),
						  Poly3Get(e, POLY3_YZ2) - Poly3Get(f, POLY3_YZ),
						  Poly3Get(e, POLY3_YZ) - Poly3Get(f, POLY3_Y),
						  Poly3Get(e, POLY3_Y));
		B[2] = Poly1New4(-Poly3Get(f, POLY3_Z3),
						  Poly3Get(e, POLY3_Z3) - Poly3Get(f, POLY3_Z2),
						  Poly3Get(e, POLY3_Z2) - Poly3Get(f, POLY3_Z),
						  Poly3Get(e, POLY3_Z) - Poly3Get(f, POLY3_UNIT),
						  Poly3Get(e, POLY3_UNIT));
		B[3] = Poly1New3(-Poly3Get(h, POLY3_XZ2),
						  Poly3Get(g, POLY3_XZ2) - Poly3Get(h, POLY3_XZ),
						  Poly3Get(g, POLY3_XZ) - Poly3Get(h, POLY3_X),
						  Poly3Get(g, POLY3_X));
		B[4] = Poly1New3(-Poly3Get(h, POLY3_YZ2),
						  Poly3Get(g, POLY3_YZ2) - Poly3Get(h, POLY3_YZ),
						  Poly3Get(g, POLY3_YZ) - Poly3Get(h, POLY3_Y),
						  Poly3Get(g, POLY3_Y));
		B[5] = Poly1New4(-Poly3Get(h, POLY3_Z3),
						  Poly3Get(g, POLY3_Z3) - Poly3Get(h, POLY3_Z2),
						  Poly3Get(g, POLY3_Z2) - Poly3Get(h, POLY3_Z),
						  Poly3Get(g, POLY3_Z) - Poly3Get(h, POLY3_UNIT),
						  Poly3Get(g, POLY3_UNIT));
		B[6] = Poly1New3(-Poly3Get(j, POLY3_XZ2),
						  Poly3Get(i, POLY3_XZ2) - Poly3Get(j, POLY3_XZ),
						  Poly3Get(i, POLY3_XZ) - Poly3Get(j, POLY3_X),
						  Poly3Get(i, POLY3_X));
		B[7] = Poly1New3(-Poly3Get(j, POLY3_YZ2),
						  Poly3Get(i, POLY3_YZ2) - Poly3Get(j, POLY3_YZ),
						  Poly3Get(i, POLY3_YZ) - Poly3Get(j, POLY3_Y),
						  Poly3Get(i, POLY3_Y));
		B[8] = Poly1New4(-Poly3Get(j, POLY3_Z3),
						  Poly3Get(i, POLY3_Z3) - Poly3Get(j, POLY3_Z2),
						  Poly3Get(i, POLY3_Z2) - Poly3Get(j, POLY3_Z),
						  Poly3Get(i, POLY3_Z) - Poly3Get(j, POLY3_UNIT),
						  Poly3Get(i, POLY3_UNIT));
	}

	void Compute_determinant(Poly1 *B, Poly1 *p1, Poly1 *p2, Poly1 *p3, Poly1 *det)
	{
		*p1 = Poly1Sub( Poly1Mult( B[1], B[5] ), Poly1Mult( B[2], B[4] ) );
		*p2 = Poly1Sub( Poly1Mult( B[2], B[3] ), Poly1Mult( B[0], B[5] ) );
		*p3 = Poly1Sub( Poly1Mult( B[0], B[4] ), Poly1Mult( B[1], B[3] ) );
		*det = Poly1Add3(Poly1Mult(*p1, B[6]), Poly1Mult(*p2, B[7]), Poly1Mult(*p3, B[8]));
	}

	void Extract_roots(Poly1 det, int *num_roots, double *roots)
	{
		double C[100], evec[100], eval[10];

		// Scale the determinant
		Poly1 det_scale = Poly1Normalize(det);

		int i, real;

		// Fill the companion matrix
		for (i = 0; i < 100; i++)	C[i] = 0.0;
		for (i = 0; i < 10; i++)	C[i] = -det_scale.v[9-i];
		for (i = 1; i < 10; i++)	C[i * 10 + (i-1)] = 1.0;

		real = dgeev_driver(10, C, evec, eval);

		memcpy(roots, eval, real * sizeof(double));
		*num_roots = real;
	}

	void Compute_Ematrices(int n, double *roots, double *basis, Poly1 p1, Poly1 p2, Poly1 p3, double *E)
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

	void Compute_Grabner_basis(Poly3 *constraints, double *Gbasis) 
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

		// now, do the back substitution
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

	void Compute_action_matrix(double *Gbasis, double *At) 
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

	void Compute_Ematrices_Gb(double *At, double *basis, int *num_solns, double *E)
	{
		double evec[100], eval[10];
		int real = dgeev_driver(10, At, evec, eval);
		int i;

		for (i = 0; i < real; i++)
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

		*num_solns = real;
	}

	void Generate_Ematrix_hypotheses(int n, v2_t *rt_pts, v2_t *left_pts, int *num_poses, double *E)
	{
		double basis[36], Gbasis[100], At[100];
		Poly3 constraints[10];

		if (n < 5)
		{
			fprintf(stderr, "[generate_Ematrix_hypotheses] n must be >= 5\n");
			return;
		}

		// generate the nullspace basis of the epipolar constraint matrix
		Compute_nullspace_basis(n, rt_pts, left_pts, basis);
		Compute_constraint_matrix(basis, constraints);
		Compute_Grabner_basis(constraints, Gbasis);
		Compute_action_matrix(Gbasis, At);
		Compute_Ematrices_Gb(At, basis, num_poses, E);
	}

	void Choose(int n, int k, int *arr)
	{
		int i;

		if (k > n)
		{
			fprintf(stderr, "[choose] Error: k > n\n");
			return;
		}

		for (i = 0; i < k; i++)
		{
			while (1)
			{
				int idx = rand() % n;
				int j, redo = 0;

				for (j = 0; j < i; j++)
				{
					if (idx == arr[j])
					{
						redo = 1;
						break;
					}
				}

				if (!redo)
				{
					arr[i] = idx;
					break;
				}
			}
		}
	}

	double Fmatrix_compute_residual(double *F, v3_t r, v3_t l)
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

	int Evaluate_Ematrix(int n, v2_t *r_pts, v2_t *l_pts, double thresh_norm, double *F, int *best_inlier, double *score)
	{
		int num_inliers = 0;
		int i;
		double min_resid = 1.0e20;
		double likelihood = 0.0;

		for (i = 0; i < n; i++)
		{
			v3_t r = v3_new(Vx(r_pts[i]), Vy(r_pts[i]), 1.0);
			v3_t l = v3_new(Vx(l_pts[i]), Vy(l_pts[i]), 1.0);

			double resid = Fmatrix_compute_residual(F, l, r);

			likelihood += log(1.0 + resid * resid / (thresh_norm));

			if (resid < thresh_norm)
			{
				num_inliers++;

				if (resid < min_resid)
				{
					min_resid = resid;
					*best_inlier = i;
				}
			}
		}

		*score = likelihood;

		return num_inliers;
	}
}

int ComputeRelativePoseRansac(int n, v2_t *r_pts, v2_t *l_pts, double *K1, double *K2, double ransac_threshold, int ransac_rounds, double *R_out, double *t_out)
{
	v2_t *r_pts_norm, *l_pts_norm;
	int i, round;
	double thresh_norm;
	double K1_inv[9], K2_inv[9];
	int max_inliers = 0;
	double min_score = DBL_MAX;
	double E_best[9];

	r_pts_norm = (v2_t*)malloc(sizeof(v2_t) * n);
	l_pts_norm = (v2_t*)malloc(sizeof(v2_t) * n);

	matrix_invert(3, K1, K1_inv);
	matrix_invert(3, K2, K2_inv);

	for (i = 0; i < n; i++)
	{
		double r[3] = { Vx(r_pts[i]), Vy(r_pts[i]), 1.0 };
		double l[3] = { Vx(l_pts[i]), Vy(l_pts[i]), 1.0 };

		double r_norm[3], l_norm[3];

		matrix_product331(K1_inv, r, r_norm);
		matrix_product331(K2_inv, l, l_norm);

		r_pts_norm[i] = v2_new(-r_norm[0], -r_norm[1]);
		l_pts_norm[i] = v2_new(-l_norm[0], -l_norm[1]);
	}

	thresh_norm = ransac_threshold * ransac_threshold;

	for (round = 0; round < ransac_rounds; round++)
	{
		// pick 5 random points
		v2_t r_pts_inner[5], l_pts_inner[5];
		int indices[5];
		int num_hyp;
		double E[90];
		int inliers_hyp[10];
		int first_hyp = -1, first_hyp_idx = -1, second_hyp = -1;
		int best = 0;
		int num_ident = 0;
		int inliers = 0;

		Choose(n, 5, indices);

		for (i = 0; i < 5; i++)
		{
			r_pts_inner[i] = r_pts_norm[indices[i]];
			l_pts_inner[i] = l_pts_norm[indices[i]];

			// check for degeneracy
			if (Vx(r_pts_inner[i]) == Vx(l_pts_inner[i]) && Vy(r_pts_inner[i]) == Vy(l_pts_inner[i])) num_ident++;
		}

		if (num_ident >= 3) continue;	// choose another 5

		Generate_Ematrix_hypotheses(5, r_pts_inner, l_pts_inner, &num_hyp, E);

		for (i = 0; i < num_hyp; i++)
		{
			int best_inlier;
			double score = 0.0;

			double E2[9], tmp[9], F[9];
			memcpy(E2, E + 9 * i, 9 * sizeof(double));
			E2[0] = -E2[0];
			E2[1] = -E2[1];
			E2[3] = -E2[3];
			E2[4] = -E2[4];
			E2[8] = -E2[8];

			matrix_transpose_product(3, 3, 3, 3, K2_inv, E2, tmp);
			matrix_product(3, 3, 3, 3, tmp, K1_inv, F);

			inliers = Evaluate_Ematrix(n, r_pts, l_pts, thresh_norm, F, &best_inlier, &score);

			if (inliers > max_inliers || (inliers == max_inliers && score < min_score))
			{
				best = 1;
				max_inliers = inliers;
				min_score = score;
				memcpy(E_best, E + 9 * i, sizeof(double) * 9);
			}

			inliers_hyp[i] = inliers;
		}

		if (best)
		{
			for (i = 0; i < num_hyp; i++)
			{
				if (inliers_hyp[i] > first_hyp)
				{
					first_hyp = inliers_hyp[i];
					first_hyp_idx = i;
				}
			}

			for (i = 0; i < num_hyp; i++)
			{
				if (i != first_hyp_idx && inliers_hyp[i] > second_hyp) second_hyp = inliers_hyp[i];
			}
		}
	}

	if (max_inliers > 0)
	{
		int success = find_extrinsics_essential_multipt(E_best, n, r_pts_norm, l_pts_norm, R_out, t_out);
	}

	free(r_pts_norm);
	free(l_pts_norm);

	return max_inliers;
}