	void ComputeEssentialMatricesGrabner(double *At, double *basis, int *num_solns, double *E)
	{
		//double evec[100], eval[10];



		// TESTING
		Eigen::Map<Eigen::Matrix<double, 10, 10, Eigen::RowMajor>> Eb(At);

		Eigen::Matrix<double, 10, 10> Eb1 = Eb;
		Eigen::EigenSolver<Eigen::Matrix<double, 10, 10>> Es(Eb1);

		//std::cerr << "Eigen - Eigenvalues:\n" << Es.eigenvalues() << std::endl;
		//std::cerr << "Eigen - Eigenvectors:\n" << Es.eigenvectors() << std::endl;

		//int real = dgeev_driver(10, At, evec, eval);

		//std::cerr << "\nBundler - Eigenvalues [REAL: " << real << "]:\n" << std::endl;
		//for (int i = 0; i < 10; ++i) std::cerr << eval[i] << " ";
		//std::cerr << "\nBundler - Eigenvectors:\n" << std::endl;
		//for (int i = 0; i < 100; ++i) std::cerr << evec[i] << " ";

		int num_e_solns = 0;
		for (int i = 0; i < 10; ++i)
		{
			auto evec = Es.eigenvectors().col(i);
			if (evec(0).imag() == 0)
			{
				double x = evec(6).real();
				double y = evec(7).real();
				double z = evec(8).real();
				double w = evec(9).real();
				double w_inv = 1.0 / w;

				x *= w_inv;
				y *= w_inv;
				z *= w_inv;

				double X[9], Y[9], Z[9], tmp1[9], tmp2[9];
				
				matrix_scale(9, 1, basis + 0, x, X);
				matrix_scale(9, 1, basis + 9, y, Y);
				matrix_scale(9, 1, basis + 18, z, Z);

				matrix_sum(9, 1, 9, 1, X, Y, tmp1);
				matrix_sum(9, 1, 9, 1, Z, basis + 27, tmp2);
				matrix_sum(9, 1, 9, 1, tmp1, tmp2, E + 9 * i);

				matrix_scale(9, 1, E + 9 * i, 1.0 / E[9 * i + 8], E + 9 * i);
				num_e_solns++;
			} else
			{
				continue;
			}
		}
		// TESTING



		//for (int i = 0; i < real; i++)
		//{
		//	double X[9], Y[9], Z[9], tmp1[9], tmp2[9];

		//	double x = evec[10 * i + 6];
		//	double y = evec[10 * i + 7];
		//	double z = evec[10 * i + 8];
		//	double w = evec[10 * i + 9];
		//	double w_inv = 1.0 / w;

		//	x *= w_inv;
		//	y *= w_inv;
		//	z *= w_inv;

		//	matrix_scale(9, 1, basis + 0, x, X);
		//	matrix_scale(9, 1, basis + 9, y, Y);
		//	matrix_scale(9, 1, basis + 18, z, Z);

		//	matrix_sum(9, 1, 9, 1, X, Y, tmp1);
		//	matrix_sum(9, 1, 9, 1, Z, basis + 27, tmp2);
		//	matrix_sum(9, 1, 9, 1, tmp1, tmp2, E + 9 * i);

		//	matrix_scale(9, 1, E + 9 * i, 1.0 / E[9 * i + 8], E + 9 * i);
		//}

		*num_solns = num_e_solns;//real;
	}
