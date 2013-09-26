	// SVD TEST
	double *Q, *S, *U, VT[16];
	int max_dim = 4;

	Q = (double*)malloc(sizeof(double) * max_dim * max_dim);
	S = (double*)malloc(sizeof(double) * max_dim);
	U = (double*)malloc(sizeof(double) * max_dim * max_dim);

	// Create the 3x4 epipolar constraint matrix
	for (int i = 0; i < 3; i++)
	{
		double *row = Q + i * 4;

		row[0] = i + 1;
		row[1] = i + 2;
		row[2] = i + 3;
		row[3] = i + 4;
	}

	Eigen::Matrix<double, 3, 4> A;
	A.setZero();
	for (int i = 0; i < 3; i++)
	{
		A(i, 0) = i + 1;
		A(i, 1) = i + 2;
		A(i, 2) = i + 3;
		A(i, 3) = i + 4;
	}

	Eigen::JacobiSVD<Eigen::Matrix<double, 3, 4>> USV(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	auto EU		= USV.matrixU();
	auto Ed		= USV.singularValues();
	auto EVt	= USV.matrixV().transpose()/*.topLeftCorner<9, 4>()*/;

	std::cerr << "EU:\n" << EU << std::endl;
	std::cerr << "Ed:\n" << Ed << std::endl;
	std::cerr << "EVt:\n" << EVt << std::endl;

	// Find four vectors that span the right nullspace of the matrix
	dgesvd_driver(3, 4, Q, U, S, VT);

	std::cerr << "BU:\n" << std::endl;
	for (int i = 0; i < 12; ++i) std::cerr << Q[i] << " ";
	std::cerr << std::endl;

	std::cerr << "BVt:\n" << std::endl;
	for (int i = 0; i < 16; ++i) std::cerr << VT[i] << " ";
	std::cerr << std::endl;

	free(Q);
	free(S);
	free(U);
