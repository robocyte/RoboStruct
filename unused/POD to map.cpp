	double array[9];
	for(int i = 0; i < 9; ++i) array[i] = i;
	for(int i = 0; i < 9; ++i) std::cerr << array[i] << " ";
	std::cerr << std::endl;

	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> marray(array);

	std::cerr << marray << std::endl;

	marray.transposeInPlace();

	std::cerr << marray << std::endl;

	for(int i = 0; i < 9; ++i) std::cerr << array[i] << " ";
	std::cerr << std::endl;
