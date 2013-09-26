if (true)
{
	std::string filename = m_path + "\\post.txt";
	std::ofstream outfile(filename);
	outfile << num_cameras << " " << num_nz_points << " " << num_projections << std::endl;

	// Write the projections
	for (int i = 0; i < num_projections; i++)
	{
		outfile << cidx[i] << " " << pidx[i] << "     " << projections[2*i] << " " << projections[2*i + 1] << std::endl;
	}

	// Write init_x
	for (unsigned int i = 0; i < num_parameters; i++) outfile << init_x[i] << std::endl;

	outfile.close();
}
