double MainFrame::RunSFM(int num_pts, int num_cameras, int start_camera,
						camera_params_t *init_camera_params, v3_t *init_pts, int *added_order,
						v3_t *colors, std::vector<ImageKeyVector> &pt_views)
{
	int		min_points		= 20;
	int		num_outliers	= 0;
	int		total_outliers	= 0;
	double	dist_total		= 0.0;
	int		num_dists		= 0;

	std::vector<int>	remap(num_pts);
	std::vector<v3_t>	nz_pts(num_pts);

	do
	{
		if ((num_pts - total_outliers) < min_points)
		{
			wxLogMessage("[RunSFM] Too few points remaining, exiting!");
			dist_total = std::numeric_limits<double>::max();
			break;
		}

		// Set up the vmask and projections
		int num_projections = 0;
		for (int i = 0; i < num_pts; i++) num_projections += (int)pt_views[i].size();

		std::vector<char>	vmask(num_pts * num_cameras, 0);
		std::vector<double>	projections(2 * num_projections);

		int arr_idx = 0;
		int nz_count = 0;
		for (int i = 0; i < num_pts; i++)
		{
			int num_views = (int)pt_views[i].size();

			if (num_views > 0)
			{
				for (int j = 0; j < num_views; j++)
				{
					int c(pt_views[i][j].first);
					int v(added_order[c]);
					int k(pt_views[i][j].second);

					vmask[nz_count * num_cameras + c] = 1;

					projections[2 * arr_idx + 0] = GetKey(v,k).m_x;
					projections[2 * arr_idx + 1] = GetKey(v,k).m_y;

					arr_idx++;
				}

				remap[i] = nz_count;
				nz_pts[nz_count] = init_pts[i];
				nz_count++;
			} else remap[i] = -1;
		}

		dist_total = 0.0;
		num_dists = 0;

		wxLogMessage("[RunSFM] RunSFM starting...");
		clock_t start = clock();
		run_sfm(nz_count, num_cameras, start_camera, vmask.data(), projections.data(), init_camera_params, nz_pts.data(), 1.0e-12, nullptr, nullptr, nullptr, nullptr);
		clock_t end = clock();
		wxLogMessage("[RunSFM] RunSFM took %0.3f s", (double) (end - start) / (double) CLOCKS_PER_SEC);

		// Check for outliers
		start = clock();

		std::vector<int>	outliers;
		std::vector<double>	reproj_errors;

		for (int i = 0; i < num_cameras; i++)
		{
			auto &data = m_images[added_order[i]];

			double K[9] = {	init_camera_params[i].f,	0.0,						0.0,
							0.0,						init_camera_params[i].f,	0.0,
							0.0,						0.0,						1.0 };
			double dt[3] = {init_camera_params[i].t[0], init_camera_params[i].t[1], init_camera_params[i].t[2]};

			// Compute inverse distortion parameters
			double *k = init_camera_params[i].k;
			double k_dist[6] = { 0.0, 1.0, 0.0, k[0], 0.0, k[1] };
			double w_2 = 0.5 * data.GetWidth();
			double h_2 = 0.5 * data.GetHeight();
			double max_radius = sqrt(w_2 * w_2 + h_2 * h_2) / init_camera_params[i].f;

			this->InvertDistortion(0.0, max_radius, k_dist, init_camera_params[i].k_inv);

			int num_keys = this->GetNumKeys(added_order[i]);
			int num_pts_proj = 0;
			for (int j = 0; j < num_keys; j++) if (GetKey(added_order[i], j).m_extra >= 0) num_pts_proj++;

			std::vector<double> dists;
			int pt_count = 0;

			for (auto &key : data.m_keys)
			{
				if (key.m_extra >= 0)
				{
					double b[3], pr[2];
					double dx, dy, dist;
					int pt_idx = key.m_extra;

					b[0] = Vx(nz_pts[remap[pt_idx]]);
					b[1] = Vy(nz_pts[remap[pt_idx]]);
					b[2] = Vz(nz_pts[remap[pt_idx]]);

					sfm_project_rd(&(init_camera_params[i]), K, init_camera_params[i].k, init_camera_params[i].R, dt, b, pr, 1, true);

					dx = pr[0] - key.m_x;
					dy = pr[1] - key.m_y;

					dist = sqrt(dx * dx + dy * dy);
					dist_total += dist;
					num_dists++;

					dists.push_back(dist);
				}
			}

			// Estimate the median of the distances and compute the average reprojection error for this camera
			double median	= util::GetNthElement(util::iround(0.8 * dists.size()), dists);
			double thresh	= util::clamp(2.4 * median, m_min_proj_error_threshold, m_max_proj_error_threshold);
			double avg		= std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
			wxLogMessage("[RunSFM] Mean error cam %d[%d] [%d pts]: %.3f [med: %.3f, outlier threshold: %.3f]", i, added_order[i], num_pts_proj, avg, median, thresh);

			pt_count = 0;
			for (int j = 0; j < num_keys; j++)
			{
				int pt_idx = GetKey(added_order[i], j).m_extra;
				if (pt_idx < 0) continue;

				if (dists[pt_count] > thresh)
				{
					// Remove this point from consideration
					bool found = false;
					for (int outlier : outliers) if (outlier == pt_idx) found = true;

					if (!found)
					{
						outliers.push_back(pt_idx);
						reproj_errors.push_back(dists[pt_count]);
					}
				}
				pt_count++;
			}
		}

		// Remove outlying points
		for (int i = 0; i < (int) outliers.size(); i++)
		{
			int idx = outliers[i];

			wxLogMessage("[RunSFM] Removing outlier %d (reproj error: %0.3f)", idx, reproj_errors[i]);

			if (colors != nullptr)
			{
				Vx(colors[idx]) = 0x0;
				Vy(colors[idx]) = 0x0;
				Vz(colors[idx]) = 0xff;
			}

			int num_views = (int)pt_views[idx].size();

			for (int j = 0; j < num_views; j++)
			{
				int v = pt_views[idx][j].first;
				int k = pt_views[idx][j].second;

				vmask[idx * num_cameras + v] = 0;

				// Sanity check
				if (GetKey(added_order[v], k).m_extra != idx) wxLogMessage("Error! Entry for (%d,%d) should be %d, but is %d", added_order[v], k, idx, GetKey(added_order[v], k).m_extra);

				GetKey(added_order[v], k).m_extra = -2;
			}

			pt_views[idx].clear();
		}

		num_outliers = outliers.size();
		total_outliers += num_outliers;

		end = clock();
		wxLogMessage("[RunSFM] Outlier removal took %0.3f s", (double) (end - start) / (double) CLOCKS_PER_SEC);
		wxLogMessage("[RunSFM] Removed %d outliers", num_outliers);

		for (int i = 0; i < num_pts; i++) if (remap[i] != -1) init_pts[i] = nz_pts[remap[i]];

		// Update points for display
		{
			wxCriticalSectionLocker lock(m_points_cs);
			m_points.clear();

			for (int i = 0; i < num_pts; i++)
			{
				// Check if the point is visible in any view
				if ((int) pt_views[i].size() == 0) continue;	// Invisible
	
				PointData pdata;
				pdata.m_pos[0] = Vx(init_pts[i]);
				pdata.m_pos[1] = Vy(init_pts[i]);
				pdata.m_pos[2] = Vz(init_pts[i]);

				pdata.m_color[0] = ((float) Vx(colors[i]))/255.0f;
				pdata.m_color[1] = ((float) Vy(colors[i]))/255.0f;
				pdata.m_color[2] = ((float) Vz(colors[i]))/255.0f;

				for (int j = 0; j < (int) pt_views[i].size(); j++)
				{
					int v = pt_views[i][j].first;
					int vnew = added_order[v];
					pdata.m_views.push_back(ImageKey(vnew, pt_views[i][j].second));
				}

				m_points.push_back(pdata);
			}
		}

		wxQueueEvent(this, new wxThreadEvent(wxEVT_THREAD_UPDATE));

	} while (num_outliers > 0);

	return dist_total / num_dists;
}
