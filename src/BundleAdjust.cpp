#include <fstream>
#include <numeric>

#include "EigenTools.hpp"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "matrix.h"

#include "FivePoint.hpp"
#include "MainFrame.hpp"
#include "Projection.hpp"
#include "Triangulation.hpp"
#include "Utilities.hpp"

namespace
{

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError
{
	SnavelyReprojectionError(double observed_x, double observed_y)
		: observed_x(observed_x), observed_y(observed_y)
	{}

	template <typename T>
	bool operator()(const T* const camera, const T* const point, T* residuals) const
	{
		// Camera[0, 1, 2] are the angle-axis rotation
		T p[3];
		ceres::AngleAxisRotatePoint(camera, point, p);

		// camera[3,4,5] are the translation.
		p[0] += camera[3];
		p[1] += camera[4];
		p[2] += camera[5];

		// Compute the center of distortion. The sign change comes from
		// the camera model that Noah Snavely's Bundler assumes, whereby
		// the camera coordinate system has a negative z axis.
		const T& focal = camera[6];
		T xp = -p[0] * focal / p[2];
		T yp = -p[1] * focal / p[2];

		// Apply second and fourth order radial distortion
		const T& l1 = camera[7];
		const T& l2 = camera[8];
		T r2 = xp*xp + yp*yp;
		T distortion = T(1.0) + r2  * (l1 + l2  * r2);

		// Compute final projected point position
		T predicted_x = /*focal **/ distortion * xp;
		T predicted_y = /*focal **/ distortion * yp;

		// The error is the difference between the predicted and observed position
		residuals[0] = predicted_x - T(observed_x);
		residuals[1] = predicted_y - T(observed_y);

		return true;
	}

	// Factory to hide the construction of the CostFunction object from the client code
	static ceres::CostFunction* Create(const double observed_x, const double observed_y)
	{
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
					new SnavelyReprojectionError(observed_x, observed_y)));
	}

	double observed_x;
	double observed_y;
};

// Penalize a camera variable for deviating from a given prior value
struct PriorError
{
	PriorError(int prior_index, double prior_value, double prior_scale)
		: prior_index(prior_index)
		, prior_value(prior_value)
		, prior_scale(prior_scale)
	{}

	template <typename T>
	bool operator()(const T* const x, T* residual) const
	{
		residual[0] = prior_scale * (prior_value - x[prior_index]);
		return true;
	}

	int prior_index;
	double prior_value;
	double prior_scale;
};

// Go from a vector representing a rotation in axis-angle format to a 3x3 rotation matrix in column major form
static void aa2rot(double const * x, double * R)
{
	const double epsilon = 1e-18;
	double theta = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
	double wx = x[0]/(theta+epsilon);
	double wy = x[1]/(theta+epsilon);
	double wz = x[2]/(theta+epsilon);

	double costheta = cos(theta);
	double sintheta = sin(theta);

	R[0] = costheta + wx*wx*(1-costheta);
	R[1] = wz*sintheta + wx*wy*(1-costheta);
	R[2] = -wy*sintheta + wx*wz*(1-costheta);
	R[3] = wx*wy*(1-costheta) - wz*sintheta;
	R[4] = costheta + wy*wy*(1-costheta);
	R[5] = wx*sintheta + wy*wz*(1-costheta);
	R[6] = wy*sintheta + wx*wz*(1-costheta);
	R[7] = -wx*sintheta + wy*wz*(1-costheta);
	R[8] = costheta + wz*wz*(1-costheta);
};

// Rotation to axis angle vector format
static void rot2aa(double *R, double *aa)
{
	double tr = R[0] + R[4] + R[8];
	double costheta = util::clamp(0.5 * (tr - 1.0), -1.0, 1.0);

	double RT[9], RRT[9];
	matrix_transpose(3, 3, R, RT);
	matrix_diff(3, 3, 3, 3, R, RT, RRT);
	double sintheta = matrix_norm(3, 3, RRT) / sqrt(8.0);

	double theta = atan2(sintheta, costheta);
	double factor = theta / (2.0 * sintheta + 1.0e-10);

	aa[0] = factor * (R[7] - R[5]);
	aa[1] = factor * (R[2] - R[6]);
	aa[2] = factor * (R[3] - R[1]);
}

}

void MainFrame::StartBundlerThread()
{
	if (CreateThread(wxTHREAD_DETACHED) != wxTHREAD_NO_ERROR)
	{
		wxLogError("Could not create the worker thread!");
		return;
	}

	if (GetThread()->Run() != wxTHREAD_NO_ERROR)
	{
		wxLogError("Could not run the worker thread!");
		return;
	}
}

wxThread::ExitCode MainFrame::Entry()
{
	this->BundleAdjust();

	return (wxThread::ExitCode)0;
}

void MainFrame::BundleAdjust()
{
	wxLogMessage("[BundleAdjust] Computing structure from motion...");
	clock_t start = clock();

	// Reset track to key mappings and vice versa
	for (auto &track : m_tracks) track.m_extra = -1;
	for (auto &img : m_images) for (auto &key : img.m_keys) key.m_extra = -1;

	int num_images	= this->GetNumImages();
	int max_pts		= (int)m_tracks.size();
	std::vector<int>				added_order(num_images);
	std::vector<camera_params_t>	cameras(num_images);
	std::vector<v3_t>				points(max_pts);
	std::vector<v3_t>				colors(max_pts);
	std::vector<ImageKeyVector>		pt_views;

	int i_best = -1, j_best = -1;
	this->BundlePickInitialPair(i_best, j_best);
	added_order[0] = i_best;
	added_order[1] = j_best;

	wxLogMessage("[BundleAdjust] Adjusting cameras %d and %d...", i_best, j_best);
	int curr_num_pts = this->SetupInitialCameraPair(i_best, j_best, cameras, points, colors, pt_views);
	RunSFM(curr_num_pts, 2, cameras.data(), points.data(), added_order, colors.data(), pt_views);
	int curr_num_cameras = 2;

	// Main loop
	int round = 0;
	while (curr_num_cameras < num_images)
	{
		int max_matches = 0;
		int max_cam = this->FindCameraWithMostMatches(curr_num_cameras, added_order, max_matches, pt_views);
		wxLogMessage("[BundleAdjust] Max_matches = %d", max_matches);

		if (max_matches < m_options.min_max_matches)
		{
			wxLogMessage("[BundleAdjust] No more connections!");
			break;
		}

		auto image_set = this->FindCamerasWithNMatches(util::iround(0.75 * max_matches), curr_num_cameras, added_order, pt_views);
		wxLogMessage("[BundleAdjust] Registering %d images", (int)image_set.size());

		// Now, throw the new cameras into the mix
		int image_count = 0;
		for (const int &image_idx : image_set)
		{
			wxLogMessage("[BundleAdjust[round %i]] Adjusting camera %d", round, image_idx);
			added_order[curr_num_cameras + image_count] = image_idx;

			bool success = false;
			auto camera_new = this->BundleInitializeImage(image_idx, curr_num_cameras + image_count, points.data(), cameras.data(), pt_views, success);
			if (success)
			{
				cameras[curr_num_cameras + image_count] = camera_new;
				image_count++;
			} else
			{
				wxLogMessage("[BundleAdjust] Couldn't initialize image %d", image_idx);
				m_images[image_idx].m_ignore_in_bundle = true;
			}
		}

		wxLogMessage("[BundleAdjust] Adding new matches");
		curr_num_cameras += image_count;
		curr_num_pts = this->BundleAdjustAddAllNewPoints(curr_num_pts, curr_num_cameras, added_order, cameras.data(), points.data(), colors.data(), pt_views);
		wxLogMessage("[BundleAdjust] Number of points = %d", curr_num_pts);

		RunSFM(curr_num_pts, curr_num_cameras, cameras.data(), points.data(), added_order, colors.data(), pt_views);

		this->RemoveBadPointsAndCameras(curr_num_pts, curr_num_cameras + 1, added_order, cameras.data(), points.data(), colors.data(), pt_views);

		wxLogMessage("[BundleAdjust] Focal lengths:");
		for (int i = 0; i < curr_num_cameras; i++)
		{
			wxLogMessage("  [%03d] %.3f %s %d; %.3f %.3f", i, cameras[i].f, m_images[added_order[i]].m_filename_short.c_str(), added_order[i], cameras[i].k[0], cameras[i].k[1]);
		}

		// Update points for display
		{
			wxCriticalSectionLocker lock(m_points_cs);
			m_points.clear();

			for (int i = 0; i < curr_num_pts; i++)
			{
				// Check if the point is visible in any view
				if ((int) pt_views[i].size() == 0) continue;	// Invisible
	
				PointData pdata;
				pdata.m_pos[0] = Vx(points[i]);
				pdata.m_pos[1] = Vy(points[i]);
				pdata.m_pos[2] = Vz(points[i]);

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
		round++;
	}

	clock_t end = clock();
	wxLogMessage("[BundleAdjust] Computing structure from motion took %0.3f s", (double) (end - start) / (double) CLOCKS_PER_SEC);

	this->SavePlyFile();
	this->SetMatchesFromPoints();

	// Update program state
	m_sfm_done = true;
}

void MainFrame::BundlePickInitialPair(int &i_best, int &j_best)
{
	int		num_images		= this->GetNumImages();
	int		min_matches		= 80;
	int		max_matches		= 0;
	double	min_score		= 0.1;
	double	max_score		= 0.0;
	double	max_score_2		= 0.0;
	double	score_threshold	= 2.0;
	int		match_threshold = 32;

	int		i_best_2 = -1, j_best_2 = -1;

	// Compute score for each image pair
	int max_pts = 0;
	for (int i = 0; i < num_images; i++)
	{
		for (int j = i + 1; j < num_images; j++)
		{
			int num_matches = this->GetNumTrackMatches(i, j);
			max_pts += num_matches;

			if (num_matches <= match_threshold) continue;

			double score = 0.0;
			double ratio = this->GetInlierRatio(i, j);
			
			if (ratio == 0.0)	score = min_score;
			else				score = 1.0 / ratio;

			// Compute the primary score
			if ((num_matches > max_matches) && (score > score_threshold))
			{
				max_matches = num_matches;
				max_score = score;
				i_best = i;
				j_best = j;
			}

			// Compute the backup score
			if ((num_matches > min_matches) && (score > max_score_2))
			{
				max_score_2 = score;
				i_best_2 = i;
				j_best_2 = j;
			}
		}
	}

	if (i_best == -1 && j_best == -1)
	{
		if (i_best_2 == -1 && j_best_2 == -1)
		{
			wxLogMessage("[BundlePickInitialPair] Error: no good camera pairs found, picking first two cameras...!");

			i_best = 0;
			j_best = 1;
		} else
		{
			i_best = i_best_2;
			j_best = j_best_2;
		}
	}
}

int MainFrame::SetupInitialCameraPair(int i_best, int j_best, std::vector<camera_params_t> &cameras,
									std::vector<v3_t> &points, std::vector<v3_t> &colors, std::vector<ImageKeyVector> &pt_views)
{
	this->SetMatchesFromTracks(i_best, j_best);

	m_images[i_best].SetTracks();
	m_images[j_best].SetTracks();

	this->InitializeCameraParams(cameras[0]);
	this->InitializeCameraParams(cameras[1]);
	cameras[0].f = cameras[0].init_f = m_images[i_best].m_init_focal;
	cameras[1].f = cameras[1].init_f = m_images[j_best].m_init_focal;

	double K1[9], K2[9];
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Ke1(K1);
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Ke2(K2);
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R1(cameras[0].R);
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R2(cameras[1].R);
	Eigen::Map<Eigen::Vector3d> t1(cameras[0].t);
	Eigen::Map<Eigen::Vector3d> t2(cameras[1].t);

	// Put first camera at origin
	R1.setIdentity();
	t1.setZero();

	// Solve for initial locations
	EstimateRelativePose(i_best, j_best, cameras[0], cameras[1]);

	// Triangulate the initial 3D points
	wxLogMessage("[SetupInitialCameraPair] Adding initial matches...");

	this->GetIntrinsics(cameras[0], K1);
	this->GetIntrinsics(cameras[1], K2);

	Mat3 K1_inv = Ke1.inverse();
	Mat3 K2_inv = Ke2.inverse();

	int pt_count = 0;
	auto &list = m_matches.GetMatchList(GetMatchIndex(i_best, j_best));
	int num_matches = list.size();

	for (int i = 0; i < num_matches; i++)
	{
		// Set up the 3D point
		int key_idx1(list[i].m_idx1);
		int key_idx2(list[i].m_idx2);

		// Normalize the point
		Vec3 p_norm1 = K1_inv * Vec3(m_images[i_best].m_keys[key_idx1].m_x, m_images[i_best].m_keys[key_idx1].m_y, -1.0);
		Vec3 p_norm2 = K2_inv * Vec3(m_images[j_best].m_keys[key_idx2].m_x, m_images[j_best].m_keys[key_idx2].m_y, -1.0);

		// Put the translation in standard form
		Observations observations;
		observations.push_back(Observation(Vec2(p_norm1.head<2>() / p_norm1.z()), R1, -(R1 * t1)));
		observations.push_back(Observation(Vec2(p_norm2.head<2>() / p_norm2.z()), R2, -(R2 * t2)));

		double error;
		auto pt = Triangulate(observations, &error);
		points[pt_count] = v3_new(pt(0), pt(1), pt(2));

		error = (cameras[0].f + cameras[1].f) * 0.5 * sqrt(error * 0.5);
		
		if (error > m_options.projection_estimation_threshold) continue;

		// Get the color of the point
		auto &key = GetKey(i_best, key_idx1);
		colors[pt_count] = v3_new((double) key.m_r, (double)key.m_g, (double) key.m_b);

		GetKey(i_best, key_idx1).m_extra = pt_count;
		GetKey(j_best, key_idx2).m_extra = pt_count;

		int track_idx = GetKey(i_best, key_idx1).m_track;
		m_tracks[track_idx].m_extra = pt_count;

		ImageKeyVector views;
		views.push_back(ImageKey(0, key_idx1));
		views.push_back(ImageKey(1, key_idx2));
		pt_views.push_back(views);

		pt_count++;
	}

	return pt_count;
}

void MainFrame::SetMatchesFromTracks(int img1, int img2)
{
	auto &matches = m_matches.GetMatchList(GetMatchIndex(img1, img2));
	auto &tracks1 = m_images[img1].m_visible_points;
	auto &tracks2 = m_images[img2].m_visible_points;

	// Find tracks visible from both cameras
	std::vector<int> intersection;
	std::set_intersection(tracks1.begin(), tracks1.end(), tracks2.begin(), tracks2.end(), std::inserter(intersection, intersection.begin()));

	if (intersection.empty()) return;

	matches.clear();
	matches.reserve(intersection.size());

	for (const int track : intersection)
	{
		auto p = std::lower_bound(tracks1.begin(), tracks1.end(), track);
		auto offset = std::distance(tracks1.begin(), p);
		int key1 = m_images[img1].m_visible_keys[offset];

		p = std::lower_bound(tracks2.begin(), tracks2.end(), track);
		offset = std::distance(tracks2.begin(), p);
		int key2 = m_images[img2].m_visible_keys[offset];

		matches.push_back(KeypointMatch(key1, key2));
	}
}

void MainFrame::SetMatchesFromPoints()
{
	wxLogMessage("[SetMatchesFromPoints] Setting up matches...");

	// Clear all matches
	m_matches.RemoveAll();

	wxCriticalSectionLocker lock(m_points_cs);
	int num_points = (int) m_points.size();
	for (int i = 0; i < num_points; i++) {
		int num_views = (int) m_points[i].m_views.size();

		for (int j = 0; j < num_views; j++) {
			for (int k = 0; k < num_views; k++) {
				if (j == k) continue;

				ImageKey view1 = m_points[i].m_views[j];
				ImageKey view2 = m_points[i].m_views[k];

				SetMatch(view1.first, view2.first);
				MatchIndex idx = GetMatchIndex(view1.first, view2.first);
				m_matches.AddMatch(idx, KeypointMatch(view1.second, view2.second));
			}
		}
	}

	wxLogMessage("[SetMatchesFromPoints] Done!");
}

void MainFrame::InitializeCameraParams(camera_params_t &camera)
{
	matrix_ident(3, camera.R);
	camera.t[0] = camera.t[1] = camera.t[2] = 0.0;
	camera.f = camera.init_f = 0.0;
	camera.k[0] = camera.k[1] = 0.0;

	camera.k_inv[0] = camera.k_inv[2] = camera.k_inv[3] = 0.0;
	camera.k_inv[4] = camera.k_inv[5] = 0.0;
	camera.k_inv[1] = 1.0;
}

int MainFrame::FindCameraWithMostMatches(int num_cameras, const std::vector<int> &added_order, int &max_matches, const std::vector<ImageKeyVector> &pt_views)
{
	int i_best = -1;
	max_matches = 0;

	for (int i = 0; i < GetNumImages(); i++)
	{
		if (m_images[i].m_ignore_in_bundle) continue;

		// Check if we added this image already
		bool added = false;
		for (int j = 0; j < num_cameras; j++)
		{
			if (added_order[j] == i)
			{
				added = true;
				break;
			}
		}

		if (added) continue;

		int num_existing_matches = 0;

		// Find the tracks seen by this image
		auto &tracks = m_images[i].m_visible_points;
		int num_tracks = (int) tracks.size();

		for (int j = 0; j < num_tracks; j++)
		{
			int tr(tracks[j]);
			if (m_tracks[tr].m_extra < 0) continue;

			// This tracks corresponds to a point
			int pt = m_tracks[tr].m_extra;
			if ((int) pt_views[pt].size() == 0) continue;

			num_existing_matches++;
		}

		if (num_existing_matches > 0) wxLogMessage("[FindCameraWithMostMatches] Existing matches[%d] = %d", i, num_existing_matches);
		if (num_existing_matches > max_matches)
		{
			i_best = i;
			max_matches = num_existing_matches;
		}
	}

	return i_best;
}

std::vector<int> MainFrame::FindCamerasWithNMatches(int n, int num_cameras, const std::vector<int> &added_order, const std::vector<ImageKeyVector> &pt_views)
{
	std::vector<int> found_cameras;

	for (int i = 0; i < GetNumImages(); i++)
	{
		if (m_images[i].m_ignore_in_bundle) continue;

		// Check if we added this image already
		bool added = false;
		for (int j = 0; j < num_cameras; j++)
		{
			if (added_order[j] == i)
			{
				added = true;
				break;
			}
		}

		if (added) continue;

		int num_existing_matches = 0;
		int parent_idx_best = -1;

		// Find the tracks seen by this image
		const auto &tracks = m_images[i].m_visible_points;
		int num_tracks = (int) tracks.size();

		for (int j = 0; j < num_tracks; j++)
		{
			int tr = tracks[j];
			if (m_tracks[tr].m_extra < 0) continue;

			// This tracks corresponds to a point
			int pt = m_tracks[tr].m_extra;
			if ((int) pt_views[pt].size() == 0) continue;

			num_existing_matches++;
		}

		if (num_existing_matches >= n) found_cameras.push_back(i);
	}

	wxLogMessage("[FindCamerasWithNMatches] Found %i cameras with at least %i matches", found_cameras.size(), n);

	return found_cameras;
}

camera_params_t	MainFrame::BundleInitializeImage(int image_idx, int camera_idx, v3_t *points, camera_params_t *cameras, std::vector<ImageKeyVector> &pt_views, bool &success_out)
{
	clock_t start = clock();
	camera_params_t dummy;
	dummy.f = 0.0;

	success_out = false;

	auto &image = m_images[image_idx];
	image.SetTracks();

	// **** Connect the new camera to any existing points ****
	Vec3Vec				points_solve;
	Vec2Vec				projs_solve;
	std::vector<int>	idxs_solve;
	std::vector<int>	keys_solve;

	wxLogMessage("[BundleInitializeImage] Connecting existing matches...");

	// Find the tracks seen by this image
	for (int i = 0; i < image.m_visible_points.size(); i++)
	{
		int track	= image.m_visible_points[i];
		int key		= image.m_visible_keys[i];

		if (m_tracks[track].m_extra < 0) continue;

		// This tracks corresponds to a point
		int pt(m_tracks[track].m_extra);
		if (pt_views[pt].empty()) continue;

		// Add the point to the set we'll use to solve for the camera position
		points_solve.push_back(Vec3(points[pt].p[0], points[pt].p[1], points[pt].p[2]));
		projs_solve.push_back(Vec2(image.m_keys[key].m_x, image.m_keys[key].m_y));
		idxs_solve.push_back(pt);
		keys_solve.push_back(key);
	}

	if (points_solve.size() < m_options.min_max_matches)
	{
		wxLogMessage("[BundleInitializeImage] Couldn't initialize (too few points)");
		return dummy;
	}

	// **** Solve for the camera position ****
	wxLogMessage("[BundleInitializeImage] Initializing camera...");

	double Kinit[9], Rinit[9], tinit[3];
	camera_params_t camera_new;
	std::vector<int> inliers, inliers_weak, outliers;
	bool found = this->FindAndVerifyCamera(points_solve, projs_solve, idxs_solve.data(),
											Kinit, Rinit, tinit,
											inliers, inliers_weak, outliers);

	if (!found)
	{
		wxLogMessage("[BundleInitializeImage] Couldn't initialize (couldn't solve for the camera position)");
		return dummy;
	} else
	{
		// Start with the new camera at same place as the best match
		this->InitializeCameraParams(camera_new);

		// Set up the new camera
		memcpy(camera_new.R, Rinit, 9 * sizeof(double));

		matrix_transpose_product(3, 3, 3, 1, Rinit, tinit, camera_new.t);
		matrix_scale(3, 1, camera_new.t, -1.0, camera_new.t);

		// Set up the new focal length
		camera_new.f = camera_new.init_f = image.m_init_focal;
		wxLogMessage("[BundleInitializeImage] Camera has initial focal length of %0.3f", camera_new.init_f);
	}

	// **** Finally, start the bundle adjustment ****
	wxLogMessage("[BundleInitializeImage] Adjusting...");

	int num_inliers = (int)inliers_weak.size();

	Vec3Vec				points_final;
	Vec2Vec				projs_final;
	std::vector<int>	idxs_final;
	std::vector<int>	keys_final;

	for (const auto &idx : inliers_weak)
	{
		points_final.push_back(points_solve[idx]);
		projs_final.push_back(projs_solve[idx]);
		idxs_final.push_back(idxs_solve[idx]);
		keys_final.push_back(keys_solve[idx]);
	}

	ECamera cam;
	cam.m_focal_length = camera_new.f;
	cam.m_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_new.R);
	cam.m_t = Eigen::Map<Vec3>(camera_new.t);
	cam.m_k = Eigen::Map<Vec2>(camera_new.k);

	this->RefineCameraParameters(&cam, points_final, projs_final, idxs_final.data(), inliers);

	camera_new.f = cam.m_focal_length;

	if ((inliers.size() < 8) || (camera_new.f < 0.1 * image.GetWidth()))
	{
		wxLogMessage("[BundleInitializeImage] Bad camera");
		return dummy;
	}

	// Point the keys to their corresponding points
	for (const auto &i : inliers)
	{
		image.m_keys[keys_final[i]].m_extra = idxs_final[i];
		pt_views[idxs_final[i]].push_back(ImageKey(camera_idx, keys_final[i]));
	}

	clock_t end = clock();

	wxLogMessage("[BundleInitializeImage] Initializing took %0.3f s", (double) (end - start) / CLOCKS_PER_SEC);

	image.m_camera.m_adjusted = true;

	success_out = true;
	return camera_new;
}

bool MainFrame::EstimateRelativePose(int i1, int i2, camera_params_t &camera1, camera_params_t &camera2)
{
	auto &matches	= m_matches.GetMatchList(GetMatchIndex(i1, i2));
	auto &keys1		= m_images[i1].m_keys;
	auto &keys2		= m_images[i2].m_keys;

	double K1[9], K2[9];
	Mat3 R;
	Vec3 t;
	this->GetIntrinsics(camera1, K1);
	this->GetIntrinsics(camera2, K2);

	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K1e(K1);
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K2e(K2);

	Vec2Vec k1_pts, k2_pts;
	k1_pts.reserve(matches.size());
	k2_pts.reserve(matches.size());

	for (const auto &match : matches)
	{
		k1_pts.push_back(Vec2(keys1[match.m_idx1].m_x, keys1[match.m_idx1].m_y));
		k2_pts.push_back(Vec2(keys2[match.m_idx2].m_x, keys2[match.m_idx2].m_y));
	}

	wxLogMessage("[EstimateRelativePose] EstimateRelativePose starting...");
	clock_t start = clock();
	int num_inliers = ComputeRelativePoseRansac(k1_pts, k2_pts, K1e, K2e, m_options.ransac_threshold_five_point, m_options.ransac_rounds_five_point, &R, &t);
	clock_t end = clock();
	wxLogMessage("[EstimateRelativePose] EstimateRelativePose took %0.3f s", (double) (end - start) / (double) CLOCKS_PER_SEC);
	wxLogMessage("[EstimateRelativePose] Found %d / %d inliers (%0.3f%%)", num_inliers, matches.size(), 100.0 * num_inliers / matches.size());

	if (num_inliers == 0) return false;

	m_images[i1].m_camera.m_adjusted = true;
	m_images[i2].m_camera.m_adjusted = true;

	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Re(camera2.R);
	Eigen::Map<Eigen::Vector3d> te(camera2.t);

	Re = R;
	te = -(R.transpose() * t);

	return true;
}

double MainFrame::RunSFM(int num_pts, int num_cameras, camera_params_t *init_camera_params, v3_t *init_pts, const std::vector<int> &added_order,
							v3_t *colors, std::vector<ImageKeyVector> &pt_views)
{
	const int		min_points			= 20;
	int				round				= 0;
	int				num_outliers		= 0;
	int				total_outliers		= 0;
	int				num_dists			= 0;
	double			dist_total			= 0.0;
	const double	huber_parameter		= 25.0;

	std::vector<int>	remap(num_pts);
	std::vector<v3_t>	nz_pts(num_pts);

	ceres::Problem problem;

	clock_t start_all, stop_all;
	start_all = clock();

	do
	{
		clock_t round_start, round_stop;
		round_start = clock();

		if (num_pts - total_outliers < min_points)
		{
			wxLogMessage("[RunSFM] Too few points remaining, exiting!");
			dist_total = std::numeric_limits<double>::max();
			break;
		}

		// Set up the projections
		int num_projections = 0;
		for (const auto &views : pt_views) num_projections += views.size();
		std::vector<double>			projections(2 * num_projections);
		std::vector<int>			pidx(num_projections);
		std::vector<int>			cidx(num_projections);
		std::vector<unsigned int>	num_vis(num_cameras, 0);

		int arr_idx = 0;
		int nz_count = 0;
		for (int i = 0; i < num_pts; i++)
		{
			int num_views = (int)pt_views[i].size();

			if (num_views > 0)
			{
				for (int j = 0; j < num_views; j++)
				{
					int c = pt_views[i][j].first;
					int v = added_order[c];
					int k = pt_views[i][j].second;

					projections[2 * arr_idx + 0] = GetKey(v, k).m_x;
					projections[2 * arr_idx + 1] = GetKey(v, k).m_y;

					pidx[arr_idx] = nz_count;
					cidx[arr_idx] = c;

					num_vis[c]++;

					arr_idx++;
				}

				remap[i] = nz_count;
				nz_pts[nz_count] = init_pts[i];
				nz_count++;
			} else
			{
				remap[i] = -1;
			}
		}

		// Set up initial parameters
		int num_nz_points = nz_count;

		int cnp = 9;
		int pnp = 3;

		unsigned int num_parameters = pnp * num_nz_points + cnp * num_cameras;

		double *init_x = new double[num_parameters];

		double *cameras = init_x;
		double *points = init_x + cnp * num_cameras;

		// Insert camera parameters
		double *ptr = init_x;

		int idx = 0;
		for (int i = 0; i < num_cameras; i++)
		{
			// Get the rotation
			double axis[3];
			rot2aa(init_camera_params[i].R, axis);

			double *c = init_camera_params[i].t;
			double t[3];
			matrix_product331(init_camera_params[i].R, c, t);
			matrix_scale(3, 1, t, -1.0, t);

			double f = init_camera_params[i].f;
			double k1 = init_camera_params[i].k[0];
			double k2 = init_camera_params[i].k[1];

			ptr[idx] = axis[0]; idx++;
			ptr[idx] = axis[1]; idx++;
			ptr[idx] = axis[2]; idx++;
			ptr[idx] = t[0]; idx++;
			ptr[idx] = t[1]; idx++;
			ptr[idx] = t[2]; idx++;
			ptr[idx] = f; idx++;
			ptr[idx] = k1 / (f * f); idx++;
			ptr[idx] = k2 / (f * f * f * f); idx++;
		}

		// Insert point parameters
		for (int i = 0; i < num_pts; i++)
		{
			if (pt_views[i].size() > 0)
			{
				ptr[idx] = Vx(init_pts[i]); idx++;
				ptr[idx] = Vy(init_pts[i]); idx++;
				ptr[idx] = Vz(init_pts[i]); idx++;
			}
		}

		for (int i = 0; i < num_projections; ++i)
		{
			// Each Residual block takes a point and a camera as input and outputs a 2 dimensional residual.
			ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(projections[2 * i + 0], projections[2 * i + 1]);

			// If enabled use Huber's loss function.
			ceres::LossFunction *loss_function = nullptr;

			//loss_function = new ceres::HuberLoss(huber_parameter);

			// Each observation correponds to a pair of a camera and a point which are identified by camera_index()[i] and point_index()[i] respectively.
			double *camera	= cameras	+ cnp * cidx[i];
			double *point	= points	+ pnp * pidx[i];

			problem.AddResidualBlock(cost_function, loss_function, camera, point);
		}

		// Now add the priors
		for (int i = 0; i < num_cameras; i++)
		{
			ceres::CostFunction *prior_cost_function;

			prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>(new PriorError(6, init_camera_params[i].init_f, m_options.focal_length_constrain_weight* num_vis[i]));
			problem.AddResidualBlock(prior_cost_function, nullptr, cameras + cnp * i);

			// Take care of priors on distortion parameters
			prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>(new PriorError(7, 0.0, m_options.distortion_constrain_weight * num_vis[i]));
			problem.AddResidualBlock(prior_cost_function, nullptr, cameras + cnp * i);

			prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>(new PriorError(8, 0.0, m_options.distortion_constrain_weight * num_vis[i]));
			problem.AddResidualBlock(prior_cost_function, nullptr, cameras + cnp * i);
		}

		dist_total = 0.0;
		num_dists = 0;

		clock_t start = clock();

		// Make call to Ceres
		wxLogMessage("[Ceres] %d points",		num_nz_points);
		wxLogMessage("[Ceres] %d cameras",		num_cameras);
		wxLogMessage("[Ceres] %d params",		num_parameters);
		wxLogMessage("[Ceres] %d projections",	num_projections);

		ceres::Solver::Options options;
		ceres::Solver::Summary summary;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		ceres::Solve(options, &problem, &summary);
		
		wxLogMessage("%s", summary.BriefReport().c_str());

		double *final_x	= init_x;
		ptr				= final_x;
		for (int i = 0; i < num_cameras; i++)
		{
			// Get the rotation
			double axis[3];

			axis[0] = *ptr; ptr++;
			axis[1] = *ptr; ptr++;
			axis[2] = *ptr; ptr++;

			double RT[9];
			aa2rot(axis, RT);
			matrix_transpose(3, 3, RT, init_camera_params[i].R);

			double t[3];
			double *c = init_camera_params[i].t;

			t[0] = *ptr; ptr++;
			t[1] = *ptr; ptr++;
			t[2] = *ptr; ptr++;

			matrix_transpose_product(3, 3, 3, 1, init_camera_params[i].R, t, c);
			matrix_scale(3, 1, c, -1.0, c);

			double f = init_camera_params[i].f = *ptr; ptr++;

			init_camera_params[i].k[0] = *ptr * (f * f); ptr++;
			init_camera_params[i].k[1] = *ptr * (f * f * f * f); ptr++;
		}

		// Insert point parameters
		for (int i = 0; i < num_nz_points; i++)
		{
			Vx(nz_pts[i]) = *ptr; ptr++;
			Vy(nz_pts[i]) = *ptr; ptr++;
			Vz(nz_pts[i]) = *ptr; ptr++;
		}

		clock_t end = clock();
	
		wxLogMessage("[RunSFM] RunSFM took %0.3fs", (double) (end - start) / (double) CLOCKS_PER_SEC);

		// Check for outliers
		start = clock();

		std::vector<int>	outliers;
		std::vector<double>	reproj_errors;

		for (int i = 0; i < num_cameras; i++)
		{
			auto &data = m_images[added_order[i]];

			double K[9];
			GetIntrinsics(init_camera_params[i], K);
			double dt[3] = { init_camera_params[i].t[0], init_camera_params[i].t[1], init_camera_params[i].t[2] };

			// Compute inverse distortion parameters
			Vec6 k_dist; k_dist << 0.0, 1.0, 0.0, init_camera_params[i].k[0], 0.0, init_camera_params[i].k[1];
			double w_2 = 0.5 * data.GetWidth();
			double h_2 = 0.5 * data.GetHeight();
			double max_radius = sqrt(w_2 * w_2 + h_2 * h_2) / init_camera_params[i].f;
			Eigen::Map<Vec6> k_inv(init_camera_params[i].k_inv);

			k_inv = InvertDistortion(0.0, max_radius, k_dist);

			int num_keys = GetNumKeys(added_order[i]);
			int num_pts_proj = 0;
			for (int j = 0; j < num_keys; j++) if (GetKey(added_order[i], j).m_extra >= 0) num_pts_proj++;

			std::vector<double> dists;

			ECamera cam;
			cam.m_focal_length = init_camera_params[i].f;
			cam.m_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(init_camera_params[i].R);
			cam.m_t = Eigen::Map<Vec3>(init_camera_params[i].t);
			cam.m_k << init_camera_params[i].k[0], init_camera_params[i].k[1];

			for (const auto &key : data.m_keys)
			{
				if (key.m_extra >= 0)
				{
					double b[3];
					int pt_idx = key.m_extra;

					b[0] = Vx(nz_pts[remap[pt_idx]]);
					b[1] = Vy(nz_pts[remap[pt_idx]]);
					b[2] = Vz(nz_pts[remap[pt_idx]]);

					Vec2 pr = SfmProjectRD(Vec3(b[0], b[1], b[2]), cam);

					double dist = (pr - Vec2(key.m_x, key.m_y)).norm();

					dists.push_back(dist);
					dist_total += dist;
					num_dists++;
				}
			}

			// Estimate the median of the distances and compute the average reprojection error for this camera
			double median	= util::GetNthElement(util::iround(0.8 * dists.size()), dists);
			double thresh	= util::clamp(2.4 * median, m_options.min_reprojection_error_threshold, m_options.max_reprojection_error_threshold);
			double avg		= std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
			wxLogMessage("[RunSFM] Mean error cam %d[%d] [%d pts]: %.3f [med: %.3f, outlier threshold: %.3f]", i, added_order[i], num_pts_proj, avg, median, thresh);

			int pt_count = 0;
			for (int j = 0; j < num_keys; j++)
			{
				int pt_idx = GetKey(added_order[i], j).m_extra;
				if (pt_idx < 0) continue;

				if (dists[pt_count] > thresh)
				{
					// Remove this point from consideration
					bool found = false;
					for (const int &outlier : outliers) if (outlier == pt_idx)
					{
						found = true;
						break;
					}

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

				// Sanity check
				if (GetKey(added_order[v], k).m_extra != idx) wxLogMessage("Error! Entry for (%d,%d) should be %d, but is %d", added_order[v], k, idx, GetKey(added_order[v], k).m_extra);

				GetKey(added_order[v], k).m_extra = -2;
			}

			pt_views[idx].clear();
		}

		num_outliers = outliers.size();
		total_outliers += num_outliers;

		end = clock();
		wxLogMessage("[RunSFM] Removed %d outliers in %0.3f s", num_outliers, (double) (end - start) / (double) CLOCKS_PER_SEC);

		RemoveBadPointsAndCameras(num_pts, num_cameras, added_order, init_camera_params, init_pts, colors, pt_views);

		for (int i = 0; i < num_pts; i++) if (remap[i] != -1) init_pts[i] = nz_pts[remap[i]];

		round_stop = clock();
		wxLogMessage("[RunSFM] Round %d took %0.3f s", round, (double) (round_stop - round_start) / (double) CLOCKS_PER_SEC);

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
					pdata.m_views.push_back(ImageKey(added_order[v], pt_views[i][j].second));
				}

				m_points.push_back(pdata);
			}
		}

		wxQueueEvent(this, new wxThreadEvent(wxEVT_THREAD_UPDATE));

		round++;

	} while (num_outliers > m_options.outlier_threshold_ba);

	stop_all = clock();
	wxLogMessage("[RunSFM] Structure from motion with outlier removal took %0.3f s (%d rounds)", (double) (stop_all - start_all) / (double) CLOCKS_PER_SEC, round);

	return dist_total / num_dists;
}

bool MainFrame::FindAndVerifyCamera(const Vec3Vec &points, const Vec2Vec &projections, int *idxs_solve, double *K, double *R, double *t,
									 std::vector<int> &inliers, std::vector<int> &inliers_weak, std::vector<int> &outliers)
{
	// First, find the projection matrix
	int r = -1;

	Mat34 P;
	if (points.size() >= 9) r = ComputeProjectionMatrixRansac(points, projections, m_options.ransac_rounds_projection,
															m_options.projection_estimation_threshold * m_options.projection_estimation_threshold, &P);

	if (r == -1)
	{
		wxLogMessage("[FindAndVerifyCamera] Couldn't find projection matrix");
		return false;
	}

	// If number of inliers is too low, fail
	if (r <= 6) // 7, 30 This constant needs adjustment
	{
		wxLogMessage("[FindAndVerifyCamera] Too few inliers to use projection matrix");
		return false;
	}

	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Ke(K);
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Re(R);
	Eigen::Map<Vec3> te(t);
	Mat3 Ke2 = Ke;
	Mat3 Re2 = Re;
	Vec3 te2 = te;

	DecomposeProjectionMatrix(P, &Ke2, &Re2, &te2);

	Ke = Ke2;
	Re = Re2;
	te = te2;

	wxLogMessage("[FindAndVerifyCamera] Checking consistency...");

	Mat34 Rigid;
	Rigid << Re2;
	Rigid.col(3) = te;

	int num_behind = 0;
	for (int j = 0; j < points.size(); j++)
	{
		Vec3 q = Ke * (Rigid * Vec4(points[j].x(), points[j].y(), points[j].z(), 1.0));
		Vec2 pimg = -q.head<2>() / q.z();
		double diff = (pimg - projections[j]).norm();

		if (diff < m_options.projection_estimation_threshold) inliers.push_back(j);
		if (diff < (16.0 * m_options.projection_estimation_threshold))
		{
			inliers_weak.push_back(j);
		} else
		{
			wxLogMessage("[FindAndVerifyCamera] Removing point [%d] with reprojection error %0.3f", idxs_solve[j], diff);
			outliers.push_back(j);
		}

		if (q.z() > 0.0) num_behind++;	// Cheirality constraint violated
	}

	if (num_behind >= 0.9 * points.size())
	{
		wxLogMessage("[FindAndVerifyCamera] Error: camera is pointing away from scene");
		return false;
	}

	return true;
}

void MainFrame::GetIntrinsics(const camera_params_t &camera, double *K)
{
	K[0] = K[4] = camera.f;
	K[1] = K[2] = K[3] = K[5] = K[6] = K[7] = 0.0;
	K[8] = 1.0;
}

void MainFrame::RefineCameraParameters(ECamera *camera, const Vec3Vec &points, const Vec2Vec &projections, int *pt_idxs, std::vector<int> &inliers)
{
	Vec3Vec points_curr(points);
	Vec2Vec projs_curr(projections);

	inliers.resize(points.size());
	std::iota(inliers.begin(), inliers.end(), 0);

	// First refine with the focal length fixed
	RefineCamera(camera, points_curr, projs_curr, false);

	int round = 0;
	while (true)
	{
		wxLogMessage("[RefineCameraParameters] Calling with %d points", points_curr.size());
		RefineCamera(camera, points_curr, projs_curr, true);

		std::vector<double> errors;
		for (int i = 0; i < points_curr.size(); i++)
		{
			Vec2 projection = SfmProjectFinal(points_curr[i], *camera);
			errors.push_back((projection - projs_curr[i]).norm());
		}

		// Sort and histogram errors
		double median		= util::GetNthElement(util::iround(0.95 * points_curr.size()), errors);
		double threshold	= util::clamp(2.4 * median, m_options.min_reprojection_error_threshold, m_options.max_reprojection_error_threshold);
		double avg			= std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
		wxLogMessage("[RefineCameraParameters] Mean error [%d pts]: %.3f [med: %.3f, outlier threshold: = %.3f]", points_curr.size(), avg, median, threshold);

		Vec3Vec points_next;
		Vec2Vec projs_next;
		std::vector<int> inliers_next;

		for (int i = 0; i < points_curr.size(); i++)
		{
			if (errors[i] < threshold)
			{
				inliers_next.push_back(inliers[i]);

				points_next.push_back(points_curr[i]);
				projs_next.push_back(projs_curr[i]);
			} else
			{
				if (pt_idxs != nullptr)
				{
					wxLogMessage("[RefineCameraParameters] Removing point [%d] with reprojection error %0.3f", pt_idxs[i], errors[i]);
				} else
				{
					wxLogMessage("[RefineCameraParameters] Removing point with reprojection error %0.3f", errors[i]);
				}
			}
		}

		if (points_next.size() == points_curr.size()) break;	// We're done

		points_curr	= points_next;
		projs_curr	= projs_next;
		inliers		= inliers_next;

		if (points_curr.size() == 0) break;	// Out of measurements

		round++;
	}

	wxLogMessage("[RefineCameraParameters] Exiting after %d rounds with %d/%d points", round + 1, points_curr.size(), points.size());
}

int MainFrame::BundleAdjustAddAllNewPoints(int num_points, int num_cameras, std::vector<int> &added_order, camera_params_t *cameras,
											v3_t *points, v3_t *colors, std::vector<ImageKeyVector> &pt_views)
{
	std::vector<ImageKeyVector>	new_tracks;
	std::vector<int>			track_idxs;
	std::vector<int>			tracks_seen(m_tracks.size(), -1);

	// Gather up the projections of all the new tracks
	for (int i = 0; i < num_cameras; i++)
	{
		int image_idx1 = added_order[i];
		int num_keys = this->GetNumKeys(image_idx1);

		for (int j = 0; j < num_keys; j++)
		{
			KeyPoint &key = GetKey(image_idx1, j);

			if (key.m_track == -1) continue;	// Key belongs to no track
			if (key.m_extra != -1) continue;	// Key is outlier or has already been added

			int track_idx(key.m_track);

			// Check if this track is already associated with a point
			if (m_tracks[track_idx].m_extra != -1) continue;

			// Check if we've seen this track
			int seen(tracks_seen[track_idx]);

			if (seen == -1)
			{
				// We haven't yet seen this track, create a new track
				tracks_seen[track_idx] = (int) new_tracks.size();

				ImageKeyVector track;
				track.push_back(ImageKey(i, j));
				new_tracks.push_back(track);
				track_idxs.push_back(track_idx);
			} else
			{
				new_tracks[seen].push_back(ImageKey(i, j));
			}
		}
	}

	// Now for each (sub) track, triangulate to see if the track is consistent
	int pt_count = num_points;
	int num_ill_conditioned = 0;
	int num_high_reprojection = 0;
	int num_cheirality_failed = 0;
	int num_added = 0;
	int num_tracks = (int)new_tracks.size();

	for (int i = 0; i < num_tracks; i++)
	{
		int num_views = (int)new_tracks[i].size();
		if (num_views < 2) continue;	// Not enough views

		// Check if at least two cameras fix the position of the point
		bool conditioned(false);
		double max_angle(0.0);

		for (int j = 0; j < num_views; j++)
		{
			for (int k = j + 1; k < num_views; k++)
			{
				int camera_idx1(new_tracks[i][j].first);
				int image_idx1(added_order[camera_idx1]);
				int key_idx1(new_tracks[i][j].second);

				int camera_idx2(new_tracks[i][k].first);
				int image_idx2(added_order[camera_idx2]);
				int key_idx2(new_tracks[i][k].second);

				KeyPoint &key1 = GetKey(image_idx1, key_idx1);
				KeyPoint &key2 = GetKey(image_idx2, key_idx2);

				Vec2 p(key1.m_x, key1.m_y);
				Vec2 q(key2.m_x, key2.m_y);

				ECamera cam1, cam2;
				cam1.m_focal_length = cameras[camera_idx1].f;
				cam1.m_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameras[camera_idx1].R);
				cam1.m_t = Eigen::Map<Vec3>(cameras[camera_idx1].t);
				cam1.m_k = Vec2(cameras[camera_idx1].k[0], cameras[camera_idx1].k[1]);

				cam2.m_focal_length = cameras[camera_idx2].f;
				cam2.m_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameras[camera_idx2].R);
				cam2.m_t = Eigen::Map<Vec3>(cameras[camera_idx2].t);
				cam2.m_k = Vec2(cameras[camera_idx2].k[0], cameras[camera_idx2].k[1]);

				double angle = ComputeRayAngle(p, q, cam1, cam2);

				if (angle > max_angle) max_angle = angle;

				// Check that the angle between the rays is large enough
				if (util::rad2deg(angle) >= m_options.ray_angle_threshold) conditioned = true;
			}
		}
		
		if (!conditioned)
		{
			num_ill_conditioned++;
			continue;
		}
		
		Observations observations;
		for (auto &track : new_tracks[i])
		{
			auto cam = cameras[track.first];

			int image_idx	= added_order[track.first];
			auto &key		= GetKey(image_idx, track.second);

			double K[9];
			this->GetIntrinsics(cam, K);

			Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Ke(K);
			Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(cam.R);
			Eigen::Map<Vec3> t(cam.t);
			Eigen::Map<Vec6> k_inv(cam.k_inv);
		
			Vec3 pn = Ke.inverse() * Vec3(key.m_x, key.m_y, 1.0);
			Vec2 pu = UndistortNormalizedPoint(Vec2(-pn.x(), -pn.y()), k_inv);

			observations.push_back(Observation(pu, R, -(R * t)));
		}

		auto point = Triangulate(observations);

		double error = 0.0;
		for (auto &track : new_tracks[i])
		{
			int image_idx	= added_order[track.first];
			auto &key		= GetKey(image_idx, track.second);

			ECamera cam;
			cam.m_focal_length = cameras[track.first].f;
			cam.m_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameras[track.first].R);
			cam.m_t = Eigen::Map<Vec3>(cameras[track.first].t);
			cam.m_k = Vec2(cameras[track.first].k[0], cameras[track.first].k[1]);
		
			Vec2 pr = SfmProjectFinal(point, cam);

			error += (pr - Vec2(key.m_x, key.m_y)).squaredNorm();
		}
		error = sqrt(error / new_tracks[i].size());

		v3_t pt =  v3_new(point.x(), point.y(), point.z());

		if (_isnan(error) || error > 16.0)
		{
			num_high_reprojection++;
			continue;
		}

		bool all_in_front = true;
		for (int j = 0; j < num_views; j++)
		{
			int camera_idx = new_tracks[i][j].first;

			ECamera cam;
			cam.m_focal_length = cameras[camera_idx].f;
			cam.m_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameras[camera_idx].R);
			cam.m_t = Eigen::Map<Vec3>(cameras[camera_idx].t);
			cam.m_k = Vec2(cameras[camera_idx].k[0], cameras[camera_idx].k[1]);

			bool in_front = CheckCheirality(Vec3(Vx(pt), Vy(pt), Vz(pt)), cam);

			if (!in_front)
			{
				all_in_front = false;
				break;
			}
		}

		if (!all_in_front)
		{
			num_cheirality_failed++;
			continue;
		}
		
		// All tests succeeded, so let's add the point
		points[pt_count] = pt;

		int camera_idx = new_tracks[i][0].first;
		int image_idx = added_order[camera_idx];
		int key_idx = new_tracks[i][0].second;
		auto &key = GetKey(image_idx, key_idx);

		colors[pt_count] = v3_new((double)key.m_r, (double)key.m_g, (double)key.m_b);

		pt_views.push_back(new_tracks[i]);

		// Set the point index on the keys
		for (int j = 0; j < num_views; j++)
		{
			int camera_idx = new_tracks[i][j].first;
			int image_idx = added_order[camera_idx];
			int key_idx = new_tracks[i][j].second;
			GetKey(image_idx, key_idx).m_extra = pt_count;
		}

		int track_idx = track_idxs[i];
		m_tracks[track_idx].m_extra = pt_count;

		pt_count++;
		num_added++;
	}

	wxLogMessage("[AddAllNewPoints] Added %d new points",			num_added);
	wxLogMessage("[AddAllNewPoints] Ill-conditioned tracks: %d",	num_ill_conditioned);
	wxLogMessage("[AddAllNewPoints] Bad reprojections: %d",			num_high_reprojection);
	wxLogMessage("[AddAllNewPoints] Failed cheirality checks: %d",	num_cheirality_failed);

	return pt_count;
}

int MainFrame::RemoveBadPointsAndCameras(int num_points, int num_cameras, const std::vector<int> &added_order,
									camera_params_t *cameras, v3_t *points, v3_t *colors, std::vector<ImageKeyVector> &pt_views)
{
	int num_pruned = 0;

	for (int i = 0; i < num_points; i++)
	{
		Vec3 pos(points[i].p[0], points[i].p[1], points[i].p[2]);
		int num_views = (int)pt_views[i].size();

		if (num_views == 0) continue;

		double max_angle = 0.0;
		for (int j = 0; j < num_views; j++)
		{
			int v1(pt_views[i][j].first);

			Eigen::Map<Vec3> t1(cameras[v1].t);
			Vec3 re1 = pos - t1;
			re1 *= 1.0 / re1.norm();

			for (int k = j+1; k < num_views; k++)
			{
				int v2(pt_views[i][k].first);

				Eigen::Map<Vec3> t2(cameras[v2].t);
				Vec3 re2 = pos - t2;
				re2 *= 1.0 / re2.norm();

				double angle = acos(util::clamp(re1.dot(re2), (-1.0 + 1.0e-8), (1.0 - 1.0e-8)));

				if (angle > max_angle) max_angle = angle;
			}
		}

		if (util::rad2deg(max_angle) < 0.5 * m_options.ray_angle_threshold)
		{
			wxLogMessage("[RemoveBadPointsAndCamera] Removing point %d with angle %0.3f", i, util::rad2deg(max_angle));

			for (int j = 0; j < num_views; j++)
			{
				// Set extra flag back to 0
				int v(pt_views[i][j].first);
				int k(pt_views[i][j].second);
				GetKey(added_order[v], k).m_extra = -1;
			}

			pt_views[i].clear();

			Vx(colors[i]) = 0x0;
			Vy(colors[i]) = 0x0;
			Vz(colors[i]) = 0xff;

			num_pruned++;
		}
	}

	wxLogMessage("[RemoveBadPointsAndCameras] Pruned %d points", num_pruned);

	return num_pruned;
}
