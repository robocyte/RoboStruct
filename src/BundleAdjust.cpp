#include <fstream>
#include <numeric>

#include "EigenTools.hpp"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "FivePoint.hpp"
#include "MainFrame.hpp"
#include "Projection.hpp"
#include "Triangulation.hpp"
#include "Utilities.hpp"

namespace
{

// Templated pinhole camera model for use with Ceres. The camera is
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

	int num_images	= GetNumImages();
	IntVec		added_order(num_images);
	CamVec      cameras(num_images);
	PointVec    points;

	auto initial_pair = PickInitialCameraPair();
    added_order[0] = initial_pair.first;
    added_order[1] = initial_pair.second;

	wxLogMessage("[BundleAdjust] Adjusting cameras %d and %d...", initial_pair.first, initial_pair.second);
	SetupInitialCameraPair(initial_pair, cameras, points);
	RunSFM(2, cameras, added_order, points);
	int curr_num_cameras = 2;

	// Main loop
	int round = 0;
	while (curr_num_cameras < num_images)
	{
		int max_matches = 0;
		int max_cam = FindCameraWithMostMatches(curr_num_cameras, added_order, max_matches, points);
		wxLogMessage("[BundleAdjust] Max_matches = %d", max_matches);

		if (max_matches < m_options.min_max_matches)
		{
			wxLogMessage("[BundleAdjust] No more connections!");
			break;
		}

		auto image_set = FindCamerasWithNMatches(util::iround(0.75 * max_matches), curr_num_cameras, added_order, points);
		wxLogMessage("[BundleAdjust] Registering %d images", (int)image_set.size());

		// Now, throw the new cameras into the mix
		int image_count = 0;
		for (const int &image_idx : image_set)
		{
			wxLogMessage("[BundleAdjust[round %i]] Adjusting camera %d", round, image_idx);
			added_order[curr_num_cameras + image_count] = image_idx;

			bool success = false;
			auto camera_new = BundleInitializeImage(image_idx, curr_num_cameras + image_count, points, success);
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
		BundleAdjustAddAllNewPoints(curr_num_cameras, added_order, cameras, points);
        wxLogMessage("[BundleAdjust] Number of points = %d", points.size());

		RunSFM(curr_num_cameras, cameras, added_order, points);

		RemoveBadPointsAndCameras(added_order, cameras, points);

		wxLogMessage("[BundleAdjust] Focal lengths:");
		for (int i = 0; i < curr_num_cameras; i++)
		{
            wxLogMessage("  [%03d] %.3f %s %d; %.3f %.3f", i, cameras[i].m_focal_length, m_images[added_order[i]].m_filename_short.c_str(), added_order[i], cameras[i].m_k[0], cameras[i].m_k[1]);
		}

		// Update points and cameras for display
		{
			wxCriticalSectionLocker lock(m_points_cs);
			m_points.clear();

			for (const auto &point : points)
			{
				// Check if the point is visible in any view
                if (point.m_views.empty()) continue; // Invisible
	
                ImageKeyVector views;
                for (const auto &view : point.m_views) views.push_back(ImageKey(added_order[view.first], view.second));

                m_points.push_back(PointData(point.m_pos, point.m_color, views));
			}

			for (int i = 0; i < curr_num_cameras; i++)
			{
				m_images[added_order[i]].m_camera = cameras[i];
			}
		}

		wxQueueEvent(this, new wxThreadEvent(wxEVT_THREAD_UPDATE));
		round++;
	}

	clock_t end = clock();
	wxLogMessage("[BundleAdjust] Computing structure from motion took %0.3f s", (double) (end - start) / (double) CLOCKS_PER_SEC);

	SavePlyFile();
	SetMatchesFromPoints();

	// Update program state
	m_sfm_done = true;
}

IntPair MainFrame::PickInitialCameraPair()
{
	int		num_images		= GetNumImages();
	int		min_matches		= 80;
	int		max_matches		= 0;
	double	min_score		= 0.1;
	double	max_score		= 0.0;
	double	max_score_2		= 0.0;
	double	score_threshold	= 2.0;
	int		match_threshold = 32;

    int     i_best = -1, j_best = -1;
	int		i_best_2 = -1, j_best_2 = -1;

	// Compute score for each image pair
	int max_pts = 0;
	for (int i = 0; i < num_images; i++)
	{
		for (int j = i + 1; j < num_images; j++)
		{
			int num_matches = GetNumTrackMatches(i, j);
			max_pts += num_matches;

			if (num_matches <= match_threshold) continue;

			double score = 0.0;
			double ratio = GetInlierRatio(i, j);
			
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

    return std::make_pair(i_best, j_best);
}

void MainFrame::SetupInitialCameraPair(IntPair initial_pair, CamVec &cameras, PointVec &points)
{
    int i_best = initial_pair.first;
    int j_best = initial_pair.second;
	SetMatchesFromTracks(i_best, j_best);

	m_images[i_best].SetTracks();
	m_images[j_best].SetTracks();

    cameras[0].m_focal_length = cameras[0].m_init_focal_length = m_images[i_best].m_init_focal;
	cameras[1].m_focal_length = cameras[1].m_init_focal_length = m_images[j_best].m_init_focal;

	// Solve for initial locations
	EstimateRelativePose(i_best, j_best, &cameras[0], &cameras[1]);

	// Triangulate the initial 3D points
	wxLogMessage("[SetupInitialCameraPair] Adding initial matches...");

    Mat3 K1_inv = cameras[0].GetIntrinsicMatrix().inverse();
    Mat3 K2_inv = cameras[1].GetIntrinsicMatrix().inverse();

	auto &list = m_matches.GetMatchList(GetMatchIndex(i_best, j_best));
	int num_matches = list.size();

	for (int i = 0; i < num_matches; i++)
	{
		// Set up the 3D point
		int key_idx1(list[i].m_idx1);
		int key_idx2(list[i].m_idx2);

		// Normalize the point
		Vec3 p_norm1 = K1_inv * Vec3(m_images[i_best].m_keys[key_idx1].m_coords.x(), m_images[i_best].m_keys[key_idx1].m_coords.y(), -1.0);
		Vec3 p_norm2 = K2_inv * Vec3(m_images[j_best].m_keys[key_idx2].m_coords.x(), m_images[j_best].m_keys[key_idx2].m_coords.y(), -1.0);

		// Put the translation in standard form
		Observations observations;
        observations.push_back(Observation(Vec2(p_norm1.head<2>() / p_norm1.z()), cameras[0]));
        observations.push_back(Observation(Vec2(p_norm2.head<2>() / p_norm2.z()), cameras[1]));

		double error;
		auto position = Triangulate(observations, &error);

        error = (cameras[0].m_focal_length + cameras[1].m_focal_length) * 0.5 * sqrt(error * 0.5);
		
		if (error > m_options.projection_estimation_threshold) continue;

		auto &key = GetKey(i_best, key_idx1);

        GetKey(i_best, key_idx1).m_extra = points.size();
		GetKey(j_best, key_idx2).m_extra = points.size();

		int track_idx = GetKey(i_best, key_idx1).m_track;
		m_tracks[track_idx].m_extra = points.size();

		ImageKeyVector views;
		views.push_back(ImageKey(0, key_idx1));
		views.push_back(ImageKey(1, key_idx2));
 
        points.push_back(PointData(position, key.m_color, views));
	}
}

bool MainFrame::EstimateRelativePose(int i1, int i2, Camera *camera1, Camera *camera2)
{
	auto &matches = m_matches.GetMatchList(GetMatchIndex(i1, i2));

    Vec2Vec projections1; projections1.reserve(matches.size());
	Vec2Vec projections2; projections2.reserve(matches.size());

	for (const auto &match : matches)
	{
		projections1.push_back(m_images[i1].m_keys[match.m_idx1].m_coords);
		projections2.push_back(m_images[i2].m_keys[match.m_idx2].m_coords);
	}

	wxLogMessage("[EstimateRelativePose] EstimateRelativePose starting...");
	clock_t start = clock();
	Mat3 R; Vec3 t;
    int num_inliers = ComputeRelativePoseRansac(projections1, projections2, camera1->GetIntrinsicMatrix(), camera2->GetIntrinsicMatrix(), m_options.ransac_threshold_five_point, m_options.ransac_rounds_five_point, &R, &t);
	clock_t end = clock();
	wxLogMessage("[EstimateRelativePose] EstimateRelativePose took %0.3f s", (double) (end - start) / (double) CLOCKS_PER_SEC);
	wxLogMessage("[EstimateRelativePose] Found %d / %d inliers (%0.3f%%)", num_inliers, matches.size(), 100.0 * num_inliers / matches.size());

	if (num_inliers == 0) return false;

	camera1->m_adjusted = true;
	camera2->m_adjusted = true;

    camera2->m_R = R;
    camera2->m_t = -(R.transpose() * t);

	return true;
}

void MainFrame::SetMatchesFromTracks(int img1, int img2)
{
	auto &matches = m_matches.GetMatchList(GetMatchIndex(img1, img2));
	auto &tracks1 = m_images[img1].m_visible_points;
	auto &tracks2 = m_images[img2].m_visible_points;

	// Find tracks visible from both cameras
    IntVec intersection;
	std::set_intersection(tracks1.begin(), tracks1.end(), tracks2.begin(), tracks2.end(), std::inserter(intersection, intersection.begin()));

	if (intersection.empty()) return;

	matches.clear();
	matches.reserve(intersection.size());

	for (const int &track : intersection)
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

	for (const auto &point : m_points)
    {
		int num_views = point.m_views.size();

		for (int j = 0; j < num_views; j++)
        {
			for (int k = 0; k < num_views; k++)
            {
				if (j == k) continue;

				ImageKey view1 = point.m_views[j];
				ImageKey view2 = point.m_views[k];

				SetMatch(view1.first, view2.first);
				m_matches.AddMatch(MatchIndex(view1.first, view2.first), KeypointMatch(view1.second, view2.second));
			}
		}
	}

	wxLogMessage("[SetMatchesFromPoints] Done!");
}

int MainFrame::FindCameraWithMostMatches(int num_cameras, const IntVec &added_order, int &max_matches, const PointVec &points)
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
		for (const auto & track : m_images[i].m_visible_points)
		{
			if (m_tracks[track].m_extra < 0) continue;

			// This tracks corresponds to a point
			int pt = m_tracks[track].m_extra;
            if (points[pt].m_views.empty()) continue;

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

IntVec MainFrame::FindCamerasWithNMatches(int n, int num_cameras, const IntVec &added_order, const PointVec &points)
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
		for (const auto & track : m_images[i].m_visible_points)
		{
			if (m_tracks[track].m_extra < 0) continue;

			// This tracks corresponds to a point
			int pt = m_tracks[track].m_extra;
            if (points[pt].m_views.empty()) continue;

			num_existing_matches++;
		}

		if (num_existing_matches >= n) found_cameras.push_back(i);
	}

	wxLogMessage("[FindCamerasWithNMatches] Found %i cameras with at least %i matches", found_cameras.size(), n);

	return found_cameras;
}

Camera MainFrame::BundleInitializeImage(int image_idx, int camera_idx, PointVec &points, bool &success_out)
{
	clock_t start = clock();
	Camera dummy;

	success_out = false;

	auto &image = m_images[image_idx];
	image.SetTracks();

	// **** Connect the new camera to any existing points ****
	Vec3Vec	points_solve;
	Vec2Vec	projs_solve;
	IntVec	idxs_solve;
	IntVec	keys_solve;

	wxLogMessage("[BundleInitializeImage] Connecting existing matches...");

	// Find the tracks seen by this image
	for (int i = 0; i < image.m_visible_points.size(); i++)
	{
		int track	= image.m_visible_points[i];
		int key		= image.m_visible_keys[i];

		if (m_tracks[track].m_extra < 0) continue;

		// This tracks corresponds to a point
		int pt(m_tracks[track].m_extra);
        if (points[pt].m_views.empty()) continue;

		// Add the point to the set we'll use to solve for the camera position
        points_solve.push_back(points[pt].m_pos);
		projs_solve.push_back(image.m_keys[key].m_coords);
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

	Mat3 Kinit, Rinit;
    Vec3 tinit;
	Camera camera_new;
	IntVec inliers, inliers_weak, outliers;
	bool found = FindAndVerifyCamera(points_solve, projs_solve, idxs_solve.data(), &Kinit, &Rinit, &tinit, inliers, inliers_weak, outliers);

	if (!found)
	{
		wxLogMessage("[BundleInitializeImage] Couldn't initialize (couldn't solve for the camera position)");
		return dummy;
	} else
	{
		// Set up the new camera
		camera_new.m_R = Rinit;
        camera_new.m_t = -(Rinit.transpose() * tinit);

		// Set up the new focal length
        camera_new.m_focal_length = camera_new.m_init_focal_length = image.m_init_focal;
        wxLogMessage("[BundleInitializeImage] Camera has initial focal length of %0.3f", camera_new.m_init_focal_length);
	}

	// **** Finally, start the bundle adjustment ****
	wxLogMessage("[BundleInitializeImage] Adjusting...");
	Vec3Vec	points_final;
	Vec2Vec	projs_final;
	IntVec	idxs_final;
	IntVec	keys_final;

	for (const auto &idx : inliers_weak)
	{
		points_final.push_back(points_solve[idx]);
		projs_final.push_back(projs_solve[idx]);
		idxs_final.push_back(idxs_solve[idx]);
		keys_final.push_back(keys_solve[idx]);
	}

	RefineCameraParameters(&camera_new, points_final, projs_final, idxs_final.data(), inliers);

    if ((inliers.size() < 8) || (camera_new.m_focal_length < 0.1 * image.GetWidth()))
	{
		wxLogMessage("[BundleInitializeImage] Bad camera");
		return dummy;
	}

	// Point the keys to their corresponding points
	for (const auto &i : inliers)
	{
		image.m_keys[keys_final[i]].m_extra = idxs_final[i];
        points[idxs_final[i]].m_views.push_back(ImageKey(camera_idx, keys_final[i]));
	}

	clock_t end = clock();

	wxLogMessage("[BundleInitializeImage] Initializing took %0.3f s", (double) (end - start) / CLOCKS_PER_SEC);

	camera_new.m_adjusted = true;

	success_out = true;
	return camera_new;
}

bool MainFrame::FindAndVerifyCamera(const Vec3Vec &points, const Vec2Vec &projections, int *idxs_solve, Mat3 *K, Mat3 *R, Vec3 *t, IntVec &inliers, IntVec &inliers_weak, IntVec &outliers)
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

	DecomposeProjectionMatrix(P, K, R, t);

	wxLogMessage("[FindAndVerifyCamera] Checking consistency...");

	Mat34 Rigid;
	Rigid << *R;
	Rigid.col(3) = *t;

	int num_behind = 0;
	for (int j = 0; j < points.size(); j++)
	{
		Vec3 q = *K * (Rigid * EuclideanToHomogenous(points[j]));
		Vec2 pimg = -q.head<2>() / q.z();
		double diff = (pimg - projections[j]).norm();

		if (diff < m_options.projection_estimation_threshold) inliers.push_back(j);
		if (diff < (16.0 * m_options.projection_estimation_threshold))
		{
			inliers_weak.push_back(j);
		} else
		{
			//wxLogMessage("[FindAndVerifyCamera] Removing point [%d] with reprojection error %0.3f", idxs_solve[j], diff);
			outliers.push_back(j);
		}

		if (q.z() > 0.0) num_behind++;	// Cheirality constraint violated
	}

    //TODO: use inliers instead of inliers_weak?
    //wxLogMessage("     weak inliers: %d", inliers_weak.size());
    //wxLogMessage("     inliers:      %d", inliers.size());
    //wxLogMessage("     outliers:      %d", outliers.size());

    if (num_behind >= 0.9 * points.size())
	{
		wxLogMessage("[FindAndVerifyCamera] Error: camera is pointing away from scene");
		return false;
	}

	return true;
}

void MainFrame::RefineCameraParameters(Camera *camera, const Vec3Vec &points, const Vec2Vec &projections, int *pt_idxs, IntVec &inliers)
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
            Vec2 reprojection = camera->ProjectFinal(points_curr[i]);
			errors.push_back((reprojection - projs_curr[i]).norm());
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
					//wxLogMessage("[RefineCameraParameters] Removing point [%d] with reprojection error %0.3f", pt_idxs[i], errors[i]);
				} else
				{
					//wxLogMessage("[RefineCameraParameters] Removing point with reprojection error %0.3f", errors[i]);
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

void MainFrame::RunSFM(int num_cameras, CamVec &cameras, const IntVec &added_order, PointVec &points)
{
	const int	min_points		= 20;
	int			round			= 0;
	int			num_outliers	= 0;
	int			total_outliers	= 0;

    int num_pts = points.size();
    std::vector<int>	remap(num_pts);
    Vec3Vec         	nz_pts(num_pts);

	clock_t start_all, stop_all;
	start_all = clock();

	do
	{
		clock_t round_start, round_stop;
		round_start = clock();

		if (num_pts - total_outliers < min_points)
		{
			wxLogMessage("[RunSFM] Too few points remaining, exiting!");
			break;
		}

		// Set up the projections
		int num_projections = 0;
        for (const auto &point : points) num_projections += point.m_views.size();
		std::vector<double>			projections(2 * num_projections);
		std::vector<int>			pidx(num_projections);
		std::vector<int>			cidx(num_projections);
		std::vector<unsigned int>	num_vis(num_cameras, 0);

		int arr_idx = 0;
		int nz_count = 0;
		for (int i = 0; i < num_pts; i++)
		{
            if (!points[i].m_views.empty())
			{
                for (const auto &view : points[i].m_views)
				{
                    int c = view.first;
                    int k = view.second;

					projections[2 * arr_idx + 0] = GetKey(added_order[c], k).m_coords.x();
					projections[2 * arr_idx + 1] = GetKey(added_order[c], k).m_coords.y();

					pidx[arr_idx] = nz_count;
					cidx[arr_idx] = c;

					num_vis[c]++;

					arr_idx++;
				}

				remap[i] = nz_count;
                nz_pts[nz_count] = points[i].m_pos;
				nz_count++;
			} else
			{
				remap[i] = -1;
			}
		}
        
    	ceres::Problem problem;
		ceres::Solver::Options options;
		ceres::Solver::Summary summary;
		options.linear_solver_type = ceres::DENSE_SCHUR;

		// Set up initial parameters
		int num_nz_points = nz_count;

		int cnp = 9;
		int pnp = 3;

		unsigned int num_parameters = pnp * num_nz_points + cnp * num_cameras;

		double *init_x = new double[num_parameters];
		double *pcameras = init_x;
		double *ppoints = init_x + cnp * num_cameras;

		// Insert camera parameters
		double *ptr = init_x;

		int idx = 0;
		for (int i = 0; i < num_cameras; i++)
		{
			// Get the rotation
            Vec3 axis = RotationMatrixToAngleAxis(cameras[i].m_R);
			Vec3 t = -(cameras[i].m_R * cameras[i].m_t);

            double f = cameras[i].m_focal_length;
            double k1 = cameras[i].m_k[0];
			double k2 = cameras[i].m_k[1];

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
            if (!points[i].m_views.empty())
			{
                ptr[idx] = points[i].m_pos.x(); idx++;
                ptr[idx] = points[i].m_pos.y(); idx++;
                ptr[idx] = points[i].m_pos.z(); idx++;
			}
		}

		for (int i = 0; i < num_projections; ++i)
		{
			// Each Residual block takes a point and a camera as input and outputs a 2 dimensional residual
			ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(projections[2 * i + 0], projections[2 * i + 1]);

			ceres::LossFunction *loss_function = nullptr;
            switch (m_options.selected_loss)
            {
            case Options::loss_function::squared:   break;   
            case Options::loss_function::huber:     loss_function = new ceres::HuberLoss(m_options.loss_function_scale);    break;
            case Options::loss_function::softlone:  loss_function = new ceres::SoftLOneLoss(m_options.loss_function_scale); break;
            case Options::loss_function::cauchy:    loss_function = new ceres::CauchyLoss(m_options.loss_function_scale);   break;
            case Options::loss_function::arctan:    loss_function = new ceres::ArctanLoss(m_options.loss_function_scale);   break;
            default: break;
            }

			// Each observation correponds to a pair of a camera and a point which are identified by camera_index[i] and point_index[i] respectively
			double *camera	= pcameras	+ cnp * cidx[i];
			double *point	= ppoints	+ pnp * pidx[i];

			problem.AddResidualBlock(cost_function, loss_function, camera, point);
		}

		// Now add the priors
		for (int i = 0; i < num_cameras; i++)
		{
			ceres::CostFunction *prior_cost_function;

            prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>(new PriorError(6, cameras[i].m_init_focal_length, m_options.focal_length_constrain_weight * num_vis[i]));
			problem.AddResidualBlock(prior_cost_function, nullptr, pcameras + cnp * i);

			// Take care of priors on distortion parameters
			prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>(new PriorError(7, 0.0, m_options.distortion_constrain_weight * num_vis[i]));
			problem.AddResidualBlock(prior_cost_function, nullptr, pcameras + cnp * i);

			prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>(new PriorError(8, 0.0, m_options.distortion_constrain_weight * num_vis[i]));
			problem.AddResidualBlock(prior_cost_function, nullptr, pcameras + cnp * i);
		}

		clock_t start = clock();

		// Make call to Ceres
		wxLogMessage("[Ceres] %d points",		num_nz_points);
		wxLogMessage("[Ceres] %d cameras",		num_cameras);
		wxLogMessage("[Ceres] %d params",		num_parameters);
		wxLogMessage("[Ceres] %d projections",	num_projections);
		ceres::Solve(options, &problem, &summary);
		wxLogMessage("[Ceres] %s", summary.BriefReport().c_str());

		ptr = init_x;
		for (int i = 0; i < num_cameras; i++)
		{
			// Get the camera parameters
			Vec3 axis, t;

			axis[0] = *ptr; ptr++;
			axis[1] = *ptr; ptr++;
			axis[2] = *ptr; ptr++;
			t[0]    = *ptr; ptr++;
			t[1]    = *ptr; ptr++;
			t[2]    = *ptr; ptr++;

            cameras[i].m_R = AngleAxisToRotationMatrix(axis);
            cameras[i].m_t = -(cameras[i].m_R.transpose() * t);
            double f = cameras[i].m_focal_length = *ptr; ptr++;
			cameras[i].m_k[0] = *ptr * (f * f); ptr++;
			cameras[i].m_k[1] = *ptr * (f * f * f * f); ptr++;
		}

		// Insert point parameters
		for (int i = 0; i < num_nz_points; i++)
		{
			nz_pts[i].x() = *ptr; ptr++;
			nz_pts[i].y() = *ptr; ptr++;
			nz_pts[i].z() = *ptr; ptr++;
		}

		clock_t end = clock();

		wxLogMessage("[RunSFM] RunSFM took %0.3fs", (double) (end - start) / (double) CLOCKS_PER_SEC);

		// Check for outliers
		start = clock();

		std::vector<int>	outliers;
		std::vector<double>	reproj_errors;

        for (int i = 0; i < num_cameras; i++)
		{
			auto &image = m_images[added_order[i]];

			// Compute inverse distortion parameters
			Vec6 k_dist; k_dist << 0.0, 1.0, 0.0, cameras[i].m_k[0], 0.0, cameras[i].m_k[1];
			double w_2 = 0.5 * image.GetWidth();
			double h_2 = 0.5 * image.GetHeight();
            double max_radius = sqrt(w_2 * w_2 + h_2 * h_2) / cameras[i].m_focal_length;
			cameras[i].m_k_inv = InvertDistortion(0.0, max_radius, k_dist);

			std::vector<double> dists;

			for (const auto &key : image.m_keys)
			{
				if (key.m_extra >= 0)
				{
					int pt_idx = key.m_extra;
                    Vec2 reprojection = cameras[i].ProjectRD(nz_pts[remap[pt_idx]]);

					dists.push_back((reprojection - key.m_coords).norm());
				}
			}

			// Estimate the median of the distances and compute the average reprojection error for this camera
			double median	= util::GetNthElement(util::iround(0.8 * dists.size()), dists);
			double thresh	= util::clamp(2.4 * median, m_options.min_reprojection_error_threshold, m_options.max_reprojection_error_threshold);
			double avg		= std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
            wxLogMessage("[RunSFM] Mean error cam %d[%d] [%d pts]: %.3f [med: %.3f, outlier threshold: %.3f]", i, added_order[i], dists.size(), avg, median, thresh);

			int pt_count = 0;
			for (const auto &key : image.m_keys)
			{
				int pt_idx = key.m_extra;
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

			//wxLogMessage("[RunSFM] Removing outlier %d (reprojection error: %0.3f)", idx, reproj_errors[i]);

            points[idx].m_color = Vec3(0.0, 0.0, -1.0);

            for (const auto &view : points[idx].m_views)
			{
                int v = view.first;
                int k = view.second;

				// Sanity check
				if (GetKey(added_order[v], k).m_extra != idx) wxLogMessage("Error! Entry for (%d,%d) should be %d, but is %d", added_order[v], k, idx, GetKey(added_order[v], k).m_extra);

				GetKey(added_order[v], k).m_extra = -2;
			}

            points[idx].m_views.clear();
		}

		num_outliers = outliers.size();
		total_outliers += num_outliers;

		end = clock();
		wxLogMessage("[RunSFM] Removed %d outliers in %0.3f s", num_outliers, (double) (end - start) / (double) CLOCKS_PER_SEC);

		RemoveBadPointsAndCameras(added_order, cameras, points);

        for (int i = 0; i < num_pts; i++) if (remap[i] != -1) points[i].m_pos = nz_pts[remap[i]];

		round_stop = clock();
		wxLogMessage("[RunSFM] Round %d took %0.3f s", round, (double) (round_stop - round_start) / (double) CLOCKS_PER_SEC);

		// Update points for display
		{
			wxCriticalSectionLocker lock(m_points_cs);
			m_points.clear();

			for (const auto &point : points)
			{
				// Check if the point is visible in any view
                if (point.m_views.empty()) continue; // Invisible
	
                ImageKeyVector views;
                for (const auto &view : point.m_views) views.push_back(ImageKey(added_order[view.first], view.second));

                m_points.push_back(PointData(point.m_pos, point.m_color, views));
			}

            for (int i = 0; i < num_cameras; i++)
			{
				m_images[added_order[i]].m_camera = cameras[i];
			}
		
        }
        
		wxQueueEvent(this, new wxThreadEvent(wxEVT_THREAD_UPDATE));

		round++;
        delete[] init_x;
	} while (num_outliers > m_options.outlier_threshold_ba);

	stop_all = clock();
	wxLogMessage("[RunSFM] Structure from motion with outlier removal took %0.3f s (%d rounds)", (double) (stop_all - start_all) / (double) CLOCKS_PER_SEC, round);
}

void MainFrame::BundleAdjustAddAllNewPoints(int num_cameras, IntVec &added_order, CamVec &cameras, PointVec &points)
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

                double angle = ComputeRayAngle(key1.m_coords, key2.m_coords, cameras[camera_idx1], cameras[camera_idx2]);

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

            Mat3 K = cam.GetIntrinsicMatrix();

			Vec3 pn = K.inverse() * EuclideanToHomogenous(key.m_coords);
            Vec2 pu = UndistortNormalizedPoint(Vec2(-pn.x(), -pn.y()), cam.m_k_inv);

			observations.push_back(Observation(pu, cam));
		}

		auto point = Triangulate(observations);

		double error = 0.0;
		for (const auto &track : new_tracks[i])
		{
			int image_idx	= added_order[track.first];
			auto &key		= GetKey(image_idx, track.second);

            Vec2 reprojection = cameras[track.first].ProjectFinal(point);

			error += (reprojection - key.m_coords).squaredNorm();
		}
		error = sqrt(error / new_tracks[i].size());

		if (_isnan(error) || error > 16.0)
		{
			num_high_reprojection++;
			continue;
		}

		bool all_in_front = true;
        for (const auto &track : new_tracks[i])
		{
            bool in_front = CheckCheirality(point, cameras[track.first]);

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
		auto &key = GetKey(added_order[new_tracks[i][0].first], new_tracks[i][0].second);

        points.push_back(PointData(point, key.m_color, new_tracks[i]));

		// Set the point index on the keys
        for (const auto &track : new_tracks[i]) GetKey(added_order[track.first], track.second).m_extra = points.size() - 1;

		m_tracks[track_idxs[i]].m_extra = points.size() - 1;

		num_added++;
	}

	wxLogMessage("[AddAllNewPoints] Added %d new points",			num_added);
	wxLogMessage("[AddAllNewPoints] Ill-conditioned tracks: %d",	num_ill_conditioned);
	wxLogMessage("[AddAllNewPoints] Bad reprojections: %d",			num_high_reprojection);
	wxLogMessage("[AddAllNewPoints] Failed cheirality checks: %d",	num_cheirality_failed);
}

int MainFrame::RemoveBadPointsAndCameras(const IntVec &added_order, CamVec &cameras, PointVec &points)
{
	int num_pruned = 0;

	for (auto &point : points)
	{
        if (point.m_views.empty()) continue;

		double max_angle = 0.0;
        for (int j = 0; j < point.m_views.size(); j++)
		{
            int v1(point.m_views[j].first);

            Vec3 re1 = point.m_pos - cameras[v1].m_t;
			re1 *= 1.0 / re1.norm();

			for (int k = j + 1; k < point.m_views.size(); k++)
			{
                int v2(point.m_views[k].first);

                Vec3 re2 = point.m_pos - cameras[v2].m_t;
				re2 *= 1.0 / re2.norm();

				double angle = acos(util::clamp(re1.dot(re2), (-1.0 + 1.0e-8), (1.0 - 1.0e-8)));

				if (angle > max_angle) max_angle = angle;
			}
		}

		if (util::rad2deg(max_angle) < 0.5 * m_options.ray_angle_threshold)
		{
			//wxLogMessage("[RemoveBadPointsAndCamera] Removing point with angle %0.3f", util::rad2deg(max_angle));

			for (const auto &view : point.m_views)
			{
				// Set extra flag back to 0
                GetKey(added_order[view.first], view.second).m_extra = -1;
			}

			point.m_color = Vec3(0.0, 0.0, -1.0);
            point.m_views.clear();

			num_pruned++;
		}
	}

	wxLogMessage("[RemoveBadPointsAndCameras] Pruned %d points", num_pruned);

	return num_pruned;
}
