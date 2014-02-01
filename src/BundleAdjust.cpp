#include <numeric>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "snavely_reprojection_error.h"

#include "flann.hpp"

#include "FivePoint.hpp"
#include "MainFrame.hpp"
#include "Projection.hpp"
#include "Triangulation.hpp"
#include "Utilities.hpp"

namespace
{

    using namespace ceres::examples;

    // Factory to hide the construction of the CostFunction object from the client code
    ceres::CostFunction* CreateBundlerCostFunction(const double observed_x, const double observed_y)
    {
        return new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>{new SnavelyReprojectionError{observed_x, observed_y}};
    }

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
    RunSFM();

    wxQueueEvent(this, new wxThreadEvent{wxEVT_SFM_THREAD_COMPLETE});

    return (wxThread::ExitCode)0;
}

void MainFrame::UpdateGeometryDisplay(const CamVec& cameras, const IntVec& added_order, const PointVec& points)
{
    wxCriticalSectionLocker lock{m_points_cs};
    m_points.clear();

    for (const auto& point : points)
    {
        if (point.m_views.empty()) continue; // Invisible

        ImageKeyVector views;
        for (const auto& view : point.m_views) views.push_back(ImageKey{added_order[view.first], view.second});

        m_points.push_back(PointData{point.m_pos, point.m_color, views});
    }

    for (int i = 0; i < cameras.size(); i++) m_images[added_order[i]].m_camera = cameras[i];
        
    wxQueueEvent(this, new wxThreadEvent{wxEVT_SFM_THREAD_UPDATE});
}

void MainFrame::RunSFM()
{
    wxLogMessage("[RunSFM] Computing structure from motion...");

    // Reset track to key mappings and vice versa
    for (auto& track : m_tracks) track.m_extra = -1;
    for (auto& img : m_images) for (auto& key : img.m_keys) key.m_extra = -1;

    CamVec   cameras;
    IntVec   added_order;
    PointVec points;

    PickInitialCameraPair(cameras, added_order);
    SetupInitialCameraPair(cameras, added_order, points);
    BundleAdjust(cameras, added_order, points);

    int round = 0;
    while (cameras.size() < m_images.size())
    {
        int max_matches = 0;
        int max_cam = FindCameraWithMostMatches(added_order, max_matches, points);

        if (max_matches < m_options.min_max_matches)
        {
            wxLogMessage("[RunSFM] No more connections!");
            break;
        }

        IntVec next_images;
        if (m_options.add_multiple_images)  next_images = FindCamerasWithNMatches(util::iround(0.75 * max_matches), added_order, points);
        else                                next_images.push_back(max_cam);
 
        for (const int& next : next_images)
        {
            added_order.push_back(next);

            auto camera_new = InitializeImage(next, cameras.size(), points);
            if (camera_new.m_adjusted)  cameras.push_back(camera_new);
            else                        m_images[next].m_ignore_in_bundle = true;
        }

        AddNewPoints(cameras, added_order, points);
        BundleAdjust(cameras, added_order, points);

        round++;
    }

    RadiusOutlierRemoval(m_options.outlier_threshold_radius, added_order, points);
    BundleAdjust(cameras, added_order, points);

    SetMatchesFromPoints();
    SavePlyFile();

    wxLogMessage("%s", m_profile_manager.Report().c_str());
}

void MainFrame::PickInitialCameraPair(CamVec& cameras, IntVec& added_order)
{
    int     num_images      = GetNumImages();
    int     min_matches     = 80;
    int     max_matches     = 0;
    double  min_score       = 0.1;
    double  max_score       = 0.0;
    double  max_score_2     = 0.0;
    double  score_threshold = 2.0;
    int     match_threshold = 32;

    int     i_best = -1, j_best = -1;
    int     i_best_2 = -1, j_best_2 = -1;

    // Compute score for each image pair
    for (int i = 0; i < num_images; i++)
    {
        for (int j = i + 1; j < num_images; j++)
        {
            int num_matches = GetNumTrackMatches(i, j);

            if (num_matches <= match_threshold) continue;

            double score = 0.0;
            double ratio = GetInlierRatio(i, j);

            if (ratio == 0.0)   score = min_score;
            else                score = 1.0 / ratio;

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

    added_order.push_back(i_best);
    added_order.push_back(j_best);
    cameras.push_back(Camera{});
    cameras.push_back(Camera{});
}

void MainFrame::SetupInitialCameraPair(CamVec& cameras, const IntVec& added_order, PointVec& points)
{
    wxLogMessage("[SetupInitialCameraPair] Adjusting cameras %d and %d...", added_order[0], added_order[1]);

    int img_1 = added_order[0];
    int img_2 = added_order[1];
    SetMatchesFromTracks(img_1, img_2);

    m_images[img_1].SetTracks();
    m_images[img_2].SetTracks();

    cameras[0].m_focal_length = cameras[0].m_init_focal_length = m_images[img_1].m_init_focal;
    cameras[1].m_focal_length = cameras[1].m_init_focal_length = m_images[img_2].m_init_focal;

    // Solve for initial locations
    EstimateRelativePose(img_1, img_2, &cameras[0], &cameras[1]);

    // Triangulate the initial 3D points
    wxLogMessage("[SetupInitialCameraPair] Triangulating initial matches...");

    Mat3 K1_inv = cameras[0].GetIntrinsicMatrix().inverse();
    Mat3 K2_inv = cameras[1].GetIntrinsicMatrix().inverse();

    auto& list = m_matches.GetMatchList(GetMatchIndex(img_1, img_2));
    int num_matches = list.size();

    for (int i = 0; i < num_matches; i++)
    {
        // Set up the 3D point
        int key_idx1(list[i].first);
        int key_idx2(list[i].second);

        // Normalize the point
        Point3 p_norm1 = K1_inv * Point3{m_images[img_1].m_keys[key_idx1].m_coords.x(), m_images[img_1].m_keys[key_idx1].m_coords.y(), -1.0};
        Point3 p_norm2 = K2_inv * Point3{m_images[img_2].m_keys[key_idx2].m_coords.x(), m_images[img_2].m_keys[key_idx2].m_coords.y(), -1.0};

        // Put the translation in standard form
        Observations observations;
        observations.push_back(Observation{Point2{p_norm1.head<2>() / p_norm1.z()}, cameras[0]});
        observations.push_back(Observation{Point2{p_norm2.head<2>() / p_norm2.z()}, cameras[1]});

        double error;
        auto position = Triangulate(observations, &error);

        error = (cameras[0].m_focal_length + cameras[1].m_focal_length) * 0.5 * sqrt(error * 0.5);

        if (error > m_options.projection_estimation_threshold) continue;

        auto& key = GetKey(img_1, key_idx1);

        GetKey(img_1, key_idx1).m_extra = points.size();
        GetKey(img_2, key_idx2).m_extra = points.size();

        int track_idx = GetKey(img_1, key_idx1).m_track;
        m_tracks[track_idx].m_extra = points.size();

        ImageKeyVector views;
        views.push_back(ImageKey{0, key_idx1});
        views.push_back(ImageKey{1, key_idx2});

        points.push_back(PointData{position, key.m_color, views});
    }

    UpdateGeometryDisplay(cameras, added_order, points);
}

bool MainFrame::EstimateRelativePose(int i1, int i2, Camera* camera1, Camera* camera2)
{
    ScopedTimer timer{m_profile_manager, "[EstimateRelativePose]"};

    auto &matches = m_matches.GetMatchList(GetMatchIndex(i1, i2));

    Point2Vec projections1; projections1.reserve(matches.size());
    Point2Vec projections2; projections2.reserve(matches.size());

    for (const auto& match : matches)
    {
        projections1.push_back(m_images[i1].m_keys[match.first].m_coords);
        projections2.push_back(m_images[i2].m_keys[match.second].m_coords);
    }

    Mat3 R; Vec3 t;
    int num_inliers = ComputeRelativePoseRansac(projections1, projections2, camera1->GetIntrinsicMatrix(), camera2->GetIntrinsicMatrix(), m_options.ransac_threshold_five_point, m_options.ransac_rounds_five_point, &R, &t);
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
    auto& matches = m_matches.GetMatchList(GetMatchIndex(img1, img2));
    auto& tracks1 = m_images[img1].m_visible_points;
    auto& tracks2 = m_images[img2].m_visible_points;

    // Find tracks visible from both cameras
    IntVec intersection;
    std::set_intersection(tracks1.begin(), tracks1.end(), tracks2.begin(), tracks2.end(), std::back_inserter(intersection));

    if (intersection.empty()) return;

    matches.clear();
    matches.reserve(intersection.size());

    for (const int& track : intersection)
    {
        auto p      = std::lower_bound(tracks1.begin(), tracks1.end(), track);
        auto offset = std::distance(tracks1.begin(), p);
        int key1    = m_images[img1].m_visible_keys[offset];

        p           = std::lower_bound(tracks2.begin(), tracks2.end(), track);
        offset      = std::distance(tracks2.begin(), p);
        int key2    = m_images[img2].m_visible_keys[offset];

        matches.push_back(KeypointMatch{key1, key2});
    }
}

void MainFrame::SetMatchesFromPoints()
{
    // Clear all matches
    m_matches.RemoveAll();

    for (const auto& point : m_points)
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
                m_matches.AddMatch(MatchIndex{view1.first, view2.first}, KeypointMatch{view1.second, view2.second});
            }
        }
    }
}

int MainFrame::FindCameraWithMostMatches(const IntVec& added_order, int& max_matches, const PointVec& points)
{
    ScopedTimer timer{m_profile_manager, "[FindCameraWithMostMatches]"};

    int i_best = -1;
    max_matches = 0;

    for (int i = 0; i < GetNumImages(); i++)
    {
        if (m_images[i].m_ignore_in_bundle) continue;

        // Check if we added this image already
        if (std::find(added_order.begin(), added_order.end(), i) != added_order.end()) continue;

        // Find the tracks seen by this image
        int num_existing_matches = 0;
        for (const auto& track : m_images[i].m_visible_points)
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

IntVec MainFrame::FindCamerasWithNMatches(int n, const IntVec& added_order, const PointVec& points)
{
    ScopedTimer timer{m_profile_manager, "[FindCamerasWithNMatches]"};

    IntVec found_cameras;

    for (int i = 0; i < GetNumImages(); i++)
    {
        if (m_images[i].m_ignore_in_bundle) continue;

        // Check if we added this image already
        if (std::find(added_order.begin(), added_order.end(), i) != added_order.end()) continue;

        // Find the tracks seen by this image
        int num_existing_matches = 0;
        for (const auto& track : m_images[i].m_visible_points)
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

Camera MainFrame::InitializeImage(int image_idx, int camera_idx, PointVec& points)
{
    wxLogMessage("[InitializeImage] Initializing camera %d", image_idx);

    auto& image = m_images[image_idx];
    image.SetTracks();

    // **** Connect the new camera to any existing points ****
    Camera      camera_new;
    Point3Vec   points_solve;
    Point2Vec   projs_solve;
    IntVec      idxs_solve;
    IntVec      keys_solve;

    // Find the tracks seen by this image
    for (int i = 0; i < image.m_visible_points.size(); i++)
    {
        int track   = image.m_visible_points[i];
        int key     = image.m_visible_keys[i];

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
        wxLogMessage("[InitializeImage] Couldn't initialize (too few points)");
        return camera_new;
    }

    // **** Solve for the camera position ****
    Mat3 Kinit, Rinit;
    Vec3 tinit;
    IntVec inliers, inliers_weak, outliers;
    bool found = FindAndVerifyCamera(points_solve, projs_solve, &Kinit, &Rinit, &tinit, inliers, inliers_weak, outliers);

    if (found)
    {
        // Set up the new camera
        camera_new.m_R = Rinit;
        camera_new.m_t = -(Rinit.transpose() * tinit);

        // Set up the new focal length
        camera_new.m_focal_length = camera_new.m_init_focal_length = image.m_init_focal;
    } else
    {
        wxLogMessage("[InitializeImage] Couldn't initialize (couldn't solve for the camera position)");
        return camera_new;
    }

    // Optimize camera parameters
    Point3Vec   points_final;
    Point2Vec   projs_final;
    IntVec      idxs_final;
    IntVec      keys_final;

    for (const auto& idx : inliers_weak)
    {
        points_final.push_back(points_solve[idx]);
        projs_final.push_back(projs_solve[idx]);
        idxs_final.push_back(idxs_solve[idx]);
        keys_final.push_back(keys_solve[idx]);
    }

    RefineCameraParameters(&camera_new, points_final, projs_final, inliers);

    if ((inliers.size() < 8) || (camera_new.m_focal_length < 0.1 * image.GetWidth()))
    {
        wxLogMessage("[InitializeImage] Bad camera");
        return camera_new;
    }

    // Point the keys to their corresponding points
    for (const auto& i : inliers)
    {
        image.m_keys[keys_final[i]].m_extra = idxs_final[i];
        points[idxs_final[i]].m_views.push_back(ImageKey{camera_idx, keys_final[i]});
    }

    camera_new.m_adjusted = true;

    return camera_new;
}

bool MainFrame::FindAndVerifyCamera(const Point3Vec& points, const Point2Vec& projections, Mat3* K, Mat3* R, Vec3* t, IntVec& inliers, IntVec& inliers_weak, IntVec& outliers)
{
    ScopedTimer timer{m_profile_manager, "[FindAndVerifyCamera]"};

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
        Point3 q = *K * (Rigid * EuclideanToHomogenous(points[j]));
        Point2 projection = -q.head<2>() / q.z();
        double error = (projection - projections[j]).norm();

        if (error < m_options.projection_estimation_threshold) inliers.push_back(j);
        if (error < (16.0 * m_options.projection_estimation_threshold))
        {
            inliers_weak.push_back(j);
        } else
        {
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

void MainFrame::RefineCameraParameters(Camera* camera, const Point3Vec& points, const Point2Vec& projections, IntVec& inliers)
{
    ScopedTimer timer{m_profile_manager, "[RefineCameraParameters]"};

    Point3Vec points_curr(points);
    Point2Vec projs_curr(projections);

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
            Point2 projection = camera->Project(points_curr[i]);
            errors.push_back((projection - projs_curr[i]).norm());
        }

        // Sort and histogram errors
        double median       = util::GetNthElement(util::iround(0.90 * points_curr.size()), errors);
        double threshold    = util::clamp(2.4 * median, m_options.min_reprojection_error_threshold, m_options.max_reprojection_error_threshold);
        double avg          = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        wxLogMessage("[RefineCameraParameters] Mean error [%d pts]: %.3f [med: %.3f, outlier threshold: %.3f]", points_curr.size(), avg, median, threshold);

        Point3Vec   points_next;
        Point2Vec   projs_next;
        IntVec      inliers_next;

        for (int i = 0; i < points_curr.size(); i++)
        {
            if (errors[i] < threshold)
            {
                inliers_next.push_back(inliers[i]);

                points_next.push_back(points_curr[i]);
                projs_next.push_back(projs_curr[i]);
            }
        }

        if (points_next.size() == points_curr.size()) break;	// We're done

        points_curr = points_next;
        projs_curr  = projs_next;
        inliers     = inliers_next;

        if (points_curr.size() == 0) break;	// Out of measurements

        round++;
    }

    wxLogMessage("[RefineCameraParameters] Exiting after %d rounds with %d/%d points", round + 1, points_curr.size(), points.size());
}

void MainFrame::BundleAdjust(CamVec& cameras, const IntVec& added_order, PointVec& points)
{
    wxLogMessage("[BundleAdjust] Optimizing...");

    const int   num_cameras     = static_cast<int>(cameras.size());
    const int   num_pts         = static_cast<int>(points.size());
    const int   min_points      = 20;
    int         round           = 0;
    int         num_outliers    = 0;
    int         total_outliers  = 0;

    IntVec      remap(num_pts, -1);
    Point3Vec   nz_pts(num_pts);

    do
    {
        if (num_pts - total_outliers < min_points)
        {
            wxLogMessage("[BundleAdjust] Too few points remaining, exiting!");
            break;
        }

        ScopedTimer timer{m_profile_manager, "[BundleAdjust]"};

        // Set up the projections
        int num_projections = 0;
        for (const auto& point : points) num_projections += point.m_views.size();
        std::vector<double>         projections;
        IntVec                      pidx;
        IntVec                      cidx;
        std::vector<unsigned int>	num_vis(num_cameras, 0);

        int nz_count = 0;
        for (int i = 0; i < num_pts; i++)
        {
            if (!points[i].m_views.empty())
            {
                for (const auto& view : points[i].m_views)
                {
                    projections.push_back(GetKey(added_order[view.first], view.second).m_coords.x());
                    projections.push_back(GetKey(added_order[view.first], view.second).m_coords.y());

                    pidx.push_back(nz_count);
                    cidx.push_back(view.first);

                    num_vis[view.first]++;
                }

                remap[i] = nz_count;
                nz_pts[nz_count] = points[i].m_pos;
                nz_count++;
            }
        }
        
        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        options.linear_solver_type        = ceres::DENSE_SCHUR;
        options.num_threads               = omp_get_max_threads();
        options.num_linear_solver_threads = omp_get_max_threads();

        // Set up initial parameters
        int num_nz_points = nz_count;

        int cnp = 9;
        int pnp = 3;

        unsigned int num_parameters = pnp * num_nz_points + cnp * num_cameras;

        double* init_x = new double[num_parameters];
        double* pcameras = init_x;
        double* ppoints = init_x + cnp * num_cameras;

        // Insert camera parameters
        double* ptr = init_x;

        int idx = 0;
        for (const auto &camera : cameras)
        {
            // Get the rotation
            Vec3 axis = RotationMatrixToAngleAxis(camera.m_R);
            Vec3 t    = -(camera.m_R * camera.m_t);

            double f  = camera.m_focal_length;
            double k1 = camera.m_k[0];
            double k2 = camera.m_k[1];

            ptr[idx] = axis[0]; idx++;
            ptr[idx] = axis[1]; idx++;
            ptr[idx] = axis[2]; idx++;
            ptr[idx] = t[0];    idx++;
            ptr[idx] = t[1];    idx++;
            ptr[idx] = t[2];    idx++;
            ptr[idx] = f;       idx++;
            ptr[idx] = k1;      idx++;
            ptr[idx] = k2;      idx++;
        }

        // Insert point parameters
        for (const auto& point : points)
        {
            if (!point.m_views.empty())
            {
                ptr[idx] = point.m_pos.x(); idx++;
                ptr[idx] = point.m_pos.y(); idx++;
                ptr[idx] = point.m_pos.z(); idx++;
            }
        }

        for (int i = 0; i < num_projections; ++i)
        {
            // Each Residual block takes a point and a camera as input and outputs a 2 dimensional residual
            ceres::CostFunction* cost_function = CreateBundlerCostFunction(projections[2 * i + 0], projections[2 * i + 1]);

            ceres::LossFunction* loss_function = nullptr;
            switch (m_options.selected_loss)
            {
            case Options::loss_function::squared:   break;
            case Options::loss_function::huber:     loss_function = new ceres::HuberLoss{m_options.loss_function_scale};    break;
            case Options::loss_function::softlone:  loss_function = new ceres::SoftLOneLoss{m_options.loss_function_scale}; break;
            case Options::loss_function::cauchy:    loss_function = new ceres::CauchyLoss{m_options.loss_function_scale};   break;
            case Options::loss_function::arctan:    loss_function = new ceres::ArctanLoss{m_options.loss_function_scale};   break;
            default: break;
            }

            // Each observation correponds to a pair of a camera and a point which are identified by camera_index[i] and point_index[i] respectively
            double* camera = pcameras  + cnp * cidx[i];
            double* point  = ppoints   + pnp * pidx[i];

            problem.AddResidualBlock(cost_function, loss_function, camera, point);
        }

        // Now add the priors
        for (int i = 0; i < num_cameras; i++)
        {
            ceres::CostFunction* prior_cost_function;

            prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>{new PriorError(6, cameras[i].m_init_focal_length, m_options.focal_length_constrain_weight * num_vis[i])};
            problem.AddResidualBlock(prior_cost_function, nullptr, pcameras + cnp * i);

            // Take care of priors on distortion parameters
            prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>{new PriorError(7, 0.0, m_options.distortion_constrain_weight * num_vis[i])};
            problem.AddResidualBlock(prior_cost_function, nullptr, pcameras + cnp * i);

            prior_cost_function = new ceres::AutoDiffCostFunction<PriorError, 1, 9>{new PriorError(8, 0.0, m_options.distortion_constrain_weight * num_vis[i])};
            problem.AddResidualBlock(prior_cost_function, nullptr, pcameras + cnp * i);
        }

        // Make call to Ceres
        wxLogMessage("[Ceres] %d points",       num_nz_points);
        wxLogMessage("[Ceres] %d cameras",      num_cameras);
        wxLogMessage("[Ceres] %d params",       num_parameters);
        wxLogMessage("[Ceres] %d projections",  num_projections);
        ceres::Solve(options, &problem, &summary);
        wxLogMessage("[Ceres] %s", summary.BriefReport().c_str());

        ptr = init_x;
        for (auto& camera : cameras)
        {
            // Get the camera parameters
            Vec3 axis, t;

            axis[0] = *ptr; ptr++;
            axis[1] = *ptr; ptr++;
            axis[2] = *ptr; ptr++;
            t[0]    = *ptr; ptr++;
            t[1]    = *ptr; ptr++;
            t[2]    = *ptr; ptr++;

            camera.m_R = AngleAxisToRotationMatrix(axis);
            camera.m_t = -(camera.m_R.transpose() * t);
            camera.m_focal_length = *ptr; ptr++;
            camera.m_k[0] = *ptr; ptr++;
            camera.m_k[1] = *ptr; ptr++;
        }

        // Insert point parameters
        for (int i = 0; i < num_nz_points; i++)
        {
            nz_pts[i].x() = *ptr; ptr++;
            nz_pts[i].y() = *ptr; ptr++;
            nz_pts[i].z() = *ptr; ptr++;
        }

        // Check for outliers
        IntVec outliers;
        for (int i = 0; i < num_cameras; i++)
        {
            auto& image = m_images[added_order[i]];

            // Compute inverse distortion parameters
            Vec6 k_dist; k_dist << 0.0, 1.0, 0.0, cameras[i].m_k[0], 0.0, cameras[i].m_k[1];
            double w_2 = 0.5 * image.GetWidth();
            double h_2 = 0.5 * image.GetHeight();
            double max_radius = sqrt(w_2 * w_2 + h_2 * h_2) / cameras[i].m_focal_length;
            cameras[i].m_k_inv = InvertDistortion(0.0, max_radius, k_dist);

            std::vector<double> dists;
            for (const auto& key : image.m_keys)
            {
                if (key.m_extra >= 0)
                {
                    int pt_idx = key.m_extra;
                    Point2 projection = cameras[i].Project(nz_pts[remap[pt_idx]]);

                    dists.push_back((projection - key.m_coords).norm());
                }
            }

            // Estimate the median of the distances and compute the average reprojection error for this camera
            double median   = util::GetNthElement(util::iround(0.8 * dists.size()), dists);
            double thresh   = util::clamp(2.4 * median, m_options.min_reprojection_error_threshold, m_options.max_reprojection_error_threshold);
            double avg      = std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
            wxLogMessage("[BundleAdjust] Mean error cam %d[%d] [%d pts]: %.3f [med: %.3f, outlier threshold: %.3f]", i, added_order[i], dists.size(), avg, median, thresh);

            int pt_count = 0;
            for (const auto& key : image.m_keys)
            {
                int pt_idx = key.m_extra;
                if (pt_idx < 0) continue;

                if (dists[pt_count] > thresh)
                {
                    // Remove this point from consideration
                    bool found = false;
                    for (const int& outlier : outliers) if (outlier == pt_idx)
                    {
                        found = true;
                        break;
                    }

                    if (!found) outliers.push_back(pt_idx);
                }

                pt_count++;
            }
        }

        // Remove outlying points
        for (const auto& idx : outliers)
        {
            points[idx].m_color = Vec3{0.0, 0.0, -1.0};

            for (const auto& view : points[idx].m_views)
            {
                int v = view.first;
                int k = view.second;

                // Sanity check
                if (GetKey(added_order[v], k).m_extra != idx) wxLogMessage("[BundleAdjust] Error! Entry for (%d, %d) should be %d, but is %d", added_order[v], k, idx, GetKey(added_order[v], k).m_extra);

                GetKey(added_order[v], k).m_extra = -2;
            }

            points[idx].m_views.clear();
        }

        num_outliers = outliers.size();
        total_outliers += num_outliers;

        wxLogMessage("[BundleAdjust] Removed %d outliers", num_outliers);

        RemoveBadPointsAndCameras(cameras, added_order, points);

        for (int i = 0; i < num_pts; i++) if (remap[i] != -1) points[i].m_pos = nz_pts[remap[i]];

        UpdateGeometryDisplay(cameras, added_order, points);

        round++;
        delete[] init_x;
    } while (num_outliers > m_options.outlier_threshold_ba);
}

void MainFrame::AddNewPoints(const CamVec& cameras, const IntVec& added_order, PointVec& points)
{
    ScopedTimer timer(m_profile_manager, "[AddNewPoints]");

    std::vector<ImageKeyVector> new_tracks;
    IntVec                      track_idxs;
    IntVec                      tracks_seen(m_tracks.size(), -1);

    // Gather up the projections of all the new tracks
    for (int i = 0; i < cameras.size(); i++)
    {
        int image_idx1 = added_order[i];
        int num_keys = GetNumKeys(image_idx1);

        for (int j = 0; j < num_keys; j++)
        {
            KeyPoint& key = GetKey(image_idx1, j);

            if (key.m_track == -1) continue;    // Key belongs to no track
            if (key.m_extra != -1) continue;    // Key is outlier or has already been added

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
                track.push_back(ImageKey{i, j});
                new_tracks.push_back(track);
                track_idxs.push_back(track_idx);
            } else
            {
                new_tracks[seen].push_back(ImageKey{i, j});
            }
        }
    }

    // Now for each (sub) track, triangulate to see if the track is consistent
    int num_ill_conditioned     = 0;
    int num_high_reprojection   = 0;
    int num_cheirality_failed   = 0;
    int num_added               = 0;
    int num_tracks = (int)new_tracks.size();

    for (int i = 0; i < num_tracks; i++)
    {
        int num_views = (int)new_tracks[i].size();
        if (num_views < 2) continue;    // Not enough views

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

                KeyPoint& key1 = GetKey(image_idx1, key_idx1);
                KeyPoint& key2 = GetKey(image_idx2, key_idx2);

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
        for (auto& track : new_tracks[i])
        {
            auto cam = cameras[track.first];

            int image_idx = added_order[track.first];
            auto& key     = GetKey(image_idx, track.second);

            Point3 pn = cam.GetIntrinsicMatrix().inverse() * EuclideanToHomogenous(key.m_coords);
            Point2 pu = UndistortNormalizedPoint(-pn.head<2>(), cam.m_k_inv);

            observations.push_back(Observation{pu, cam});
        }

        auto point = Triangulate(observations);

        double error = 0.0;
        for (const auto& track : new_tracks[i])
        {
            int image_idx = added_order[track.first];
            auto& key = GetKey(image_idx, track.second);

            Point2 projection = cameras[track.first].Project(point);
            error += (projection - key.m_coords).squaredNorm();
        }
        error = sqrt(error / new_tracks[i].size());

        if (_isnan(error) || error > 16.0)
        {
            num_high_reprojection++;
            continue;
        }

        bool all_in_front = true;
        for (const auto& track : new_tracks[i])
        {
            if (!CheckCheirality(point, cameras[track.first]))
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
        auto& key = GetKey(added_order[new_tracks[i][0].first], new_tracks[i][0].second);

        points.push_back(PointData{point, key.m_color, new_tracks[i]});

        // Set the point index on the keys
        for (const auto& track : new_tracks[i]) GetKey(added_order[track.first], track.second).m_extra = points.size() - 1;

        m_tracks[track_idxs[i]].m_extra = points.size() - 1;

        num_added++;
    }

    wxLogMessage("[AddNewPoints] Added %d new points",           num_added);
    wxLogMessage("[AddNewPoints] Ill-conditioned tracks: %d",    num_ill_conditioned);
    wxLogMessage("[AddNewPoints] Bad reprojections: %d",         num_high_reprojection);
    wxLogMessage("[AddNewPoints] Failed cheirality checks: %d",  num_cheirality_failed);
}

int MainFrame::RemoveBadPointsAndCameras(const CamVec& cameras, const IntVec& added_order, PointVec& points)
{
    ScopedTimer timer{m_profile_manager, "[RemoveBadPointsAndCameras]"};

    int num_pruned = 0;

    for (auto& point : points)
    {
        if (point.m_views.empty()) continue;

        double max_angle = 0.0;
        for (int j = 0; j < point.m_views.size(); j++)
        {
            int v1{point.m_views[j].first};

            Point3 re1 = point.m_pos - cameras[v1].m_t;
            re1 /= re1.norm();

            for (int k = j + 1; k < point.m_views.size(); k++)
            {
                int v2(point.m_views[k].first);

                Point3 re2 = point.m_pos - cameras[v2].m_t;
                re2 /= re2.norm();

                double angle = acos(util::clamp(re1.dot(re2), (-1.0 + 1.0e-8), (1.0 - 1.0e-8)));

                if (angle > max_angle) max_angle = angle;
            }
        }

        if (util::rad2deg(max_angle) < 0.5 * m_options.ray_angle_threshold)
        {
            for (const auto& view : point.m_views) GetKey(added_order[view.first], view.second).m_extra = -1;

            point.m_color = Vec3{0.0, 0.0, -1.0};
            point.m_views.clear();

            num_pruned++;
        }
    }

    wxLogMessage("[RemoveBadPointsAndCameras] Pruned %d points", num_pruned);

    return num_pruned;
}

void MainFrame::RadiusOutlierRemoval(double threshold, const IntVec& added_order, PointVec& points)
{
    ScopedTimer timer{m_profile_manager, "[RadiusOutlierRemoval]"};

    std::vector<int> outliers;

    std::vector<float> coords;
    for (const auto& point : points)
    {
        coords.push_back((float)point.m_pos.x());
        coords.push_back((float)point.m_pos.y());
        coords.push_back((float)point.m_pos.z());
    }

    const auto num_points = points.size();

    flann::Matrix<float> train{coords.data(), num_points, 3};
    flann::Matrix<float> query{coords.data(), num_points, 3};
    flann::Matrix<int>   indices{new int[num_points * 2], num_points, 2};
    flann::Matrix<float> dists{new float[num_points * 2], num_points, 2};

    flann::Index<flann::L2<float>> index{train, flann::KDTreeSingleIndexParams{}};
    index.buildIndex();

    flann::SearchParams params{m_options.matching_checks};
    params.cores = 0;
    index.knnSearch(query, indices, dists, 2, params);

    std::vector<float> d;
    for (int i = 0; i < num_points; ++i) d.push_back(*(dists[i] + 1));
    double thresh = util::GetNthElement(util::iround(threshold * d.size()), d);
    for (int i = 0; i < num_points; ++i) if (d[i] > thresh) outliers.push_back(i);

    delete[] indices.ptr();
    delete[] dists.ptr();

    for (const auto& idx : outliers)
    {
        points[idx].m_color = Vec3{0.0, 0.0, -1.0};

        for (const auto& view : points[idx].m_views)
        {
            int v = view.first;
            int k = view.second;

            // Sanity check
            if (GetKey(added_order[v], k).m_extra != idx) wxLogMessage("[RadiusOutlierRemoval] Error! Entry for (%d,%d) should be %d, but is %d", added_order[v], k, idx, GetKey(added_order[v], k).m_extra);

            GetKey(added_order[v], k).m_extra = -2;
        }

        points[idx].m_views.clear();
    }

    wxLogMessage("[RadiusOutlierRemoval] Removed %d outliers", outliers.size());
}