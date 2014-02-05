#include <numeric>
#include <unordered_set>
#include <queue>
#include <sstream>

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

#include "wx/progdlg.h"
#include "wx/textdlg.h"

#include "EigenTools.hpp"
#include "MainFrame.hpp"

#include "Eigen/Geometry"

#include "flann.hpp"

const wxEventTypeTag<wxThreadEvent> wxEVT_SFM_THREAD_UPDATE{wxNewEventType()};
const wxEventTypeTag<wxThreadEvent> wxEVT_SFM_THREAD_COMPLETE{wxNewEventType()};

MainFrame::MainFrame(wxWindow* parent)
    : MainFrame_base(parent)
{
    m_turntable_timer       = new wxTimer{this, ID_TIMER_TURNTABLE};
    m_reset_viewport_timer  = new wxTimer{this, ID_TIMER_RESET_VIEWPORT};

    m_rotate_cursor         = wxCursor{"rotate_cursor"};
    m_pan_cursor            = wxCursor{"pan_cursor"};
    m_zoom_cursor           = wxCursor{"zoom_cursor"};

    // Setup the image list
    m_img_ctrl->InsertColumn(0, "Name",         wxLIST_FORMAT_LEFT, 60);
    m_img_ctrl->InsertColumn(1, "Resolution",   wxLIST_FORMAT_LEFT, 75);
    m_img_ctrl->InsertColumn(2, "Focal",        wxLIST_FORMAT_LEFT, 50);
    m_img_ctrl->InsertColumn(3, "Features",     wxLIST_FORMAT_LEFT, 60);
    m_img_ctrl->InsertColumn(4, "k1",           wxLIST_FORMAT_LEFT, 40);
    m_img_ctrl->InsertColumn(5, "k2",           wxLIST_FORMAT_LEFT, 40);

    this->Bind(wxEVT_PG_CHANGED,            &MainFrame::OnOptionsChanged,       this);
    this->Bind(wxEVT_TIMER,                 &MainFrame::OnTimerUpdate,          this);
    this->Bind(wxEVT_SFM_THREAD_UPDATE,     &MainFrame::OnSFMThreadUpdate,      this);
    this->Bind(wxEVT_SFM_THREAD_COMPLETE,   &MainFrame::OnSFMThreadComplete,    this);
}

MainFrame::~MainFrame()
{
    this->Unbind(wxEVT_PG_CHANGED,          &MainFrame::OnOptionsChanged,       this);
    this->Unbind(wxEVT_TIMER,               &MainFrame::OnTimerUpdate,          this);
    this->Unbind(wxEVT_SFM_THREAD_UPDATE,   &MainFrame::OnSFMThreadUpdate,      this);
    this->Unbind(wxEVT_SFM_THREAD_COMPLETE, &MainFrame::OnSFMThreadComplete,    this);
}

void MainFrame::OnSFMThreadUpdate(wxThreadEvent& event)
{
    wxCriticalSectionLocker lock{m_points_cs};

    gly::Meshdata data;

    data.m_indices.resize(m_points.size());
    std::iota(data.m_indices.begin(), data.m_indices.end(), 0);

    data.m_vertices.reserve(m_points.size());
    for (const auto& point : m_points)
    {
        gly::Vertex vert;
        vert.m_position = glm::vec3{point.m_pos[0],   point.m_pos[1],   point.m_pos[2]};
        vert.m_color    = glm::vec4{point.m_color[0], point.m_color[1], point.m_color[2], 1.0f};
        data.m_vertices.push_back(vert);
    }

    m_scene->GetNode("Points")->GetMesh()->ChangeData(data);

    for (const auto& image : m_images)
    {
        if (!image.m_camera.m_adjusted) continue;

        image.m_camera_mesh->SetVisibilityMesh(true);

        image.m_camera_mesh->GetTransform().Reset();
        image.m_camera_mesh->GetTransform().Translate(glm::vec3{image.m_camera.m_t.x(), image.m_camera.m_t.y(), image.m_camera.m_t.z()});
        Eigen::Quaternion<double> quat(image.m_camera.m_R.transpose());
        image.m_camera_mesh->GetTransform().Rotate(glm::quat(quat.w(), quat.x(), quat.y(), quat.z()));
        image.m_camera_mesh->GetTransform().Scale(glm::vec3{0.02f, 0.02f, 0.02f});
    }

    for (int i = 0; i < (int)m_images.size(); i++)
    {
        const auto& cam = m_images[i].m_camera;
        if (!cam.m_adjusted) continue;

        wxString focal, k1, k2;
        focal.Printf("%.1f", cam.m_focal_length);
        k1.Printf("%.2f", cam.m_k(0));
        k2.Printf("%.2f", cam.m_k(1));

        m_img_ctrl->SetItem(i, 2, focal, -1);
        m_img_ctrl->SetItem(i, 4, k1, -1);
        m_img_ctrl->SetItem(i, 5, k2, -1);
    }

    m_gl_canvas->Refresh(false);
}

void MainFrame::OnSFMThreadComplete(wxThreadEvent& event)
{
    m_sfm_done = true;

    for (int i = 0; i < (int)m_images.size(); i++)
    {
        const auto& cam = m_images[i].m_camera;
        if (!cam.m_adjusted) continue;

        wxString focal, k1, k2;
        focal.Printf("%.1f", cam.m_focal_length);
        k1.Printf("%.2f", cam.m_k(0));
        k2.Printf("%.2f", cam.m_k(1));

        m_img_ctrl->SetItem(i, 2, focal, -1);
        m_img_ctrl->SetItem(i, 4, k1, -1);
        m_img_ctrl->SetItem(i, 5, k2, -1);
    }
}

void MainFrame::InitializeLog()
{
    // Redirect log messages
    wxLog::SetActiveTarget(new wxLogTextCtrl{m_tc_log});
    wxLogMessage("Log initialized");
    wxLogMessage("OS: %s", wxPlatformInfo::Get().GetOperatingSystemDescription());

    auto font = new wxFont{9, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, "consolas"};
    m_tc_log->SetFont(*font);
}

void MainFrame::InitializeCameraDatabase()
{
    // Try to read the camera data file
    if(ReadCamDBFile("CamDB.txt"))  wxLogMessage("%i entries found in CamDB.txt", static_cast<int>(m_camDB.size()));
    else                            wxLogMessage("Error: Camera database not found!");
}

bool MainFrame::AddImage(const std::string& filename, const std::string& filename_short)
{
    ImageData img{filename, filename_short};

    if(!img.GetExifInfo())
    {
        wxLogMessage("Can't get Exif info");
        return false;
    } else
    {
        if (FindCameraInDatabase(img))
        {
            std::ostringstream node_name;
            node_name << "camera " << m_images.size();
            img.m_node_name = node_name.str();

            m_scene->MakeNode(node_name.str(), "dslr_mesh", "cam_program");
            m_scene->AssignTexture(node_name.str(), "dslr_diffuse",     GL_TEXTURE0,        gly::SPL_MIPMAP_LINEAR);
            m_scene->AssignTexture(node_name.str(), "dslr_normal",      GL_TEXTURE0 + 1,    gly::SPL_MIPMAP_LINEAR);
            m_scene->AssignTexture(node_name.str(), "dslr_specular",    GL_TEXTURE0 + 2,    gly::SPL_MIPMAP_LINEAR);
            m_scene->AssignTexture(node_name.str(), "dslr_cube",        GL_TEXTURE0 + 3,    gly::SPL_MIPMAP_LINEAR);
            img.m_camera_mesh = m_scene->GetNode(node_name.str());
            img.m_camera_mesh->SetVisibilityMesh(false);

            m_images.push_back(img);

            return true;
        } else
        {
            wxString caption;
            caption << img.m_camera_make << " " << img.m_camera_model << ": " << "Camera model not found!";
            wxTextEntryDialog dlg{this, "Add CCD width (in mm) to database...\n(decimal point must be a dot!)", caption};

            CamDBEntry cam;

            while(true)
            {
                if (dlg.ShowModal() == wxID_CANCEL) return false;
                if (dlg.GetValue().ToCDouble(&cam.second)) break;
            }

            cam.first = img.m_camera_make + " " + img.m_camera_model;
            m_camDB.push_back(cam);

            FindCameraInDatabase(img);
            AddCamDBFileEntry();

            m_images.push_back(img);
            return true;
        }
    }
}

bool MainFrame::FindCameraInDatabase(ImageData& img)
{
    auto found = std::find_if(m_camDB.begin(), m_camDB.end(), [&](CamDBEntry& entry)
    {
        return (entry.first.find(img.m_camera_model) != std::string::npos);
    });

    if (found != m_camDB.end())
    {
        img.m_ccd_width = found->second;
        if (img.GetWidth() > img.GetHeight())   img.m_init_focal = img.GetWidth()  * (img.m_init_focal_mm / img.m_ccd_width);
        else                                    img.m_init_focal = img.GetHeight() * (img.m_init_focal_mm / img.m_ccd_width);

        return true;
    }

    return false;
}

void MainFrame::DetectFeaturesAll()
{
    int num_images = GetNumImages();

    // Show progress dialog
    wxProgressDialog dialog{"Progress", "Detecting features...", num_images, this,
                            wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_ELAPSED_TIME |
                            wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME};

    // Detect features
    for (int i = 0; i < num_images; i++)
    {
        DetectFeatures(i);

        wxString numfeat;
        numfeat << GetNumKeys(i);
        m_img_ctrl->SetItem(i, 3, numfeat);
        dialog.Update(i + 1);
        wxSafeYield();
    }

    // Update program state
    m_features_detected = true;
}

void MainFrame::DetectFeatures(int img_idx)
{
    ScopedTimer timer{m_profile_manager, "[DetectFeatures]"};
    m_images[img_idx].DetectFeatures(m_options);
    wxLogMessage("[DetectFeatures] %s: found %i features", m_images[img_idx].m_filename_short.c_str(), GetNumKeys(img_idx));
}

void MainFrame::MatchAllAkaze()
{
    typedef flann::Hamming<unsigned char> Distance;
    typedef Distance::ElementType         ElementType;
    typedef Distance::ResultType          DistanceType;

    int num_images = GetNumImages();
    int num_pairs = (num_images * (num_images - 1)) / 2;
    int progress_idx = 0;

    // Clean up
    for (auto& image : m_images)
    {
        image.m_visible_points.clear();
        image.m_visible_keys.clear();
        image.m_key_flags.clear();
    }

    m_tracks.clear();
    m_matches = MatchTable{num_images};
    m_matches.RemoveAll();
    m_transforms.clear();

    // Show progress dialog
    wxProgressDialog dialog{"Progress", "Matching images...", num_pairs, this, wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME};

    for (int i = 1; i < num_images; i++)
    {
        wxLogMessage("[MatchAll] Matching %s...", m_images[i].m_filename_short.c_str());

        m_desc_length = m_images[i].m_descriptors_akaze.cols;

        flann::Index<Distance> flann_index{flann::Matrix<ElementType>(m_images[i].m_descriptors_akaze.data, m_images[i].m_keys.size(), m_desc_length),
                                           flann::LshIndexParams{12, 20, 2}};
        flann_index.buildIndex();

        for (int j = 0; j < i; j++)
        {
            // Setup query data
            const int num_descriptors = m_images[j].m_descriptors_akaze.rows;

            std::vector<std::size_t>  indices(num_descriptors * 2);
            std::vector<DistanceType> dists(num_descriptors * 2);

            // Match!
            {
                ScopedTimer timer{m_profile_manager, "[MatchImagePair]"};

                flann::SearchParams params{m_options.matching_checks};
                params.cores = 0;
                flann_index.knnSearch(flann::Matrix<ElementType>(m_images[j].m_descriptors_akaze.data, num_descriptors, m_desc_length),
                                      flann::Matrix<std::size_t>(indices.data(), num_descriptors, 2),
                                      flann::Matrix<DistanceType>(dists.data(), num_descriptors, 2),
                                      2, params);
            }

            // Store putative matches in ptpairs
            IntPairVec tmp_matches;
            for (int k = 0; k < num_descriptors; ++k)
            {
                if (dists[2 * k] < (m_options.matching_distance_ratio * dists[2 * k + 1]))
                {
                    tmp_matches.push_back(IntPair{indices[2 * k], k});
                }
            }

            if (tmp_matches.size() < m_options.matching_min_matches)
            {
                wxLogMessage("[MatchAll]    ...with %s: no match before fundamental", m_images[j].m_filename_short.c_str());
                progress_idx++;
                dialog.Update(progress_idx);
                wxSafeYield();
                continue;
            }

            int num_putative = static_cast<int>(tmp_matches.size());

            // Find and delete double matches
            int num_pruned = PruneDoubleMatches(tmp_matches);

            // Compute the fundamental matrix and remove outliers
            int num_inliers = ComputeEpipolarGeometry(i, j, tmp_matches);

            // Compute transforms
            TransformInfo tinfo;
            MatchIndex midx{i, j};
            tinfo.m_inlier_ratio = ComputeHomography(i, j, tmp_matches);

            // Store matches and transforms
            if (num_inliers > m_options.matching_min_matches)
            {
                TransformsEntry trans_entry{midx, tinfo};
                m_transforms.insert(trans_entry);

                SetMatch(i, j);
                auto& matches = m_matches.GetMatchList(GetMatchIndex(i, j));

                matches.clear();
                matches.reserve(num_inliers);

                for (const auto& match : tmp_matches) matches.push_back(KeypointMatch{match.first, match.second});

                // Be verbose
                wxLogMessage("[MatchAll]    ...with %s: %i inliers (%i putative, %i duplicates pruned), ratio = %.2f",
                    m_images[j].m_filename_short.c_str(), num_inliers, num_putative, num_pruned, tinfo.m_inlier_ratio);
            } else
            {
                // Be verbose
                wxLogMessage("[MatchAll]    ...with %s: no match", m_images[j].m_filename_short.c_str());
            }

            progress_idx++;
            dialog.Update(progress_idx);
            wxSafeYield();
        }
    }

    MakeMatchListsSymmetric();
    ComputeTracks();
    m_matches.RemoveAll();

    m_matches_loaded = true;
}

void MainFrame::MatchAll()
{
    typedef flann::L2<float>      Distance;
    typedef Distance::ElementType ElementType;
    typedef Distance::ResultType  DistanceType;

    int num_images = GetNumImages();
    int num_pairs = (num_images * (num_images - 1)) / 2;
    int progress_idx = 0;

    // Clean up
    for (auto& image : m_images)
    {
        image.m_visible_points.clear();
        image.m_visible_keys.clear();
        image.m_key_flags.clear();
    }

    m_tracks.clear();
    m_matches = MatchTable{num_images};
    m_matches.RemoveAll();
    m_transforms.clear();

    // Show progress dialog
    wxProgressDialog dialog("Progress", "Matching images...", num_pairs, this, wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

    for (int i = 1; i < num_images; i++)
    {
        wxLogMessage("[MatchAll] Matching %s...", m_images[i].m_filename_short.c_str());

        m_images[i].LoadDescriptors();
        m_desc_length = m_images[i].m_desc_size;

        // Create a search index
        ;
        flann::Index<flann::L2<float>> flann_index{flann::Matrix<ElementType>(m_images[i].m_descriptors.data(), m_images[i].m_keys.size(), m_desc_length),
                                                   flann::KDTreeIndexParams{m_options.matching_trees}};
        flann_index.buildIndex();

        for (int j = 0; j < i; j++)
        {
            m_images[j].LoadDescriptors();

            // Setup query data
            const int num_descriptors = static_cast<int>(m_images[j].m_keys.size());

            std::vector<std::size_t>  indices(num_descriptors * 2);
            std::vector<DistanceType> dists(num_descriptors * 2);

            // Match!
            {
                ScopedTimer timer{m_profile_manager, "[MatchImagePair]"};

                flann::SearchParams params{m_options.matching_checks};
                params.cores = 0;
                flann_index.knnSearch(flann::Matrix<ElementType>(m_images[j].m_descriptors.data(), num_descriptors, m_desc_length),
                                      flann::Matrix<std::size_t>(indices.data(), num_descriptors, 2),
                                      flann::Matrix<DistanceType>(dists.data(), num_descriptors, 2),
                                      2, params);
            }

            // Store putative matches in ptpairs
            IntPairVec tmp_matches;
            for (int k = 0; k < num_descriptors; ++k)
            {
                if (dists[2 * k] < (m_options.matching_distance_ratio * dists[2 * k + 1]))
                {
                    tmp_matches.push_back(IntPair{indices[2 * k], k});
                }
            }

            int num_putative = static_cast<int>(tmp_matches.size());

            // Find and delete double matches
            int num_pruned = PruneDoubleMatches(tmp_matches);

            // Compute the fundamental matrix and remove outliers
            int num_inliers = ComputeEpipolarGeometry(i, j, tmp_matches);

            // Compute transforms
            TransformInfo tinfo;
            MatchIndex midx{i, j};
            tinfo.m_inlier_ratio = ComputeHomography(i, j, tmp_matches);

            // Store matches and transforms
            if (num_inliers > m_options.matching_min_matches)
            {
                TransformsEntry trans_entry{midx, tinfo};
                m_transforms.insert(trans_entry);

                SetMatch(i, j);
                auto& matches = m_matches.GetMatchList(GetMatchIndex(i, j));

                matches.clear();
                matches.reserve(num_inliers);

                for (const auto& match : tmp_matches) matches.push_back(KeypointMatch{match.first, match.second});

                // Be verbose
                wxLogMessage("[MatchAll]    ...with %s: %i inliers (%i putative, %i duplicates pruned), ratio = %.2f",
                    m_images[j].m_filename_short.c_str(), num_inliers, num_putative, num_pruned, tinfo.m_inlier_ratio);
            } else
            {
                // Be verbose
                wxLogMessage("[MatchAll]    ...with %s: no match", m_images[j].m_filename_short.c_str());
            }

            m_images[j].ClearDescriptors();

            progress_idx++;
            dialog.Update(progress_idx);
            wxSafeYield();
        }
        m_images[i].ClearDescriptors();
    }

    MakeMatchListsSymmetric();
    ComputeTracks();
    m_matches.RemoveAll();

    m_matches_loaded = true;
}

int MainFrame::PruneDoubleMatches(IntPairVec& matches)
{
    ScopedTimer timer{m_profile_manager, "[PruneDoubleMatches]"};

    int num_before = matches.size();

    // Mark an index as duplicate if it's registered more than once
    std::unordered_set<int> duplicates;
    for(const auto& match : matches)
    {
        if (std::count_if(matches.begin(), matches.end(), [&](const IntPair& item) { return item.first == match.first; }) > 1) duplicates.insert(match.first);
    }

    auto found_in_duplicates = [&](const IntPair& match) { return duplicates.find(match.first) != duplicates.end(); };
    matches.erase(std::remove_if(matches.begin(), matches.end(), found_in_duplicates), matches.end());

    return num_before - matches.size();
}

int MainFrame::ComputeEpipolarGeometry(int idx1, int idx2, IntPairVec& matches)
{
    ScopedTimer timer{m_profile_manager, "[ComputeEpipolarGeometry]"};

    auto num_putative = matches.size();
    std::vector<cv::Point2f> points1, points2;
    std::vector<uchar> status;
    points1.reserve(num_putative);
    points2.reserve(num_putative);
    status.reserve(num_putative);

    const auto& keys1 = m_images[idx1].m_keys;
    const auto& keys2 = m_images[idx2].m_keys;

    for (const auto& match : matches)
    {
        points1.push_back(cv::Point2f(keys1[match.first].m_coords.x(), keys1[match.first].m_coords.y()));
        points2.push_back(cv::Point2f(keys2[match.second].m_coords.x(), keys2[match.second].m_coords.y()));
    }

    // Find the fundamental matrix
    double threshold(m_options.ransac_threshold_fundamental * std::max(m_images[idx1].GetWidth(), m_images[idx1].GetHeight()));
    cv::findFundamentalMat(cv::Mat{points1, true}, cv::Mat{points2, true}, status, cv::FM_RANSAC, threshold);

    // Remove outliers from ptpairs
    auto matches_old = matches;
    matches.clear();

    for (int m = 0; m < (int)status.size(); m++) if (status[m] == 1) matches.push_back(matches_old[m]);

    return (int)matches.size();
}

double MainFrame::ComputeHomography(int idx1, int idx2, const IntPairVec& matches)
{
    ScopedTimer timer{m_profile_manager, "[ComputeHomography]"};

    auto num_matches = matches.size();
    std::vector<cv::Point2f> points1, points2;
    std::vector<uchar> status;
    points1.reserve(num_matches);
    points2.reserve(num_matches);
    status.reserve(num_matches);

    auto& keys1 = m_images[idx1].m_keys;
    auto& keys2 = m_images[idx2].m_keys;

    for (const auto& match : matches)
    {
        points1.push_back(cv::Point2f(keys1[match.first].m_coords.x(), keys1[match.first].m_coords.y()));
        points2.push_back(cv::Point2f(keys2[match.second].m_coords.x(), keys2[match.second].m_coords.y()));
    }

    // Find the homography matrix
    double threshold(m_options.ransac_threshold_homography * std::max(m_images[idx1].GetWidth(), m_images[idx1].GetHeight()));
    cv::findHomography(cv::Mat{points1, true}, cv::Mat{points2, true}, status, cv::RANSAC, threshold);

    // Compute and return inlier ratio
    return std::count(status.begin(), status.end(), 1) / static_cast<double>(num_matches);
}

void MainFrame::ComputeTracks()
{
    ScopedTimer timer{m_profile_manager, "[ComputeTracks]"};

    int num_images = GetNumImages();

    // Clear all marks for new images
    for (int i = 0; i < num_images; i++)
    {
        // If this image has no neighbors, don't worry about its keys
        if (m_matches.GetNumNeighbors(i) == 0) continue;

        int num_features = m_images[i].m_keys.size();
        m_images[i].m_key_flags.resize(num_features);
    }

    int pt_idx = 0;

    // Sort all match lists
    for (int i = 0; i < num_images; i++)
    {
        std::for_each(m_matches.begin(i), m_matches.end(i), [&](AdjListElem val)
        {
            auto& list = val.m_match_list;
            std::sort(list.begin(), list.end(), [](KeypointMatch k1, KeypointMatch k2) { return k1.first < k2.first; });
        });
    }

    std::vector<bool> img_marked(num_images, false);

    IntVec touched{num_images};
    std::vector<TrackData> tracks;

    for (int i = 0; i < num_images; i++)
    {
        // If this image has no neighbors, skip it
        if (!m_matches.GetNumNeighbors(i)) continue;

        int num_features = m_images[i].m_keys.size();

        for (int j = 0; j < num_features; j++)
        {
            ImageKeyVector features;
            std::queue<ImageKey> features_queue;

            // Check if this feature was visited
            if (m_images[i].m_key_flags[j]) continue;   // already visited this feature

            // Reset flags
            int num_touched = touched.size();
            for (int k = 0; k < num_touched; k++) img_marked[touched[k]] = false;
            touched.clear();

            // Do a breadth first search given this feature
            m_images[i].m_key_flags[j] = true;

            features.push_back(ImageKey{i, j});
            features_queue.push(ImageKey{i, j});

            img_marked[i] = true;
            touched.push_back(i);

            int num_rounds = 0;
            while (!features_queue.empty())
            {
                num_rounds++;

                ImageKey feature = features_queue.front();
                features_queue.pop();

                int img1 = feature.first;
                int f1 = feature.second;
                KeypointMatch dummy;
                dummy.first = f1;

                // Check all adjacent images
                auto& nbrs = m_matches.GetNeighbors(img1);
                for (auto iter = nbrs.begin(); iter != nbrs.end(); iter++)
                {
                    unsigned int k = iter->m_index;
                    if (img_marked[k]) continue;

                    MatchIndex base = GetMatchIndex(img1, k);

                    auto& list = m_matches.GetMatchList(base);

                    // Do a binary search for the feature
                    auto p = std::equal_range(list.begin(), list.end(), dummy, [](KeypointMatch k1, KeypointMatch k2) { return k1.first < k2.first; });

                    if (p.first == p.second) continue;  // not found

                    int idx2 = (p.first)->second;

                    // Check if we visited this point already
                    if (m_images[k].m_key_flags[idx2]) continue;

                    // Mark and push the point
                    m_images[k].m_key_flags[idx2] = true;
                    features.push_back(ImageKey{k, idx2});
                    features_queue.push(ImageKey{k, idx2});

                    img_marked[k] = true;
                    touched.push_back(k);
                }
            } // While loop

            if (features.size() >= 2)
            {
                tracks.push_back(TrackData{features});
                pt_idx++;
            }
        } // For loop over features

        wxLogMessage("[ComputeTracks] Got %i tracks after checking image %i", (int)tracks.size(), i);
    } // For loop over images

    if (pt_idx != (int)tracks.size()) wxLogMessage("[ComputeTracks] Error: point count inconsistent!");

    // Create the new consistent match lists
    m_matches.RemoveAll();

    int num_pts = pt_idx;

    for (int i = 0; i < num_pts; i++)
    {
        for (const auto& view : tracks[i].m_views)
        {
            m_images[view.first].m_visible_points.push_back(i);
            m_images[view.first].m_visible_keys.push_back(view.second);
        }
    }

    // Save the tracks
    m_tracks = tracks;

    // Print track stats
    IntVec stats(num_images + 1);
    for (const auto& track : m_tracks) stats[track.m_views.size()] += 1;
    for (int i = 2; i < (num_images + 1); i++) wxLogMessage("[ComputeTracks] %i projections: %i", i, stats[i]);
}

void MainFrame::MakeMatchListsSymmetric()
{
    ScopedTimer timer{m_profile_manager, "[MakeMatchListsSymmetric]"};

    const unsigned int num_images = GetNumImages();

    std::vector<MatchIndex> matches;

    for (unsigned int i = 0; i < num_images; i++)
    {
        for (auto iter = m_matches.begin(i); iter != m_matches.end(i); ++iter)
        {
            unsigned int j = iter->m_index;

            MatchIndex idx     = GetMatchIndex(i, j);
            MatchIndex idx_rev = GetMatchIndex(j, i);

            auto& list = iter->m_match_list;

            m_matches.SetMatch(idx_rev);
            m_matches.ClearMatch(idx_rev);

            for (const auto& pair : list) m_matches.AddMatch(idx_rev, KeypointMatch(pair.second, pair.first));

            matches.push_back(idx);
        }
    }

    for (const auto& match : matches) SetMatch(static_cast<int>(match.second), static_cast<int>(match.first));

    matches.clear();
}

int MainFrame::GetNumTrackMatches(int img1, int img2)
{
    int num_intersections = 0;

    const auto& tracks1 = m_images[img1].m_visible_points;
    const auto& tracks2 = m_images[img2].m_visible_points;

    for (auto track_idx : tracks2) m_tracks[track_idx].m_extra = 0;
    for (auto track_idx : tracks1) m_tracks[track_idx].m_extra = 1;
    for (auto track_idx : tracks2) num_intersections += m_tracks[track_idx].m_extra;
    for (auto& track : m_tracks) track.m_extra = -1;

    return num_intersections;
}

double MainFrame::GetInlierRatio(int idx1, int idx2)
{
    double inlier_ratio = 0.0;

    auto match = m_transforms.find(MatchIndex{idx1, idx2});

    if (match != m_transforms.end())
    {
        inlier_ratio = match->second.m_inlier_ratio;
    } else
    {
        match = m_transforms.find(MatchIndex{idx2, idx1});
        if (match != m_transforms.end())
        {
            inlier_ratio = match->second.m_inlier_ratio;
        }
    }

    return inlier_ratio;
}
