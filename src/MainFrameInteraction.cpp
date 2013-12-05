#include <numeric>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/compatibility.hpp"
#include "glm/gtx/epsilon.hpp"
#include "glm/gtx/quaternion.hpp"

#include "wx/dir.h"

#include "MainFrame.hpp"

void MainFrame::OnUpdateUI(wxUpdateUIEvent& event)
{
    int id = event.GetId();

    switch(id)
    {
    case ID_RECONSTRUCT:                    event.Enable(m_has_images);         break;

    case ID_EXPORT_MATCHES:                 event.Enable(m_matches_loaded);     break;
    case ID_EXPORT_TRACKS:                  event.Enable(m_matches_loaded);     break;
    case ID_EXPORT_CMVS:                    event.Enable(m_sfm_done);           break;
    case ID_EXPORT_BUNDLE_FILE:             event.Enable(m_sfm_done);           break;
    case ID_EXPORT_PLY_FILE:                event.Enable(m_sfm_done);           break;
    case ID_EXPORT_MAYA_FILE:               event.Enable(m_sfm_done);           break;

    case ID_TOGGLE_FRUSTRUM_VISIBILITY:     event.Enable(m_sfm_done);           break;
    case ID_TOGGLE_POINTS_VISIBILITY:       event.Enable(m_sfm_done);           break;
    case ID_TOGGLE_CAMERAS_VISIBILITY:      event.Enable(m_sfm_done);           break;

    case ID_PANE_MATCHES:                   event.Enable(m_sfm_done);           break;
    default:                                event.Skip();
    }
}

void MainFrame::OnMenuExit(wxCommandEvent& event)
{
    this->Close();
}

void MainFrame::OnClose(wxCloseEvent& event)
{
    m_turntable_timer->Stop();
    m_reset_viewport_timer->Stop();

    this->Destroy();
}

void MainFrame::OnViewWindows(wxCommandEvent& event)
{
    int id = event.GetId();

    switch(id)
    {
    case ID_VIEW_IMAGE_BROWSER:
        m_mgr.GetPane("Image browser").Show();
        m_mgr.Update();
        break;
    case ID_VIEW_IMAGE_PREVIEW:
        m_mgr.GetPane("Image preview").Show();
        m_mgr.Update();
        break;
    case ID_VIEW_OPTIONS:
        m_mgr.GetPane("Options").Show();
        m_mgr.Update();
        break;
    case ID_VIEW_LOG:
        m_mgr.GetPane("Log").Show();
        m_mgr.Update();
        break;
    case ID_VIEW_ABOUT:
        m_mgr.GetPane("About").Show();
        m_mgr.Update();
        break;
    default: event.Skip();
    }
}

void MainFrame::OnResetOptions(wxCommandEvent& event)
{
    ResetOptions();
}

void MainFrame::OnReset3dViewport(wxCommandEvent& event)
{
    if (m_reset_viewport_timer->IsRunning()) m_reset_viewport_timer->Stop();
    else
    {
        m_counter = 0.0f;
        m_reset_viewport_timer->Start(10);
    }
}

void MainFrame::OnToggleTurntableAnimation(wxCommandEvent& event)
{
    if (!m_turntable_timer->IsRunning())    m_turntable_timer->Start(10);
    else                                    m_turntable_timer->Stop();

    m_gl_canvas->Refresh(false);
}

void MainFrame::OnToggleVisibility(wxCommandEvent& event)
{
    switch(event.GetId())
    {
    case ID_TOGGLE_TRACKBALL_VISIBILITY:
        if (m_scene->GetNode("Trackball X")->IsVisibleMesh() &&
            m_scene->GetNode("Trackball Y")->IsVisibleMesh() &&
            m_scene->GetNode("Trackball Z")->IsVisibleMesh())
        {
            m_scene->GetNode("Trackball X")->SetVisibilityMesh(false);
            m_scene->GetNode("Trackball Y")->SetVisibilityMesh(false);
            m_scene->GetNode("Trackball Z")->SetVisibilityMesh(false);
        } else
        {
            m_scene->GetNode("Trackball X")->SetVisibilityMesh(true);
            m_scene->GetNode("Trackball Y")->SetVisibilityMesh(true);
            m_scene->GetNode("Trackball Z")->SetVisibilityMesh(true);
        }
        break;
    case ID_TOGGLE_GRID_VISIBILITY:
        {
            auto grid = m_scene->GetNode("Grid");
            if (grid->IsVisibleMesh())  grid->SetVisibilityMesh(false);
            else                        grid->SetVisibilityMesh(true);
            break;
        }
    case ID_TOGGLE_CAMERAS_VISIBILITY:
        {
            for (const auto &image : m_images)
            {
                if (!image.m_camera.m_adjusted) continue;
                if (image.m_camera_mesh->IsVisibleMesh())   image.m_camera_mesh->SetVisibilityMesh(false);
                else                                        image.m_camera_mesh->SetVisibilityMesh(true);
            }
            break;
        }
    case ID_TOGGLE_POINTS_VISIBILITY:
        {
            auto points = m_scene->GetNode("Points");
            if (points->IsVisibleMesh())    points->SetVisibilityMesh(false);
            else                            points->SetVisibilityMesh(true);
            break;
        }
    default: return;
    }

    m_gl_canvas->Refresh(false);
}

void MainFrame::OnTimerUpdate(wxTimerEvent& event)
{
    switch (event.GetId())
    {
    case ID_TIMER_TURNTABLE:
        {
            auto camera     = m_scene->GetCamera();
            auto rotation   = camera->GetTrackballOrientation();

            camera->RotateTrackball(glm::vec2(0.0f, 0.0f), glm::vec2(-0.0002f * m_tb_turntable_speed_slider->GetValue(), 0.0f));

            m_scene->GetNode("Trackball X")->GetTransform().SetOrientation(rotation * glm::angleAxis(90.0f, glm::vec3(0.0f, 1.0f, 0.0f)));
            m_scene->GetNode("Trackball Y")->GetTransform().SetOrientation(rotation * glm::angleAxis(90.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
            m_scene->GetNode("Trackball Z")->GetTransform().SetOrientation(rotation);

            break;
        }
    case ID_TIMER_RESET_VIEWPORT:
        {
            m_counter += 0.02f;
            if (m_counter >= 1.0f) m_reset_viewport_timer->Stop();

            auto camera = m_scene->GetCamera();
            auto quat_x = glm::angleAxis(15.0f, glm::vec3(1, 0, 0));
            auto quat_y = glm::angleAxis(45.0f, glm::vec3(0, 1, 0));
            auto quat_z = glm::angleAxis(0.0f, glm::vec3(0, 0, 1));

            auto orientation_start  = camera->GetTrackballOrientation();
            auto orientation_target = quat_x * quat_y * quat_z;

            auto position_start     = camera->GetTrackballPosition();
            auto position_target    = glm::vec3(0.0f, 0.0f, 0.0f);

            auto zoom_start         = camera->GetTrackballZoom();
            auto zoom_target        = 1.0f;

            auto zoom_mix           = glm::lerp(zoom_start, zoom_target, m_counter);
            auto position_mix       = glm::lerp(position_start, position_target, m_counter);
            auto orientation_mix    = glm::shortMix(orientation_start, orientation_target, m_counter);

            camera->SetTrackballZoom(zoom_mix);
            camera->SetTrackballPosition(position_mix);
            camera->SetTrackballOrientation(orientation_mix);

            auto rotation = camera->GetTrackballOrientation();
            m_scene->GetNode("Trackball X")->GetTransform().SetOrientation(orientation_mix * glm::angleAxis(90.0f, glm::vec3(0.0f, 1.0f, 0.0f)));
            m_scene->GetNode("Trackball Y")->GetTransform().SetOrientation(orientation_mix * glm::angleAxis(90.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
            m_scene->GetNode("Trackball Z")->GetTransform().SetOrientation(orientation_mix);

            m_beginx = m_beginy = 0.0f;

            break;
        }
    default: event.Skip();
    }

    m_gl_canvas->Refresh(false);
}

void MainFrame::OnExport(wxCommandEvent& event)
{
    int id = event.GetId();

    switch(id)
    {
    case ID_EXPORT_TRACKS:
        {
            SaveTrackFile();
            break;
        }
    case ID_EXPORT_MATCHES:
        {
            SaveMatchFile();
            break;
        }
    case ID_EXPORT_CMVS:
        {
            ExportToCMVS();
            break;
        }
    case ID_EXPORT_BUNDLE_FILE:
        {
            SaveBundleFile(m_path);
            break;
        }
    case ID_EXPORT_PLY_FILE:
        {
            SavePlyFile();
            break;
        }
    case ID_EXPORT_MAYA_FILE:
        {
            SaveMayaFile();
            break;
        }
    default: event.Skip();
    }
}

void MainFrame::OnSaveLog(wxCommandEvent& event)
{
    auto target = m_path + "\\Log.txt";
    if (m_tc_log->SaveFile(target)) wxLogMessage("Log saved to %s", target);
}

void MainFrame::OnClearLog(wxCommandEvent& event)
{
    m_tc_log->Clear();
}

void MainFrame::OnSelectDirectory(wxFileDirPickerEvent& event)
{
    wxString filename, focalPx, res, path;
    path = m_dir_picker->GetPath();
    m_path = (path).ToStdString();
    wxDir dir(path);

    m_images.clear();

    m_cb_matches_left->Clear();
    m_cb_matches_right->Clear();
    m_window_image_preview->Refresh(true);
    m_pane_matches_view->Refresh(true);


    m_img_ctrl->ClearAll();
    m_img_ctrl->InsertColumn(0, "Name",			wxLIST_FORMAT_LEFT, 60);
    m_img_ctrl->InsertColumn(1, "Resolution",	wxLIST_FORMAT_LEFT, 75);
    m_img_ctrl->InsertColumn(2, "Focal",	    wxLIST_FORMAT_LEFT, 50);
    m_img_ctrl->InsertColumn(3, "Features",	    wxLIST_FORMAT_LEFT, 60);
    m_img_ctrl->InsertColumn(4, "k1",	        wxLIST_FORMAT_LEFT, 40);
    m_img_ctrl->InsertColumn(5, "k2",       	wxLIST_FORMAT_LEFT, 40);

    // Parse directory and process jpg images
    bool found = dir.GetFirst(&filename, "*.jpg", wxDIR_FILES);
    int index = 0;

    while (found)
    {
        // Get focal from EXIF tags, convert to px-coordinates and add image if successful
        if(AddImage(dir.FindFirst(path, filename).ToStdString(), filename.ToStdString()))
        {
            m_img_ctrl->InsertItem(index, filename);
            m_cb_matches_left->Append(filename);
            m_cb_matches_right->Append(filename);
        }

        // Get the next jpg in the directory
        found = dir.GetNext(&filename);
        index++;
    }

    // Display findings
    if(m_images.size() > 0)
    {
        for (int i = 0; i < (int)m_images.size(); i++)
        {
            res.Printf("%i x %i", GetImageWidth(i), GetImageHeight(i));
            focalPx.Printf("%.2f", GetFocalLength(i));
            m_img_ctrl->SetItem(i, 1, res, -1);
            m_img_ctrl->SetItem(i, 2, focalPx, -1);
        }

        m_has_images = true;
        wxLogMessage("%s: %i images ready for reconstruction", path, m_images.size());
    } else
    {
        wxLogMessage("No suitable images found in %s", path);
    }
}

void MainFrame::OnReconstruct(wxCommandEvent& event)
{
    // Detect features
    DetectFeaturesAll();

    // Match features
    if (m_options.feature_type != 2)    MatchAll();
    else                                MatchAllAkaze();

    // Compute structure from motion in another thread
    if (CreateThread(wxTHREAD_DETACHED) != wxTHREAD_NO_ERROR)
    {
        wxLogError("Error: Could not create the worker thread!");
        return;
    }

    if (GetThread()->Run() != wxTHREAD_NO_ERROR)
    {
        wxLogError("Error: Could not run the worker thread!");
        return;
    }
}
