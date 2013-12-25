#include "GL/glew.h"

#include "glm/gtx/quaternion.hpp"
#include "wx/dcclient.h"

#include "MainFrame.hpp"

void MainFrame::InitializeOpenGL()
{
    m_gl_context = new wxGLContext(m_gl_canvas);
    m_gl_canvas->SetCurrent(*m_gl_context);

    // Initialize GLEW
    GLenum result = glewInit();
    if (result != GLEW_OK)
    {
        wxLogMessage("ERROR: %s", wxString(glewGetErrorString(result)));
        this->Close();
    }

    // Display OpenGL info
    wxLogMessage("GLEW version: %s",	wxString(glewGetString(GLEW_VERSION)));
    wxLogMessage("OpenGL version: %s",	wxString(glGetString(GL_VERSION)));
    wxLogMessage("OpenGL vendor: %s",	wxString(glGetString(GL_VENDOR)));
    wxLogMessage("OpenGL renderer: %s",	wxString(glGetString(GL_RENDERER)));
    wxLogMessage("GLSL version: %s",	wxString(glGetString(GL_SHADING_LANGUAGE_VERSION)));
}

void MainFrame::InitializeScene()
{
    gly::Clock::Init();
    m_clock = gly::ClockPtr(new gly::Clock(0.0f));
    m_scene = gly::ScenePtr(new gly::Scene);

    m_scene->SetBackground(gly::BackgroundPtr(new gly::Background));
    m_scene->GetBackground()->SetType(gly::Background::BGR_GRADIENT);
    m_scene->GetBackground()->SetMesh(gly::CreatePlane());
    m_scene->GetBackground()->SetProgram(gly::LoadProgram("shaders\\Background.xml"));

    int width, height;
    m_gl_canvas->GetClientSize(&width, &height);
    m_scene->SetCamera(gly::CameraPtr(new gly::Camera(50, width, height, 1.0f, 1000.0f)));
    m_scene->GetCamera()->SetType(gly::Camera::CAM_TRACKBALL);

    m_scene->AddMesh("points_mesh",             std::unique_ptr<gly::Mesh>(new gly::Mesh(gly::Meshdescription(GL_POINTS, 0), gly::Meshdata())));
    m_scene->AddMesh("dslr_mesh",               gly::LoadMesh("meshes\\DSLR.ctm"));
    m_scene->AddMesh("grid_mesh",               gly::CreateGrid(21));
    m_scene->AddMesh("trackball_x_mesh",        gly::CreateCircle(100, glm::vec4(0.6f, 0.0f, 0.0f, 1.0f)));
    m_scene->AddMesh("trackball_y_mesh",        gly::CreateCircle(100, glm::vec4(0.0f, 0.6f, 0.0f, 1.0f)));
    m_scene->AddMesh("trackball_z_mesh",        gly::CreateCircle(100, glm::vec4(0.0f, 0.0f, 0.6f, 1.0f)));

    m_scene->AddProgram("points_program",       gly::LoadProgram("shaders\\Points.xml"));
    m_scene->AddProgram("cam_program",          gly::LoadProgram("shaders\\Camera.xml"));
    m_scene->AddProgram("grid_program",         gly::LoadProgram("shaders\\Grid.xml"));
    m_scene->AddProgram("trackball_program",    gly::LoadProgram("shaders\\Trackball.xml"));
    m_scene->AddProgram("tbn_program",          gly::LoadProgram("shaders\\TBNDisplay.xml"));
    m_scene->AddProgram("tbns_program",         gly::LoadProgram("shaders\\TBNDisplaySkinned.xml"));

    m_scene->AddTexture("dslr_diffuse",         gly::LoadTexture("textures\\DSLR_diffuse.dds"));
    m_scene->AddTexture("dslr_normal",          gly::LoadTexture("textures\\DSLR_normal.dds"));
    m_scene->AddTexture("dslr_specular",        gly::LoadTexture("textures\\DSLR_specular.dds"));
    m_scene->AddTexture("dslr_cube",            gly::LoadTexture("textures\\Cube_reflection.dds"));

    m_scene->MakeNode("Points",                 "points_mesh",      "points_program");
    m_scene->MakeNode("Grid",                   "grid_mesh",        "grid_program");
    m_scene->MakeNode("Trackball X",            "trackball_x_mesh", "trackball_program");
    m_scene->MakeNode("Trackball Y",            "trackball_y_mesh", "trackball_program");
    m_scene->MakeNode("Trackball Z",            "trackball_z_mesh", "trackball_program");

    m_scene->GetNode("Grid")->GetTransform().Scale(glm::vec3(20.0f, 20.0f, 20.0f));
    m_scene->GetNode("Trackball X")->GetTransform().Translate(glm::vec3(0.0f, 0.0f, -25.0f));
    m_scene->GetNode("Trackball Y")->GetTransform().Translate(glm::vec3(0.0f, 0.0f, -25.0f));
    m_scene->GetNode("Trackball Z")->GetTransform().Translate(glm::vec3(0.0f, 0.0f, -25.0f));
    m_scene->GetNode("Trackball X")->GetTransform().Scale(glm::vec3(14.0f, 14.0f, 14.0f));
    m_scene->GetNode("Trackball Y")->GetTransform().Scale(glm::vec3(14.0f, 14.0f, 14.0f));
    m_scene->GetNode("Trackball Z")->GetTransform().Scale(glm::vec3(14.0f, 14.0f, 14.0f));
}

void MainFrame::ResetGLCanvas()
{
    auto camera = m_scene->GetCamera();
    camera->Reset();
    camera->SetTrackballAngles(15.0f, 45.0f, 0.0f);

    auto rotation = camera->GetTrackballOrientation();
    m_scene->GetNode("Trackball X")->GetTransform().SetOrientation(rotation * glm::angleAxis(90.0f, glm::vec3(0.0f, 1.0f, 0.0f)));
    m_scene->GetNode("Trackball Y")->GetTransform().SetOrientation(rotation * glm::angleAxis(90.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
    m_scene->GetNode("Trackball Z")->GetTransform().SetOrientation(rotation);

    m_beginx = m_beginy = 0.0f;

    ResetPerspectiveMatrix();
}

void MainFrame::ResetPerspectiveMatrix()
{
    int width, height;
    m_gl_canvas->GetClientSize(&width, &height);
    m_scene->GetCamera()->SetPerspective(50.0f, width, height);

    m_gl_canvas->Refresh(false);
}

void MainFrame::OnGLCanvasPaint(wxPaintEvent& event)
{
    // Must always be here
    wxPaintDC dc(m_gl_canvas);

    auto start = m_clock->Now();
    m_scene->Render();
    m_gl_canvas->SwapBuffers();
    auto end = m_clock->Now();

    wxString msg;
    msg.Printf("SPF: %.3f ms", gly::Clock::CyclesToSeconds(end - start) * 1000.0f);
    m_statusbar->SetStatusText(msg, 2);
}

void MainFrame::OnGLCanvasMouse(wxMouseEvent& event)
{
    // Set focus, otherwise mousewheel zoom not working!
    m_gl_canvas->SetFocus();

    // Disable viewport navigation in turntable mode
    if (m_turntable_timer->IsRunning()) return;

    wxSize sz(m_gl_canvas->GetClientSize());
    float distx = m_beginx - event.GetX();
    float disty = m_beginy - event.GetY();

    auto camera = m_scene->GetCamera();

    // Rotate
    if (event.m_leftDown)
    {
        m_gl_canvas->SetCursor(m_rotate_cursor);
        camera->RotateTrackball(    glm::vec2(0.8f * (2.0f * m_beginx - sz.x)     / sz.y, 0.8f * (sz.y - 2.0f * m_beginy)     / sz.y),
                                    glm::vec2(0.8f * (2.0f * event.GetX() - sz.x) / sz.y, 0.8f * (sz.y - 2.0f * event.GetY()) / sz.y));
    }

    // Pan
    if (event.m_middleDown)
    {
        m_gl_canvas->SetCursor(m_pan_cursor);
        camera->PanTrackball(glm::vec2(-distx / (sz.y * 0.044f), disty / (sz.y * 0.044f)));
    }

    // Zoom right mouse button
    if (event.m_rightDown)
    {
        if (event.ControlDown())
        {
            m_options.point_size += disty * 0.03f;
            if (m_options.point_size < 0.01f) m_options.point_size = 0.01f;
            if (m_options.point_size > 4.0f)  m_options.point_size = 4.0f;
        } else if (event.ShiftDown())
        {
            m_options.camera_size += disty * 0.001f;
            if (m_options.camera_size < 0.01f) m_options.camera_size = 0.01f;
            if (m_options.camera_size > 1.0f)  m_options.camera_size = 1.0f;
        } else
        {
            m_gl_canvas->SetCursor(m_zoom_cursor);
            camera->ZoomTrackball(disty * 2.0f / sz.y);
        }
    }

    // Zoom mouse wheel
    if (event.m_wheelRotation)
    {
        if (event.ControlDown())
        {
            m_options.point_size += event.m_wheelRotation * 0.008f;
            if (m_options.point_size < 0.01f) m_options.point_size = 0.01f;
            if (m_options.point_size > 4.0f)  m_options.point_size = 4.0f;
        } else if (event.ShiftDown())
        {
            m_options.camera_size += event.m_wheelRotation * 0.00025f;
            if (m_options.camera_size < 0.01f) m_options.camera_size = 0.01f;
            if (m_options.camera_size > 1.0f)  m_options.camera_size = 1.0f;
        } else
        {
            camera->ZoomTrackball(event.m_wheelRotation * 0.4f / sz.y);
        }
    }

    auto rotation = camera->GetTrackballOrientation();
    m_scene->GetNode("Trackball X")->GetTransform().SetOrientation(rotation * glm::angleAxis(90.0f, glm::vec3(0.0f, 1.0f, 0.0f)));
    m_scene->GetNode("Trackball Y")->GetTransform().SetOrientation(rotation * glm::angleAxis(90.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
    m_scene->GetNode("Trackball Z")->GetTransform().SetOrientation(rotation);

    // Reset cursor icon
    if (event.LeftUp() || event.MiddleUp() || event.RightUp() || event.Leaving()) m_gl_canvas->SetCursor(wxCURSOR_ARROW);

    m_beginx = event.GetX();
    m_beginy = event.GetY();

    m_gl_canvas->Refresh(false);
}

void MainFrame::OnGLCanvasSize(wxSizeEvent& event)
{
    // This is necessary to update the context on some platforms
    m_gl_canvas->OnSize(event);

    // Reset the OpenGL view aspect
    ResetPerspectiveMatrix();
}

void MainFrame::OnGLCanvasEraseBackground(wxEraseEvent& event)
{
    // Do nothing, to avoid flashing on MSW
}
