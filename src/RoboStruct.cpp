#include "RoboStruct.hpp"
#include "MainFrame.hpp"

IMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{
    wxImage::AddHandler(new wxPNGHandler{});
    wxImage::AddHandler(new wxJPEGHandler{});

    auto frame = new MainFrame{};

    frame->InitializeLog();
    frame->InitializeOpenGL();
    frame->InitializeScene();
    frame->InitializeCameraDatabase();
    frame->ResetGLCanvas();
    frame->ResetOptions();
    frame->SetIcon(wxICON(aaaamain));
    frame->InitializeGuiStyle();
    frame->Maximize();
    frame->Show();

    return true;
}