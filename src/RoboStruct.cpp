#include "RoboStruct.hpp"
#include "MainFrame.hpp"

IMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{
    wxImage::AddHandler(new wxPNGHandler());
    wxImage::AddHandler(new wxJPEGHandler());

    MainFrame* frame = new MainFrame();

    frame->SetIcon(wxICON(aaaamain));
    frame->Maximize();
    frame->Show();

    return true;
}