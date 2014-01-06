# pragma once

#include "wx/aui/framemanager.h"
#include "wx/aui/dockart.h"
#include "wx/aui/auibook.h"
#include "wx/aui/auibar.h"

const wxColour light_theme_main_color{238, 238, 242};

wxString wxAuiChopText(wxDC& dc, const wxString& text, int max_size);

class wxAuiSolidToolBarArt : public wxAuiDefaultToolBarArt
{
public:
    wxAuiSolidToolBarArt() = default;
    ~wxAuiSolidToolBarArt() {}

    wxAuiToolBarArt *Clone();

    void DrawBackground(wxDC& dc, wxWindow* wnd, const wxRect& rect);
};

class wxAuiSolidTabArt : public wxAuiGenericTabArt
{
public:
    wxAuiSolidTabArt();
    ~wxAuiSolidTabArt() {}

    wxAuiSolidTabArt *Clone();

    void DrawBackground(wxDC& dc, wxWindow* wnd, const wxRect& rect);

    void DrawTab(wxDC& dc, wxWindow* wnd, const wxAuiNotebookPage& page, const wxRect& in_rect,
                 int close_button_state, wxRect* out_tab_rect, wxRect* out_button_rect, int* x_extent);

    int GetBestTabCtrlSize(wxWindow* wnd, const wxAuiNotebookPageArray& pages, const wxSize& requiredBmp_size);
};

// Deriving my own class from wxAuiNotebook to get rid of the border, see http://forums.wxwidgets.org/viewtopic.php?t=30277
class wxAuiMyNotebook : public wxAuiNotebook
{
public:
    wxAuiMyNotebook(wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize,
                    long style = wxAUI_NB_DEFAULT_STYLE);
    ~wxAuiMyNotebook() {}
};