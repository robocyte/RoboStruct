#include "wx/dc.h"
#include "wx/dcclient.h"

#include "RoboStruct_GUI_style.hpp"
#include "MainFrame.hpp"

wxAuiToolBarArt* wxAuiSolidToolBarArt::Clone()
{
    return new wxAuiSolidToolBarArt{};
}

void wxAuiSolidToolBarArt::DrawBackground(wxDC& dc, wxWindow* wnd, const wxRect& rect)
{
    wxUnusedVar(wnd);
    dc.GradientFillLinear(rect, wxColour{238, 238, 242}, wxColour{238, 238, 242});
}



wxAuiSolidTabArt::wxAuiSolidTabArt()
    : wxAuiGenericTabArt()
{
    m_normalFont    = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    m_selectedFont  = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    m_selectedFont.SetWeight(wxNORMAL);
    m_measuringFont = m_selectedFont;
}

wxAuiSolidTabArt* wxAuiSolidTabArt::Clone()
{
    return new wxAuiSolidTabArt{};
}

void wxAuiSolidTabArt::DrawBackground(wxDC& dc, wxWindow* wnd, const wxRect& rect)
{
    wxUnusedVar(wnd);
    dc.GradientFillLinear(rect, wxColour{238, 238, 242}, wxColour{238, 238, 242});
}

void wxAuiSolidTabArt::DrawTab(wxDC& dc, wxWindow* wnd, const wxAuiNotebookPage& page, const wxRect& in_rect,
                int close_button_state, wxRect* out_tab_rect, wxRect* out_button_rect, int* x_extent)
{
    wxCoord normal_textx, normal_texty;

    // If the caption is empty, measure some temporary text
    wxString caption{page.caption};
    if (caption.empty()) caption = "Xj";

    dc.SetFont(m_normalFont);
    dc.GetTextExtent(caption, &normal_textx, &normal_texty);

    // Figure out the size of the tab
    wxSize  tab_size   = GetTabSize(dc, wnd, page.caption, page.bitmap, page.active, close_button_state, x_extent);
    wxCoord tab_height = m_tabCtrlHeight;
    wxCoord tab_width  = tab_size.x;
    wxCoord tab_x      = in_rect.x;
    wxCoord tab_y      = in_rect.y + in_rect.height - tab_height;

    caption = page.caption;

    dc.SetFont(m_normalFont);

    int drawn_tab_height = tab_height - 2;

    if (page.active)
    {
        // draw base background color
        wxRect r{tab_x, tab_y, tab_width, tab_height};
        dc.GradientFillLinear(r, wxColour{51, 153, 255}, wxColour{51, 153, 255});
    } else
    {
        wxRect r{tab_x, tab_y, tab_width, tab_height};
        dc.GradientFillLinear(r, wxColour{204, 206, 219}, wxColour{204, 206, 219});
    }

    int text_offset = tab_x + 8;

    // Draw tab text
    wxString draw_text = wxAuiChopText(dc, caption, tab_width - (text_offset-tab_x));
    if (page.active)
    {
        dc.SetTextForeground(wxColour{255, 255, 255});
        dc.DrawText(draw_text, text_offset, tab_y + 2 + (drawn_tab_height) / 2 - (normal_texty / 2) - 1);
    } else
    {
        dc.SetTextForeground(wxColour{0, 0, 0});
        dc.DrawText(draw_text, text_offset, tab_y + 2 + (drawn_tab_height) / 2 - (normal_texty / 2) - 1);
    }

    *out_tab_rect = wxRect{tab_x, tab_y, tab_width, tab_height};
}

int wxAuiSolidTabArt::GetBestTabCtrlSize(wxWindow* wnd, const wxAuiNotebookPageArray& pages, const wxSize& requiredBmp_size)
{
    return 22;
}



wxAuiMyNotebook::wxAuiMyNotebook(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : wxAuiNotebook(parent, id, pos, size, style)
{
    SetArtProvider(new wxAuiSolidTabArt{});
        
    m_mgr.GetArtProvider()->SetMetric(wxAUI_DOCKART_PANE_BORDER_SIZE,0);
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_BORDER_COLOUR, wxColour{238, 238, 242});
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_SASH_COLOUR, wxColour{238, 238, 242});
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_BACKGROUND_COLOUR, wxColour{238, 238, 242});
}



void MainFrame::InitializeGuiStyle()
{
    m_toolbar->SetArtProvider(new wxAuiSolidToolBarArt{});
    m_toolbar_log->SetArtProvider(new wxAuiSolidToolBarArt{});
    m_toolbar_options->SetArtProvider(new wxAuiSolidToolBarArt{});

    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_BORDER_COLOUR, light_theme_main_color);
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_SASH_COLOUR, light_theme_main_color);
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_BACKGROUND_COLOUR, light_theme_main_color);
    m_mgr.GetArtProvider()->SetMetric(wxAUI_DOCKART_PANE_BORDER_SIZE, 0);

    m_mgr.GetArtProvider()->SetFont(wxAUI_DOCKART_CAPTION_FONT, wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT));
    m_mgr.GetArtProvider()->SetMetric(wxAUI_DOCKART_CAPTION_SIZE, 19);
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_ACTIVE_CAPTION_COLOUR, wxColour{0, 122, 204});
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_ACTIVE_CAPTION_GRADIENT_COLOUR, wxColour{0, 122, 204});
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_ACTIVE_CAPTION_TEXT_COLOUR, wxColour{255, 255, 255});
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_INACTIVE_CAPTION_COLOUR, light_theme_main_color);
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_INACTIVE_CAPTION_GRADIENT_COLOUR, light_theme_main_color);
    m_mgr.GetArtProvider()->SetColor(wxAUI_DOCKART_INACTIVE_CAPTION_TEXT_COLOUR, wxColour{20, 20, 20});
    m_mgr.Update();
}
