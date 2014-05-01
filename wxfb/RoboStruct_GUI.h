///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Feb 26 2014)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __ROBOSTRUCT_GUI_H__
#define __ROBOSTRUCT_GUI_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
class wxAuiMyNotebook;

#include <wx/statusbr.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/aui/aui.h>
#include <wx/aui/auibar.h>
#include <wx/glcanvas.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/combobox.h>
#include <wx/aui/auibook.h>
#include <wx/filepicker.h>
#include <wx/listctrl.h>
#include "wx/propgrid/propgrid.h"
#include "wx/propgrid/advprops.h"
#include <wx/textctrl.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// Class MainFrame_base
///////////////////////////////////////////////////////////////////////////////
class MainFrame_base : public wxFrame 
{
	private:
	
	protected:
		enum
		{
			ID_MENU_EXIT = 1000,
			ID_RECONSTRUCT,
			ID_VIEW_IMAGE_BROWSER,
			ID_VIEW_IMAGE_PREVIEW,
			ID_VIEW_OPTIONS,
			ID_VIEW_LOG,
			ID_RESET_3D_VIEWPORT,
			ID_TOGGLE_TURNTABLE_ANIMATION,
			ID_TOGGLE_TRACKBALL_VISIBILITY,
			ID_TOGGLE_GRID_VISIBILITY,
			ID_TOGGLE_POINTS_VISIBILITY,
			ID_TOGGLE_CAMERAS_VISIBILITY,
			ID_EXPORT_TRACKS,
			ID_EXPORT_MATCHES,
			ID_EXPORT_CMVS,
			ID_EXPORT_BUNDLE_FILE,
			ID_EXPORT_PLY_FILE,
			ID_EXPORT_MESHLAB_FILE,
			ID_EXPORT_MAYA_FILE,
			ID_VIEW_ABOUT,
			ID_PANE_MATCHES,
			ID_RESET_OPTIONS,
			ID_SAVE_OPTIONS,
			ID_SAVE_LOG,
			ID_CLEAR_LOG
		};
		
		wxStatusBar* m_statusbar;
		wxMenuBar* m_menubar;
		wxMenu* m_file_menu;
		wxMenu* m_process_menu;
		wxMenu* m_view_menu;
		wxMenu* m_export_menu;
		wxMenu* m_help_menu;
		wxAuiToolBar* m_toolbar;
		wxAuiMyNotebook* m_window_viewport;
		wxPanel* m_panel8;
		wxAuiToolBar* m_toolbar_viewport;
		wxGLCanvas *m_gl_canvas;
		wxPanel* m_pane_matches;
		wxPanel* m_pane_matches_view;
		wxComboBox* m_cb_matches_left;
		wxComboBox* m_cb_matches_right;
		wxPanel* m_window_image_browser;
		wxDirPickerCtrl* m_dir_picker;
		wxListCtrl* m_img_ctrl;
		wxPanel* m_window_image_preview;
		wxPanel* m_window_options;
		wxAuiToolBar* m_toolbar_options;
		wxPropertyGrid* m_pg_options;
		
		wxPanel* m_window_log;
		wxTextCtrl* m_tc_log;
		wxAuiToolBar* m_toolbar_log;
		wxPanel* m_window_about;
		wxTextCtrl* m_tc_about;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnUpdateUI( wxUpdateUIEvent& event ) { event.Skip(); }
		virtual void OnMenuExit( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnReconstruct( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnViewWindows( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnReset3dViewport( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnToggleTurntableAnimation( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnToggleVisibility( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnExport( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnGLCanvasEraseBackground( wxEraseEvent& event ) { event.Skip(); }
		virtual void OnGLCanvasMouse( wxMouseEvent& event ) { event.Skip(); }
		virtual void OnGLCanvasPaint( wxPaintEvent& event ) { event.Skip(); }
		virtual void OnGLCanvasSize( wxSizeEvent& event ) { event.Skip(); }
		virtual void OnMatchesViewPaint( wxPaintEvent& event ) { event.Skip(); }
		virtual void OnMatchesViewResize( wxSizeEvent& event ) { event.Skip(); }
		virtual void OnSelectMatchImage( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectDirectory( wxFileDirPickerEvent& event ) { event.Skip(); }
		virtual void OnSelectPreviewImage( wxListEvent& event ) { event.Skip(); }
		virtual void OnImagePreviewMouse( wxMouseEvent& event ) { event.Skip(); }
		virtual void OnImagePreviewPaint( wxPaintEvent& event ) { event.Skip(); }
		virtual void OnImagePreviewResize( wxSizeEvent& event ) { event.Skip(); }
		virtual void OnResetOptions( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSaveLog( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnClearLog( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		MainFrame_base( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("RoboStruct"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 950,780 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		wxAuiManager m_mgr;
		
		~MainFrame_base();
	
};

#endif //__ROBOSTRUCT_GUI_H__
