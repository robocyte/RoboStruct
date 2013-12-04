﻿///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 10 2012)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "RoboStruct_GUI.h"

///////////////////////////////////////////////////////////////////////////

MainFrame_base::MainFrame_base( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	m_mgr.SetManagedWindow(this);
	m_mgr.SetFlags(wxAUI_MGR_ALLOW_FLOATING|wxAUI_MGR_HINT_FADE|wxAUI_MGR_NO_VENETIAN_BLINDS_FADE|wxAUI_MGR_TRANSPARENT_DRAG|wxAUI_MGR_TRANSPARENT_HINT);
	
	m_statusbar = this->CreateStatusBar( 4, wxST_SIZEGRIP, wxID_ANY );
	m_menubar = new wxMenuBar( 0 );
	m_file_menu = new wxMenu();
	wxMenuItem* m_menu_exit;
	m_menu_exit = new wxMenuItem( m_file_menu, ID_MENU_EXIT, wxString( wxT("E&xit") ) + wxT('\t') + wxT("Alt+F4"), wxT("Exit the application"), wxITEM_NORMAL );
	m_file_menu->Append( m_menu_exit );
	
	m_menubar->Append( m_file_menu, wxT("&File") ); 
	
	m_process_menu = new wxMenu();
	wxMenuItem* m_menu_reconstruct;
	m_menu_reconstruct = new wxMenuItem( m_process_menu, ID_RECONSTRUCT, wxString( wxT("&Reconstruct") ) + wxT('\t') + wxT("Alt+r"), wxT("Compute 3d geometry from images"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_reconstruct->SetBitmaps( wxICON( start_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_reconstruct->SetBitmap( wxICON( start_icon ) );
	#endif
	m_process_menu->Append( m_menu_reconstruct );
	
	m_menubar->Append( m_process_menu, wxT("&Process") ); 
	
	m_view_menu = new wxMenu();
	wxMenuItem* m_menu_view_image_browser;
	m_menu_view_image_browser = new wxMenuItem( m_view_menu, ID_VIEW_IMAGE_BROWSER, wxString( wxT("&Image browser") ) + wxT('\t') + wxT("Alt+i"), wxT("Show image browser window"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_view_image_browser->SetBitmaps( wxICON( image_browser_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_view_image_browser->SetBitmap( wxICON( image_browser_icon ) );
	#endif
	m_view_menu->Append( m_menu_view_image_browser );
	
	wxMenuItem* m_menu_view_image_preview;
	m_menu_view_image_preview = new wxMenuItem( m_view_menu, ID_VIEW_IMAGE_PREVIEW, wxString( wxT("Image &preview") ) + wxT('\t') + wxT("Alt+p"), wxT("Show image preview window"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_view_image_preview->SetBitmaps( wxICON( image_preview_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_view_image_preview->SetBitmap( wxICON( image_preview_icon ) );
	#endif
	m_view_menu->Append( m_menu_view_image_preview );
	
	wxMenuItem* m_menu_view_options;
	m_menu_view_options = new wxMenuItem( m_view_menu, ID_VIEW_OPTIONS, wxString( wxT("&Options") ) + wxT('\t') + wxT("Alt+o"), wxT("Show options window"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_view_options->SetBitmaps( wxICON( options_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_view_options->SetBitmap( wxICON( options_icon ) );
	#endif
	m_view_menu->Append( m_menu_view_options );
	
	wxMenuItem* m_menu_view_log;
	m_menu_view_log = new wxMenuItem( m_view_menu, ID_VIEW_LOG, wxString( wxT("&Log") ) + wxT('\t') + wxT("Alt+l"), wxT("Show log window"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_view_log->SetBitmaps( wxICON( log_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_view_log->SetBitmap( wxICON( log_icon ) );
	#endif
	m_view_menu->Append( m_menu_view_log );
	
	m_view_menu->AppendSeparator();
	
	wxMenuItem* m_menu_reset_3d_viewport;
	m_menu_reset_3d_viewport = new wxMenuItem( m_view_menu, ID_RESET_3D_VIEWPORT, wxString( wxT("Reset 3d viewport") ) + wxT('\t') + wxT("Alt+h"), wxT("Reset the 3d viewport"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_reset_3d_viewport->SetBitmaps( wxICON( home_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_reset_3d_viewport->SetBitmap( wxICON( home_icon ) );
	#endif
	m_view_menu->Append( m_menu_reset_3d_viewport );
	
	wxMenuItem* m_menu_toggle_turntable_animation;
	m_menu_toggle_turntable_animation = new wxMenuItem( m_view_menu, ID_TOGGLE_TURNTABLE_ANIMATION, wxString( wxT("Tur&ntable") ) + wxT('\t') + wxT("Alt+n"), wxT("Toggle turntable animation"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_toggle_turntable_animation->SetBitmaps( wxICON( turntable_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_toggle_turntable_animation->SetBitmap( wxICON( turntable_icon ) );
	#endif
	m_view_menu->Append( m_menu_toggle_turntable_animation );
	
	m_view_menu->AppendSeparator();
	
	wxMenuItem* m_menu_toggle_trackball_visibility;
	m_menu_toggle_trackball_visibility = new wxMenuItem( m_view_menu, ID_TOGGLE_TRACKBALL_VISIBILITY, wxString( wxT("Trackball") ) + wxT('\t') + wxT("Alt+v"), wxT("Toggle trackball visibility"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_toggle_trackball_visibility->SetBitmaps( wxICON( trackball_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_toggle_trackball_visibility->SetBitmap( wxICON( trackball_icon ) );
	#endif
	m_view_menu->Append( m_menu_toggle_trackball_visibility );
	
	wxMenuItem* m_menu_toggle_grid_visibility;
	m_menu_toggle_grid_visibility = new wxMenuItem( m_view_menu, ID_TOGGLE_GRID_VISIBILITY, wxString( wxT("&Grid") ) + wxT('\t') + wxT("Alt+g"), wxT("Toggle grid visibility"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_toggle_grid_visibility->SetBitmaps( wxICON( grid_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_toggle_grid_visibility->SetBitmap( wxICON( grid_icon ) );
	#endif
	m_view_menu->Append( m_menu_toggle_grid_visibility );
	
	wxMenuItem* m_menu_toggle_points_visibility;
	m_menu_toggle_points_visibility = new wxMenuItem( m_view_menu, ID_TOGGLE_POINTS_VISIBILITY, wxString( wxT("Points") ) , wxT("Toggle points visibility"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_toggle_points_visibility->SetBitmaps( wxICON( points_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_toggle_points_visibility->SetBitmap( wxICON( points_icon ) );
	#endif
	m_view_menu->Append( m_menu_toggle_points_visibility );
	
	wxMenuItem* m_menu_toggle_cameras_visibility;
	m_menu_toggle_cameras_visibility = new wxMenuItem( m_view_menu, ID_TOGGLE_CAMERAS_VISIBILITY, wxString( wxT("Cameras") ) , wxT("Toggle cameras visibility"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_toggle_cameras_visibility->SetBitmaps( wxICON( camera_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_toggle_cameras_visibility->SetBitmap( wxICON( camera_icon ) );
	#endif
	m_view_menu->Append( m_menu_toggle_cameras_visibility );
	
	wxMenuItem* m_menu_toggle_frustrum_visibility;
	m_menu_toggle_frustrum_visibility = new wxMenuItem( m_view_menu, ID_TOGGLE_FRUSTRUM_VISIBILITY, wxString( wxT("&Frustrum") ) + wxT('\t') + wxT("Alt+f"), wxT("Toggle frustrum visibility"), wxITEM_NORMAL );
	#ifdef __WXMSW__
	m_menu_toggle_frustrum_visibility->SetBitmaps( wxICON( frustrum_icon ) );
	#elif defined( __WXGTK__ )
	m_menu_toggle_frustrum_visibility->SetBitmap( wxICON( frustrum_icon ) );
	#endif
	m_view_menu->Append( m_menu_toggle_frustrum_visibility );
	
	m_menubar->Append( m_view_menu, wxT("&View") ); 
	
	m_export_menu = new wxMenu();
	wxMenuItem* m_menu_export_tracks;
	m_menu_export_tracks = new wxMenuItem( m_export_menu, ID_EXPORT_TRACKS, wxString( wxT("Tracks") ) , wxT("Export tracks to textfile"), wxITEM_NORMAL );
	m_export_menu->Append( m_menu_export_tracks );
	
	wxMenuItem* m_menu_export_matches;
	m_menu_export_matches = new wxMenuItem( m_export_menu, ID_EXPORT_MATCHES, wxString( wxT("Matches") ) , wxT("Export matches to textfile"), wxITEM_NORMAL );
	m_export_menu->Append( m_menu_export_matches );
	
	m_export_menu->AppendSeparator();
	
	wxMenuItem* m_menu_export_cmvs;
	m_menu_export_cmvs = new wxMenuItem( m_export_menu, ID_EXPORT_CMVS, wxString( wxT("CMVS") ) , wxT("Export to CMVS"), wxITEM_NORMAL );
	m_export_menu->Append( m_menu_export_cmvs );
	
	wxMenuItem* m_menu_export_bundle_file;
	m_menu_export_bundle_file = new wxMenuItem( m_export_menu, ID_EXPORT_BUNDLE_FILE, wxString( wxT("Bundle file") ) , wxT("Export to bundle.rd.out"), wxITEM_NORMAL );
	m_export_menu->Append( m_menu_export_bundle_file );
	
	wxMenuItem* m_menu_export_ply_file;
	m_menu_export_ply_file = new wxMenuItem( m_export_menu, ID_EXPORT_PLY_FILE, wxString( wxT("Ply file") ) , wxT("Export reconstruction as .ply file"), wxITEM_NORMAL );
	m_export_menu->Append( m_menu_export_ply_file );
	
	wxMenuItem* m_menu_export_maya;
	m_menu_export_maya = new wxMenuItem( m_export_menu, ID_EXPORT_MAYA_FILE, wxString( wxT("Maya") ) , wxT("Export reconstruction as .ma file"), wxITEM_NORMAL );
	m_export_menu->Append( m_menu_export_maya );
	
	m_menubar->Append( m_export_menu, wxT("&Export") ); 
	
	m_help_menu = new wxMenu();
	wxMenuItem* m_menu_about;
	m_menu_about = new wxMenuItem( m_help_menu, ID_VIEW_ABOUT, wxString( wxT("About RoboStruct...") ) + wxT('\t') + wxT("F1"), wxT("Show info about this application"), wxITEM_NORMAL );
	m_help_menu->Append( m_menu_about );
	
	m_menubar->Append( m_help_menu, wxT("&Help") ); 
	
	this->SetMenuBar( m_menubar );
	
	m_toolbar = new wxAuiToolBar( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_TB_HORZ_LAYOUT|wxNO_BORDER ); 
	m_toolbar->AddTool( ID_RECONSTRUCT, wxT("Reconstruct"), wxICON( start_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Compute 3d geometry from images"), wxT("Compute 3d geometry from images"), NULL ); 
	
	m_toolbar->AddSeparator(); 
	
	m_toolbar->AddTool( ID_RESET_3D_VIEWPORT, wxT("Reset 3d viewport"), wxICON( home_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Reset the 3d viewport"), wxT("Reset the 3d viewport"), NULL ); 
	
	m_toolbar->AddTool( ID_TOGGLE_TURNTABLE_ANIMATION, wxT("Turntable"), wxICON( turntable_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Toggle turntable animation"), wxT("Toggle turntable animation"), NULL ); 
	
	m_tb_turntable_speed_slider = new wxSlider( m_toolbar, ID_TURNTABLE_SPEED_SLIDER, 50, 1, 100, wxDefaultPosition, wxDefaultSize, 0|wxSTATIC_BORDER );
	m_tb_turntable_speed_slider->SetToolTip( wxT("Set turntable animation speed") );
	
	m_toolbar->AddControl( m_tb_turntable_speed_slider );
	m_toolbar->AddSeparator(); 
	
	m_toolbar->AddTool( ID_TOGGLE_TRACKBALL_VISIBILITY, wxT("Trackball"), wxICON( trackball_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Toggle trackball visibility"), wxT("Toggle trackball visibility"), NULL ); 
	
	m_toolbar->AddTool( ID_TOGGLE_GRID_VISIBILITY, wxT("Grid"), wxICON( grid_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Toggle grid visibility"), wxT("Toggle grid visibility"), NULL ); 
	
	m_toolbar->AddTool( ID_TOGGLE_POINTS_VISIBILITY, wxT("Points"), wxICON( points_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Toggle points visibility"), wxT("Toggle points visibility"), NULL ); 
	
	m_toolbar->AddTool( ID_TOGGLE_CAMERAS_VISIBILITY, wxT("Cameras"), wxICON( camera_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Toggle cameras visibility"), wxT("Toggle cameras visibility"), NULL ); 
	
	m_toolbar->AddTool( ID_TOGGLE_FRUSTRUM_VISIBILITY, wxT("Frustrum"), wxICON( frustrum_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Toggle frustrum visibility"), wxT("Toggle frustrum visibility"), NULL ); 
	
	m_toolbar->AddSeparator(); 
	
	wxString m_tb_camera_mesh_choiceChoices[] = { wxT("DSLR"), wxT("Compact"), wxT("Box") };
	int m_tb_camera_mesh_choiceNChoices = sizeof( m_tb_camera_mesh_choiceChoices ) / sizeof( wxString );
	m_tb_camera_mesh_choice = new wxChoice( m_toolbar, ID_CAMERA_MESH_CHOICE, wxDefaultPosition, wxDefaultSize, m_tb_camera_mesh_choiceNChoices, m_tb_camera_mesh_choiceChoices, 0 );
	m_tb_camera_mesh_choice->SetSelection( 0 );
	m_tb_camera_mesh_choice->SetToolTip( wxT("Select camera mesh") );
	
	m_toolbar->AddControl( m_tb_camera_mesh_choice );
	m_toolbar->AddSeparator(); 
	
	m_toolbar->AddTool( ID_VIEW_IMAGE_BROWSER, wxT("Browser"), wxICON( image_browser_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Show image browser window"), wxT("Show image browser window"), NULL ); 
	
	m_toolbar->AddTool( ID_VIEW_IMAGE_PREVIEW, wxT("Preview"), wxICON( image_preview_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Show image preview window"), wxT("Show image preview window"), NULL ); 
	
	m_toolbar->AddTool( ID_VIEW_OPTIONS, wxT("Options"), wxICON( options_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Show options window"), wxT("Show options window"), NULL ); 
	
	m_toolbar->AddTool( ID_VIEW_LOG, wxT("Log"), wxICON( log_icon ), wxNullBitmap, wxITEM_NORMAL, wxT("Show log window"), wxT("Show log window"), NULL ); 
	
	m_toolbar->Realize();
	m_mgr.AddPane( m_toolbar, wxAuiPaneInfo().Top().CaptionVisible( false ).CloseButton( false ).PaneBorder( false ).Movable( false ).Dock().Resizable().FloatingSize( wxSize( -1,-1 ) ).BottomDockable( false ).TopDockable( false ).LeftDockable( false ).RightDockable( false ).Floatable( false ).Layer( 10 ).ToolbarPane() );
	
	m_window_viewport = new wxAuiNotebook(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_NB_TAB_EXTERNAL_MOVE | wxAUI_NB_TAB_SPLIT | wxNO_BORDER );
	m_mgr.AddPane(m_window_viewport, wxAuiPaneInfo().CentrePane().Name("Viewport").Caption("Viewport").CaptionVisible(true).MaximizeButton(true).MinimizeButton(false).PinButton(false).CloseButton(false));
	
	m_panel8 = new wxPanel( m_window_viewport, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	int GLCanvasAttributes[] = {WX_GL_RGBA, 1, WX_GL_DOUBLEBUFFER, 1, WX_GL_SAMPLES, 8, 0};
	m_gl_canvas = new wxGLCanvas(m_panel8, -1, GLCanvasAttributes);
	m_gl_canvas->SetBackgroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_BTNSHADOW ) );
	
	bSizer8->Add( m_gl_canvas, 1, wxEXPAND, 5 );
	
	
	m_panel8->SetSizer( bSizer8 );
	m_panel8->Layout();
	bSizer8->Fit( m_panel8 );
	m_window_viewport->AddPage( m_panel8, wxT("3d view"), true, wxNullBitmap );
	m_pane_matches = new wxPanel( m_window_viewport, ID_PANE_MATCHES, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxVERTICAL );
	
	m_pane_matches_view = new wxPanel( m_pane_matches, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
	m_pane_matches_view->Enable( false );
	
	bSizer5->Add( m_pane_matches_view, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxHORIZONTAL );
	
	m_cb_matches_left = new wxComboBox( m_pane_matches, wxID_ANY, wxT("Combo!"), wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_READONLY ); 
	bSizer6->Add( m_cb_matches_left, 1, wxEXPAND|wxALL, 5 );
	
	m_cb_matches_right = new wxComboBox( m_pane_matches, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_READONLY ); 
	bSizer6->Add( m_cb_matches_right, 1, wxALL|wxEXPAND, 5 );
	
	
	bSizer5->Add( bSizer6, 0, wxEXPAND, 5 );
	
	
	m_pane_matches->SetSizer( bSizer5 );
	m_pane_matches->Layout();
	bSizer5->Fit( m_pane_matches );
	m_window_viewport->AddPage( m_pane_matches, wxT("Matches"), false, wxNullBitmap );
	
	m_window_image_browser = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 370,200 ), wxTAB_TRAVERSAL );
	m_window_image_browser->SetMinSize( wxSize( 370,200 ) );
	
	m_mgr.AddPane( m_window_image_browser, wxAuiPaneInfo() .Name( wxT("Image browser") ).Left() .Caption( wxT("Image browser") ).MaximizeButton( true ).PaneBorder( false ).Dock().Resizable().FloatingSize( wxSize( 386,234 ) ).DockFixed( false ).Layer( 1 ) );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	m_dir_picker = new wxDirPickerCtrl( m_window_image_browser, wxID_ANY, wxT("D:\\Reconstruct\\TestData\\"), wxT("Select a folder"), wxDefaultPosition, wxDefaultSize, wxDIRP_DEFAULT_STYLE|wxNO_BORDER );
	bSizer1->Add( m_dir_picker, 0, wxEXPAND, 5 );
	
	m_img_ctrl = new wxListCtrl( m_window_image_browser, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_HRULES|wxLC_REPORT|wxLC_SINGLE_SEL|wxNO_BORDER );
	bSizer1->Add( m_img_ctrl, 1, wxEXPAND, 5 );
	

	m_window_image_browser->SetSizer( bSizer1 );
	m_window_image_browser->Layout();
	m_window_image_preview = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 370,200 ), wxNO_BORDER|wxTAB_TRAVERSAL );
	m_window_image_preview->SetMinSize( wxSize( 370,200 ) );
	
	m_mgr.AddPane( m_window_image_preview, wxAuiPaneInfo() .Name( wxT("Image preview") ).Left() .Caption( wxT("Image preview") ).MaximizeButton( true ).PaneBorder( false ).Dock().Resizable().FloatingSize( wxSize( 386,234 ) ).DockFixed( false ).Layer( 1 ) );

    m_window_options = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 370,200 ), wxNO_BORDER|wxTAB_TRAVERSAL );
	m_window_options->SetMinSize( wxSize( 370,200 ) );
	
	m_mgr.AddPane( m_window_options, wxAuiPaneInfo() .Name( wxT("Options") ).Left() .Caption( wxT("Options") ).MaximizeButton( true ).PaneBorder( false ).Dock().Resizable().FloatingSize( wxSize( 439,316 ) ).DockFixed( false ).Layer( 1 ) );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	m_pg_options = new wxPropertyGrid(m_window_options, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_BOLD_MODIFIED | wxPG_DEFAULT_STYLE);
	
	bSizer2->Add( m_pg_options, 1, wxEXPAND, 5 );
	
	m_btn_reset_options = new wxButton( m_window_options, wxID_ANY, wxT("Reset"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer2->Add( m_btn_reset_options, 0, 0, 5 );
	
	
	m_window_options->SetSizer( bSizer2 );
	m_window_options->Layout();
	m_window_log = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 150,150 ), wxTAB_TRAVERSAL );
	m_window_log->SetMinSize( wxSize( 150,150 ) );
	
	m_mgr.AddPane( m_window_log, wxAuiPaneInfo() .Name( wxT("Log") ).Bottom() .Caption( wxT("Log") ).MaximizeButton( true ).PaneBorder( false ).Dock().Resizable().FloatingSize( wxSize( 596,244 ) ).DockFixed( false ).Row( 0 ).Layer( 0 ) );
	
	wxBoxSizer* bSizer3 = new wxBoxSizer(wxHORIZONTAL);
	
	m_tc_log = new wxTextCtrl(m_window_log, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_DONTWRAP | wxTE_MULTILINE | wxTE_READONLY |wxNO_BORDER);
	bSizer3->Add(m_tc_log, 1, wxEXPAND, 5);
	
	m_toolbar_log = new wxAuiToolBar(m_window_log, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_TB_DEFAULT_STYLE | wxAUI_TB_VERTICAL);
	m_toolbar_log->SetToolBitmapSize(wxSize(24, 24));
	m_toolbar_log->SetToolPacking(2);

	m_toolbar_log->AddTool(ID_SAVE_LOG,		"Save Log",		wxIcon("save_icon", wxBITMAP_TYPE_ICO_RESOURCE, 22, 22),	wxNullBitmap, wxITEM_NORMAL, "Save the log to a .txt file", "Save the log to a .txt file", NULL);
	m_toolbar_log->AddTool(ID_CLEAR_LOG,	"Clear log",	wxIcon("delete_icon", wxBITMAP_TYPE_ICO_RESOURCE, 22, 22),	wxNullBitmap, wxITEM_NORMAL, "Clear the log window", "Clear the log window", NULL);
	
	m_toolbar_log->Realize(); 
	
	bSizer3->Add( m_toolbar_log, 0, wxEXPAND, 5 );
	
	
	m_window_log->SetSizer( bSizer3 );
	m_window_log->Layout();
	m_window_about = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
	m_mgr.AddPane( m_window_about, wxAuiPaneInfo() .Name( wxT("About") ).Left() .Caption( wxT("About RoboStruct") ).Hide().Float().FloatingPosition( wxPoint( 643,334 ) ).Resizable().FloatingSize( wxSize( 300,300 ) ).DockFixed( false ).BottomDockable( false ).TopDockable( false ).LeftDockable( false ).RightDockable( false ).Floatable( false ) );
	
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxVERTICAL );
	
	m_tc_about = new wxTextCtrl( m_window_about, wxID_ANY, wxT("This is awesome!\nLicense(s)\nIcons (Tango, Crystal project...)\nGLocyte"), wxDefaultPosition, wxDefaultSize, wxTE_CENTRE|wxTE_MULTILINE|wxTE_NO_VSCROLL|wxTE_READONLY|wxNO_BORDER );
	bSizer7->Add( m_tc_about, 1, wxEXPAND, 5 );
	
	
	m_window_about->SetSizer( bSizer7 );
	m_window_about->Layout();
	bSizer7->Fit( m_window_about );
	
	m_mgr.Update();
	this->Centre( wxBOTH );
	
	// Connect Events
	this->Connect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( MainFrame_base::OnUpdateUI ) );
	this->Connect( m_menu_exit->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnMenuExit ) );
	this->Connect( m_menu_reconstruct->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnReconstruct ) );
	this->Connect( m_menu_view_image_browser->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( m_menu_view_image_preview->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( m_menu_view_options->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( m_menu_view_log->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( m_menu_reset_3d_viewport->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnReset3dViewport ) );
	this->Connect( m_menu_toggle_turntable_animation->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleTurntableAnimation ) );
	this->Connect( m_menu_toggle_trackball_visibility->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( m_menu_toggle_grid_visibility->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( m_menu_toggle_points_visibility->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( m_menu_toggle_cameras_visibility->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( m_menu_toggle_frustrum_visibility->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( m_menu_export_tracks->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Connect( m_menu_export_matches->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Connect( m_menu_export_cmvs->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Connect( m_menu_export_bundle_file->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Connect( m_menu_export_ply_file->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Connect( m_menu_export_maya->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Connect( m_menu_about->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( ID_RECONSTRUCT, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnReconstruct ) );
	this->Connect( ID_RESET_3D_VIEWPORT, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnReset3dViewport ) );
	this->Connect( ID_TOGGLE_TURNTABLE_ANIMATION, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleTurntableAnimation ) );
	this->Connect( ID_TOGGLE_TRACKBALL_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( ID_TOGGLE_GRID_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( ID_TOGGLE_POINTS_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( ID_TOGGLE_CAMERAS_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( ID_TOGGLE_FRUSTRUM_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Connect( ID_VIEW_IMAGE_BROWSER, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( ID_VIEW_IMAGE_PREVIEW, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( ID_VIEW_OPTIONS, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Connect( ID_VIEW_LOG, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	m_gl_canvas->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( MainFrame_base::OnGLCanvasEraseBackground ), NULL, this );
	m_gl_canvas->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_MOTION, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_MIDDLE_DCLICK, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_RIGHT_DCLICK, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_ENTER_WINDOW, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Connect( wxEVT_PAINT, wxPaintEventHandler( MainFrame_base::OnGLCanvasPaint ), NULL, this );
	m_gl_canvas->Connect( wxEVT_SIZE, wxSizeEventHandler( MainFrame_base::OnGLCanvasSize ), NULL, this );
	m_pane_matches_view->Connect( wxEVT_PAINT, wxPaintEventHandler( MainFrame_base::OnMatchesViewPaint ), NULL, this );
	m_pane_matches_view->Connect( wxEVT_SIZE, wxSizeEventHandler( MainFrame_base::OnMatchesViewResize ), NULL, this );
	m_cb_matches_left->Connect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( MainFrame_base::OnSelectMatchImage ), NULL, this );
	m_cb_matches_right->Connect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( MainFrame_base::OnSelectMatchImage ), NULL, this );
	m_dir_picker->Connect( wxEVT_COMMAND_DIRPICKER_CHANGED, wxFileDirPickerEventHandler( MainFrame_base::OnSelectDirectory ), NULL, this );
	m_img_ctrl->Connect( wxEVT_COMMAND_LIST_ITEM_SELECTED, wxListEventHandler( MainFrame_base::OnSelectPreviewImage ), NULL, this );
	m_window_image_preview->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_MOTION, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_MIDDLE_DCLICK, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_RIGHT_DCLICK, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_ENTER_WINDOW, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Connect( wxEVT_PAINT, wxPaintEventHandler( MainFrame_base::OnImagePreviewPaint ), NULL, this );
	m_window_image_preview->Connect( wxEVT_SIZE, wxSizeEventHandler( MainFrame_base::OnImagePreviewResize ), NULL, this );
	m_btn_reset_options->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainFrame_base::OnResetOptions ), NULL, this );
	this->Connect( ID_SAVE_LOG, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnSaveLog ) );
	this->Connect( ID_CLEAR_LOG, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnClearLog ) );
}

MainFrame_base::~MainFrame_base()
{
	// Disconnect Events
	this->Disconnect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( MainFrame_base::OnUpdateUI ) );
	this->Disconnect( ID_MENU_EXIT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnMenuExit ) );
	this->Disconnect( ID_RECONSTRUCT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnReconstruct ) );
	this->Disconnect( ID_VIEW_IMAGE_BROWSER, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_VIEW_IMAGE_PREVIEW, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_VIEW_OPTIONS, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_VIEW_LOG, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_RESET_3D_VIEWPORT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnReset3dViewport ) );
	this->Disconnect( ID_TOGGLE_TURNTABLE_ANIMATION, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleTurntableAnimation ) );
	this->Disconnect( ID_TOGGLE_TRACKBALL_VISIBILITY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_GRID_VISIBILITY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_POINTS_VISIBILITY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_CAMERAS_VISIBILITY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_FRUSTRUM_VISIBILITY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_EXPORT_TRACKS, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Disconnect( ID_EXPORT_MATCHES, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Disconnect( ID_EXPORT_CMVS, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Disconnect( ID_EXPORT_BUNDLE_FILE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Disconnect( ID_EXPORT_PLY_FILE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Disconnect( ID_EXPORT_MAYA_FILE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnExport ) );
	this->Disconnect( ID_VIEW_ABOUT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_RECONSTRUCT, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnReconstruct ) );
	this->Disconnect( ID_RESET_3D_VIEWPORT, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnReset3dViewport ) );
	this->Disconnect( ID_TOGGLE_TURNTABLE_ANIMATION, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleTurntableAnimation ) );
	this->Disconnect( ID_TOGGLE_TRACKBALL_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_GRID_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_POINTS_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_CAMERAS_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_TOGGLE_FRUSTRUM_VISIBILITY, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnToggleVisibility ) );
	this->Disconnect( ID_VIEW_IMAGE_BROWSER, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_VIEW_IMAGE_PREVIEW, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_VIEW_OPTIONS, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	this->Disconnect( ID_VIEW_LOG, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnViewWindows ) );
	m_gl_canvas->Disconnect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( MainFrame_base::OnGLCanvasEraseBackground ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_MOTION, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_MIDDLE_DCLICK, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_RIGHT_DCLICK, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_ENTER_WINDOW, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( MainFrame_base::OnGLCanvasMouse ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_PAINT, wxPaintEventHandler( MainFrame_base::OnGLCanvasPaint ), NULL, this );
	m_gl_canvas->Disconnect( wxEVT_SIZE, wxSizeEventHandler( MainFrame_base::OnGLCanvasSize ), NULL, this );
	m_pane_matches_view->Disconnect( wxEVT_PAINT, wxPaintEventHandler( MainFrame_base::OnMatchesViewPaint ), NULL, this );
	m_pane_matches_view->Disconnect( wxEVT_SIZE, wxSizeEventHandler( MainFrame_base::OnMatchesViewResize ), NULL, this );
	m_cb_matches_left->Disconnect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( MainFrame_base::OnSelectMatchImage ), NULL, this );
	m_cb_matches_right->Disconnect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( MainFrame_base::OnSelectMatchImage ), NULL, this );
	m_dir_picker->Disconnect( wxEVT_COMMAND_DIRPICKER_CHANGED, wxFileDirPickerEventHandler( MainFrame_base::OnSelectDirectory ), NULL, this );
	m_img_ctrl->Disconnect( wxEVT_COMMAND_LIST_ITEM_SELECTED, wxListEventHandler( MainFrame_base::OnSelectPreviewImage ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_MOTION, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_MIDDLE_DCLICK, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_RIGHT_DCLICK, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_ENTER_WINDOW, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( MainFrame_base::OnImagePreviewMouse ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_PAINT, wxPaintEventHandler( MainFrame_base::OnImagePreviewPaint ), NULL, this );
	m_window_image_preview->Disconnect( wxEVT_SIZE, wxSizeEventHandler( MainFrame_base::OnImagePreviewResize ), NULL, this );
	m_btn_reset_options->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainFrame_base::OnResetOptions ), NULL, this );
	this->Disconnect( ID_SAVE_LOG, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnSaveLog ) );
	this->Disconnect( ID_CLEAR_LOG, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( MainFrame_base::OnClearLog ) );
	
	m_mgr.UnInit();
	
}
