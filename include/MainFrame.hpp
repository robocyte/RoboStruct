#pragma once

#include "gly_scene.hpp"
#include "wx/msw/winundef.h"
#include "wx/glcanvas.h"
#include "wx/timer.h"

#include "Sfm.hpp"
#include "ImageData.hpp"
#include "MatchData.hpp"
#include "Options.hpp"
#include "PointData.hpp"
#include "TrackData.hpp"
#include "TransformData.hpp"

#include "RoboStruct_GUI.h"

typedef std::vector<int>                    IntVec;
typedef std::pair<int, int>					IntPair;
typedef std::vector<Camera>                 CamVec;
typedef std::vector<PointData>              PointVec;
typedef std::pair<std::string, double>		CamDBEntry;
typedef std::vector<std::pair<int, int>>	IntPairVec;

enum TimerIDs
{
	ID_TIMER_TURNTABLE,
	ID_TIMER_RESET_VIEWPORT,
	ID_TIMER_ANIMATION
};

extern const wxEventTypeTag<wxThreadEvent> wxEVT_THREAD_UPDATE;

class MainFrame : public MainFrame_base, public wxThreadHelper
{
public:
	MainFrame(wxWindow* parent = nullptr);
	~MainFrame();

private:
	Options			m_options;
	gly::ScenePtr	m_scene;
	gly::ClockPtr	m_clock;

	wxGLContext*	m_gl_context;
	wxTimer*		m_turntable_timer;
	wxTimer*		m_reset_viewport_timer;

	wxCursor		m_rotate_cursor;
	wxCursor		m_pan_cursor;
	wxCursor		m_zoom_cursor;

	float			m_beginx, m_beginy;		// Old mouse position
	float			m_counter;
	std::string		m_path;

	std::vector<CamDBEntry>		m_camDB;								// Contains the information from CamDB.txt
	std::vector<ImageData>		m_images;								// Image data
	std::vector<TrackData>		m_tracks;								// Information about the detected 3D tracks
	MatchTable					m_matches;								// Holds the matches
	TransformData				m_transforms;							// Holds transform info
	PointVec            		m_points;								// Holds reconstructed points

	bool						m_has_images;
	bool						m_features_detected;					// Have features been detected?
	bool						m_matches_loaded;						// Have the matches been loaded?
	bool						m_matches_refined;						// Have the matches been refined?
	bool						m_sfm_done;

	int							m_desc_length;
	wxCriticalSection			m_points_cs;

	void InitializeLog();
	void InitializeOpenGL();
	void InitializeScene();
	void InitializeCameraDatabase();

	void ResetOptions();
	void ResetGLCanvas();
	void ResetPerspectiveMatrix();
	void UpdateGLTransformations();

	bool AddImage(const std::string filename, const std::string filename_short);				// Try to add an image to the initial list of images
	bool ReadCamDBFile(const std::string filename);
	void AddCamDBFileEntry();
	bool FindCameraInDatabase(ImageData &img);

	void		DetectFeaturesAll();
	void		DetectFeatures(int img_idx);
	void		MatchAll();
	KeyPoint&	GetKey(int img, int key)    { return m_images[img].m_keys[key]; };
	void		SetMatch(int i1, int i2)    { m_matches.SetMatch(GetMatchIndex(i1, i2)); };
	int			PruneDoubleMatches(IntPairVec &matches);
	int			ComputeEpipolarGeometry(int idx1, int idx2, IntPairVec &matches);	        // Computes the fundamental matrix F and removes outliers
	double		ComputeHomography(int idx1, int idx2, const IntPairVec &matches);	        // Computes the homography H and returns the inlier ratio
	void		MakeMatchListsSymmetric();
	void		ComputeTracks();														    // Organize the matches into tracks, where a track is a connected set of matching keypoints across multiple images
	void		SetMatchesFromTracks(int img1, int img2);
	void		SetMatchesFromPoints();

	int		GetImageWidth(int img_index)    { return m_images[img_index].GetWidth(); };		// Returns the image width
	int		GetImageHeight(int img_index)   { return m_images[img_index].GetHeight(); };	// Returns the image height
	double	GetFocalLength(int img_index)   { return m_images[img_index].GetInitFocal(); };	// Returns the focal length
	int		GetNumImages()                  { return (int)m_images.size(); };				// Returns the number of successfully added images
	int		GetNumKeys(int img)             { return (int)m_images[img].m_keys.size(); };	// Returns the number of detected features for the given image index
	int		GetNumTrackMatches(int img1, int img2);
	double	GetInlierRatio(int idx1, int idx2);												// Returns the inlier ratio for an image pair
	void	SaveMatchFile();
	void	SaveMatchPics();
	void	SaveTrackFile();
	void	SaveProjectionMatrix(const std::string &path, int img_idx);
	void	SaveUndistortedImage(const std::string &path, int img_idx);
	void	ExportToCMVS();
	void	SaveBundleFile(const std::string &path);
	void	SavePlyFile();
	void	SaveMayaFile();

	void				StartBundlerThread();
	wxThread::ExitCode	Entry();
	void				BundleAdjust();
	IntPair				PickInitialCameraPair();
    void				SetupInitialCameraPair(IntPair initial_pair, CamVec &cameras, PointVec &points);
	bool				EstimateRelativePose(int i1, int i2, Camera *camera1, Camera *camera2);
	int					FindCameraWithMostMatches(int num_cameras, const IntVec &added_order, int &max_matches, const PointVec &points);
	IntVec          	FindCamerasWithNMatches(int n, int num_cameras, const IntVec &added_order, const PointVec &points);
	bool				FindAndVerifyCamera(const Vec3Vec &points, const Vec2Vec &projections, int *idxs_solve, Mat3 *K, Mat3 *R, Vec3 *t, IntVec &inliers, IntVec &inliers_weak, IntVec &outliers);
	Camera	        	BundleInitializeImage(int image_idx, int camera_idx, PointVec &points, bool &success_out);
	void				RunSFM(int num_cameras, CamVec &cameras, const IntVec &added_order, PointVec &points);
	void				RefineCameraParameters(Camera *camera, const Vec3Vec &points, const Vec2Vec &projections, int *pt_idxs, IntVec &inliers);
	void				BundleAdjustAddAllNewPoints(int num_cameras, IntVec &added_order, CamVec &cameras, PointVec &points);
	int					RemoveBadPointsAndCameras(const IntVec &added_order, CamVec &cameras, PointVec &points);

protected:
	// Handlers for MainFrame events
	void OnThreadUpdate(wxThreadEvent& event);
	void OnUpdateUI(wxUpdateUIEvent& event);

	void OnClose(wxCloseEvent& event);
	void OnMenuExit(wxCommandEvent& event);

	void OnResetOptions(wxCommandEvent& event);
	void OnOptionsChanged(wxPropertyGridEvent& event);

	void OnGLCanvasEraseBackground(wxEraseEvent& event);
	void OnGLCanvasMouse(wxMouseEvent& event);
	void OnGLCanvasPaint(wxPaintEvent& event);
	void OnGLCanvasSize(wxSizeEvent& event);

	void OnReconstruct(wxCommandEvent& event);
	void OnViewWindows(wxCommandEvent& event);
	void OnReset3dViewport(wxCommandEvent& event);
	void OnChangeMSAASamples(wxSpinEvent& event);
	void OnToggleTurntableAnimation(wxCommandEvent& event);
	void OnToggleVisibility(wxCommandEvent& event);
	void OnExport(wxCommandEvent& event);

	void OnSelectDirectory(wxFileDirPickerEvent& event);
	//void OnSelectPreviewImage(wxListEvent& event);
	//void OnSelectMatchImage(wxCommandEvent& event);

	//void OnImagePreviewMouse(wxMouseEvent& event);
	//void OnImagePreviewPaint(wxPaintEvent& event);
	//void OnImagePreviewResize(wxSizeEvent& event);
	//void OnMatchesViewPaint(wxPaintEvent& event);
	//void OnMatchesViewResize(wxSizeEvent& event);

	void OnSaveLog(wxCommandEvent& event);
	void OnClearLog(wxCommandEvent& event);

	void OnTimerUpdate(wxTimerEvent& event);
};
