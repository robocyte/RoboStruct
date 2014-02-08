#pragma once

#include "gly_scene.hpp"

#include "Sfm.hpp"
#include "ImageData.hpp"
#include "MatchData.hpp"
#include "Options.hpp"
#include "PointData.hpp"
#include "Profiling.hpp"
#include "TrackData.hpp"
#include "TransformData.hpp"

#include "RoboStruct_GUI.h"

typedef std::vector<int>                    IntVec;
typedef std::pair<int, int>                 IntPair;
typedef std::vector<Camera>                 CamVec;
typedef std::vector<PointData>              PointVec;
typedef std::pair<std::string, double>      CamDBEntry;
typedef std::vector<std::pair<int, int>>    IntPairVec;

enum TimerIDs
{
    ID_TIMER_TURNTABLE,
    ID_TIMER_RESET_VIEWPORT,
    ID_TIMER_ANIMATION
};

extern const wxEventTypeTag<wxThreadEvent> wxEVT_SFM_THREAD_UPDATE;
extern const wxEventTypeTag<wxThreadEvent> wxEVT_SFM_THREAD_COMPLETE;

class MainFrame : public MainFrame_base, public wxThreadHelper
{
public:
    MainFrame(wxWindow* parent = nullptr);
    ~MainFrame();

    void InitializeGuiStyle();
    void InitializeLog();
    void InitializeOpenGL();
    void InitializeScene();
    void InitializeCameraDatabase();

    void ResetOptions();
    void ResetGLCanvas();
    void ResetPerspectiveMatrix();

private:
    Options                 m_options;
    gly::ScenePtr           m_scene;
    gly::ClockPtr           m_clock;

    wxGLContext*            m_gl_context;
    wxTimer*                m_turntable_timer;
    wxTimer*                m_reset_viewport_timer;

    wxCursor                m_rotate_cursor;
    wxCursor                m_pan_cursor;
    wxCursor                m_zoom_cursor;

    wxImage                 m_preview_image;
    wxImage                 m_matches_image;

    wxCriticalSection       m_points_cs;

    float                   m_beginx = 0.0f, m_beginy = 0.0f;   // Old mouse position
    float                   m_counter = 0.0f;
    std::string             m_path;

    std::vector<CamDBEntry> m_camDB;                            // Contains the information from CamDB.txt
    std::vector<ImageData>  m_images;                           // Image data
    std::vector<TrackData>  m_tracks;                           // Information about the detected 3D tracks
    MatchTable              m_matches;                          // Holds the matches
    TransformData           m_transforms;                       // Holds transform info
    PointVec                m_points;                           // Holds reconstructed points

    ProfileManager          m_profile_manager;

    bool                    m_has_images = false;
    bool                    m_features_detected = false;
    bool                    m_matches_loaded = false;
    bool                    m_matches_refined = false;
    bool                    m_sfm_done = false;

    int                     m_desc_length = 0;

    void        GeneratePreviewImage(int img_idx);
    void        GenerateMatchImage();
    void        DrawImagePreview(wxDC& dc);
    void        DrawMatches(wxDC& dc);

    bool        AddImage(const std::string& filename, const std::string& filename_short);       // Try to add an image to the initial list of images
    bool        ReadCamDBFile(const std::string& filename);
    void        AddCamDBFileEntry();
    bool        FindCameraInDatabase(ImageData& img);

    void        DetectFeaturesAll();
    void        DetectFeatures(int img_idx);

    void        MatchAll();
    void        MatchAllAkaze();
    std::size_t PruneDoubleMatches(IntPairVec& matches);
    std::size_t ComputeEpipolarGeometry(int idx1, int idx2, IntPairVec& matches);               // Computes the fundamental matrix F and removes outliers
    double      ComputeHomography(int idx1, int idx2, const IntPairVec& matches);               // Computes the homography H and returns the inlier ratio
    void        MakeMatchListsSymmetric();
    void        ComputeTracks();                                                                // Organize the matches into tracks, where a track is a connected set of matching keypoints across multiple images

    int         GetImageWidth(int img_index)    { return m_images[img_index].GetWidth(); };     // Returns the image width
    int         GetImageHeight(int img_index)   { return m_images[img_index].GetHeight(); };    // Returns the image height
    double      GetFocalLength(int img_index)   { return m_images[img_index].GetInitFocal(); }; // Returns the focal length
    int         GetNumImages()                  { return (int)m_images.size(); };               // Returns the number of successfully added images
    int         GetNumKeys(int img)             { return (int)m_images[img].m_keys.size(); };   // Returns the number of detected features for the given image index
    int         GetNumTrackMatches(int img1, int img2);
    double      GetInlierRatio(int idx1, int idx2);                                             // Returns the inlier ratio for an image pair
    KeyPoint&   GetKey(int img, int key)        { return m_images[img].m_keys[key]; };

    void        SetMatch(int i1, int i2)        { m_matches.SetMatch(ImagePair(i1, i2)); };
    void        SetMatchesFromTracks(int img1, int img2);
    void        SetMatchesFromPoints();

    void        SaveMatchFile();
    void        SaveTrackFile();
    void        SaveProjectionMatrix(const std::string& path, int img_idx);
    void        SaveUndistortedImage(const std::string& path, int img_idx);
    void        ExportToCMVS(const std::string& path);
    void        SaveBundleFile(const std::string& path);
    void        SavePlyFile();
    void        SaveMeshLabFile();
    void        SaveMayaFile();

    wxThread::ExitCode  Entry();
    void                RunSFM();
    void                PickInitialCameraPair(CamVec& cameras, IntVec& added_order);
    void                SetupInitialCameraPair(CamVec& cameras, const IntVec& added_order, PointVec& points);
    bool                EstimateRelativePose(int i1, int i2, Camera* camera1, Camera* camera2);
    int                 FindCameraWithMostMatches(const IntVec& added_order, int& max_matches, const PointVec& points);
    IntVec              FindCamerasWithNMatches(int n, const IntVec& added_order, const PointVec& points);
    bool                FindAndVerifyCamera(const Point3Vec& points, const Point2Vec& projections, Mat3* K, Mat3* R, Vec3* t, IntVec& inliers, IntVec& inliers_weak, IntVec& outliers);
    Camera              InitializeImage(int image_idx, int camera_idx, PointVec& points);
    void                BundleAdjust(CamVec& cameras, const IntVec& added_order, PointVec& points);
    void                RefineCameraParameters(Camera* camera, const Point3Vec& points, const Point2Vec& projections, IntVec& inliers);
    void                AddNewPoints(const CamVec& cameras, const IntVec& added_order, PointVec& points);
    int                 RemoveBadPointsAndCameras(const CamVec& cameras, const IntVec& added_order, PointVec& points);
    void                RadiusOutlierRemoval(double threshold, const IntVec& added_order, PointVec& points);
    void                UpdateGeometryDisplay(const CamVec& cameras, const IntVec& added_order, const PointVec& points);

protected:
    // Handlers for MainFrame events
    void OnSFMThreadUpdate(wxThreadEvent& event);
    void OnSFMThreadComplete(wxThreadEvent& event);
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
    void OnToggleTurntableAnimation(wxCommandEvent& event);
    void OnToggleVisibility(wxCommandEvent& event);
    void OnExport(wxCommandEvent& event);

    void OnSelectDirectory(wxFileDirPickerEvent& event);
    void OnSelectPreviewImage(wxListEvent& event);
    void OnSelectMatchImage(wxCommandEvent& event);

    void OnImagePreviewMouse(wxMouseEvent& event);
    void OnImagePreviewPaint(wxPaintEvent& event);
    void OnImagePreviewResize(wxSizeEvent& event);
    void OnMatchesViewPaint(wxPaintEvent& event);
    void OnMatchesViewResize(wxSizeEvent& event);

    void OnSaveLog(wxCommandEvent& event);
    void OnClearLog(wxCommandEvent& event);

    void OnTimerUpdate(wxTimerEvent& event);
};
