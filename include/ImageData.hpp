#pragma once

#include <vector>

#include "gly_scene.hpp"

#include "opencv2/core.hpp"

#include "Keys.hpp"
#include "Sfm.hpp"

struct Options;

class ImageData
{
public:
    ImageData(const std::string &filename, const std::string &filename_short);

    std::string             m_filename;             // Filename (including path)
    std::string             m_filename_undistorted; // Filename (including path) of the undistorted image
    std::string             m_filename_short;       // Filename (without path)
    std::string             m_camera_model;         // Camera model name
    std::string             m_camera_make;          // Camera maker name
    std::string             m_node_name;

    Camera                  m_camera;               // Information about the camera used to capture this image
    gly::NodePtr            m_camera_mesh;

    std::vector<KeyPoint>   m_keys;                 // Bundler keypoints
    std::vector<bool>       m_key_flags;
    std::vector<int>        m_visible_keys;         // Indices of keys visible in this image
    std::vector<int>        m_visible_points;       // Indices of points visible in this image
    std::vector<float>      m_descriptors;
    cv::Mat                 m_descriptors_akaze;

    double                  m_ccd_width;            // CCD width (mm)
    double                  m_init_focal;           // Initial focal length estimate (px)
    double                  m_init_focal_mm;        // Focal length in mm from exif tag

    int                     m_desc_size;

    bool                    m_ignore_in_bundle;
    bool                    m_descriptors_loaded;

    bool	GetExifInfo();                              // Get focal length (mm) and other info from EXIF tags
    double	GetInitFocal()  { return m_init_focal; }    // Returns the initial focal length in px
    int		GetWidth()      { return m_width; }         // Returns the image width in px
    int		GetHeight()     { return m_height; }        // Returns the image height in px

    void	DetectFeatures(const Options& opts);        // Detects features and returns the descriptor length

    void	ClearDescriptors();
    void	SaveDescriptors(bool clear = true);
    void	LoadDescriptors();
    void	SetTracks();

private:
    int     m_width;                                // Width from exif tags
    int     m_height;                               // Height from exif tags

    void    ConvertOpenCVKeys(const std::vector<cv::KeyPoint>& keys, const cv::Mat& image);
};
