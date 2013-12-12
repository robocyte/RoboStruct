#pragma once

#include "wx/colour.h"

struct Options
{
    Options() = default;

    // Feature detectors/descriptors
    int         feature_type = 0;
    bool        save_keys = true;

    // YAPE
    int         yape_radius = 7;
    int         yape_threshold = 20;
    int         yape_octaves = 3;
    int         yape_views = 1000;
    double      yape_base_feature_size = 32;
    double      yape_clustering_distance = 2;

    // Daisy
    int         daisy_radius = 15;
    int         daisy_radius_quantization = 3;
    int         daisy_angular_quantization = 8;
    int         daisy_histogram_quantization = 8;

    // SURF
    int         surf_common_octaves = 4;
    int         surf_common_octave_layers = 2;
    double      surf_det_hessian_threshold = 400.0;
    bool        surf_desc_extended = false;
    bool        surf_desc_upright = false;

    // AKAZE
    double      akaze_threshold = 0.0007;
    int         akaze_descriptor_size = 0;

    // Feature matching
    int         matching_trees = 4;
    int         matching_checks = 32;
    double      matching_distance_ratio = 0.6;              // Ratio threshold for Lowe's test
    int         matching_min_matches = 20;
    double      ransac_threshold_fundamental = 0.001;
    double      ransac_threshold_homography = 0.001;


    // Sfm options
    bool        add_multiple_images = true;
    double      ransac_threshold_five_point = 2.25;
    int         ransac_rounds_five_point = 500;
    int         min_max_matches = 16;                       // Minimum number of matches needed to register an image
    int         ransac_rounds_projection = 1000;
    double      focal_length_constrain_weight = 0.0001;     // Weight on focal length constraint
    double      distortion_constrain_weight = 0.0001;       // Weight on distortion parameter constraint
    double      projection_estimation_threshold = 4.0;      // RANSAC threshold for estimating projection matrix
    double      min_reprojection_error_threshold = 4.0;
    double      max_reprojection_error_threshold = 8.0;
    double      ray_angle_threshold = 2.0;
    int         outlier_threshold_ba = 0.0;
    double      outlier_threshold_radius = 0.98;

    // Ceres options
    enum class loss_function
    {
        squared     = 1,
        huber       = 2,
        softlone    = 3,
        cauchy      = 4,
        arctan      = 5
    };

    loss_function   selected_loss = loss_function::squared;
    double          loss_function_scale = 0.5;

    // Display options
    wxColour    viewport_top_color = { 77, 77, 77 };
    wxColour    viewport_bottom_color = { 204, 204, 204 };
    bool        trackball_visibility = true;
    bool        grid_visibility = true;
    bool        points_visibility = true;
    bool        cameras_visibility = true;
    bool        frustrum_visibility = true;
    bool        draw_matches_only = true;
    bool        draw_coloured_lines = true;
    float       point_size = 0.2f;
    float       camera_size = 0.1f;
};
