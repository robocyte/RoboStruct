#pragma once

#include <wx/colour.h>

struct Options
{
	Options()
		: feature_type(0)
		, save_keys(true)
		
		, yape_radius(7)
		, yape_threshold(20)
		, yape_octaves(3)
		, yape_views(1000)
		, yape_base_feature_size(32.0)
		, yape_clustering_distance(2.0)
		
		, daisy_radius(15)
		, daisy_radius_quantization(3)
		, daisy_angular_quantization(8)
		, daisy_histogram_quantization(8)
		
		, sift_common_octaves(4)
		, sift_common_octave_layers(3)
		, sift_det_threshold(0.04)
		, sift_det_edge_threshold(10.0)
		, sift_desc_magnification(3.0)
		, sift_desc_recalculate_angles(true)
		
		, surf_det_hessian_threshold(400.0)
		, surf_common_octaves(4)
		, surf_common_octave_layers(2)
		, surf_desc_extended(false)
		, surf_desc_upright(false)
		
		, matching_trees(6)
		, matching_checks(64)
		, matching_distance_ratio(0.6)
		, matching_min_matches(20)

		, ransac_threshold_five_point(2.25)
		, ransac_rounds_five_point(500)
		, min_max_matches(16)
		, ransac_rounds_projection(1000)
		, focal_length_constrain_weight(0.0001)
		, distortion_constrain_weight(100.0)
		, projection_estimation_threshold(4.0)
		, min_reprojection_error_threshold(4.0)
		, max_reprojection_error_threshold(8.0)
		, ray_angle_threshold(2.0)
		, outlier_threshold_ba(0.0)
		
		, viewport_top_color(77, 77, 77)
		, viewport_bottom_color(204, 204, 204)
		, trackball_visibility(true)
		, grid_visibility(true)
		, points_visibility(true)
		, cameras_visibility(true)
		, frustrum_visibility(true)
		, draw_matches_only(true)
		, draw_coloured_lines(true)
		, point_size(0.2f)
		, camera_size(0.1f)
	{}

	// Feature detectors/descriptors
	int			feature_type;
	bool		save_keys;

	// YAPE
	int			yape_radius;
	int			yape_threshold;
	int			yape_octaves;
	int			yape_views;
	double		yape_base_feature_size;
	double		yape_clustering_distance;

	// Daisy
	int			daisy_radius;
	int			daisy_radius_quantization;
	int			daisy_angular_quantization;
	int			daisy_histogram_quantization;

	// SIFT
	int			sift_common_octaves;
	int			sift_common_octave_layers;
	double		sift_det_threshold;
	double		sift_det_edge_threshold;
	double		sift_desc_magnification;
	bool		sift_desc_recalculate_angles;

	// SURF
	int			surf_common_octaves;
	int			surf_common_octave_layers;
	double		surf_det_hessian_threshold;
	bool		surf_desc_extended;
	bool		surf_desc_upright;

	// Feature matching
	int			matching_trees;
	int			matching_checks;
	double		matching_distance_ratio;				// Ratio threshold for Lowe's test
	int			matching_min_matches;

	// Sfm options
	double		ransac_threshold_five_point;
	int			ransac_rounds_five_point;
	int			min_max_matches;						// Minimum number of matches needed to register an image
	int			ransac_rounds_projection;
	double		focal_length_constrain_weight;			// Weight on focal length constraint
	double		distortion_constrain_weight;			// Weight on distortion parameter constraint
    double		projection_estimation_threshold;		// RANSAC threshold for estimating projection matrix
	double		min_reprojection_error_threshold;
	double		max_reprojection_error_threshold;
	double		ray_angle_threshold;
	int			outlier_threshold_ba;

	// Ceres options


	// Display options
	wxColour	viewport_top_color;
	wxColour	viewport_bottom_color;
	bool		trackball_visibility;
	bool		grid_visibility;
	bool		points_visibility;
	bool		cameras_visibility;
	bool		frustrum_visibility;
	bool		draw_matches_only;
	bool		draw_coloured_lines;
	float		point_size;
	float		camera_size;
};
