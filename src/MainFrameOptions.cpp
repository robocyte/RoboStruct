#include "MainFrame.hpp"

void MainFrame::ResetOptions()
{
    m_options = Options{};

    // Setup the option box
    m_pg_options->Clear();
    m_pg_options->SetPropertyAttributeAll(wxPG_BOOL_USE_CHECKBOX, true);

    // Detection category
    wxArrayString features_list;
    features_list.Add("YAPE/Daisy");
    features_list.Add("SURF");
    features_list.Add("AKAZE");

    // Feature detectors/descriptors
    wxPGProperty* feature_category = m_pg_options->Append( new wxPropertyCategory{ "Feature detection/description", wxPG_LABEL } );
    m_pg_options->Append( new wxEnumProperty{"Feature type",    wxPG_LABEL, features_list } );
    m_pg_options->Append( new wxBoolProperty{"Save keys",       wxPG_LABEL, m_options.save_keys } );

        // YAPE category
        wxPGProperty* yape_category = m_pg_options->AppendIn( feature_category, new wxStringProperty{ "YAPE detector", wxPG_LABEL, "<composed>" } );
        m_pg_options->AppendIn( yape_category, new wxIntProperty{"Yape radius",                     wxPG_LABEL,     m_options.yape_radius } );
        m_pg_options->AppendIn( yape_category, new wxIntProperty{"Yape threshold",                  wxPG_LABEL,     m_options.yape_threshold } );
        m_pg_options->AppendIn( yape_category, new wxIntProperty{"Yape octaves",                    wxPG_LABEL,     m_options.yape_octaves } );
        m_pg_options->AppendIn( yape_category, new wxIntProperty{"Yape views",                      wxPG_LABEL,     m_options.yape_views } );
        m_pg_options->AppendIn( yape_category, new wxFloatProperty{"Yape base feature size",        wxPG_LABEL,     m_options.yape_base_feature_size } );
        m_pg_options->AppendIn( yape_category, new wxFloatProperty{"Yape clustering distance",      wxPG_LABEL,     m_options.yape_clustering_distance } );
        m_pg_options->GetProperty("YAPE detector")->SetExpanded(false);

        // DAISY category
        wxPGProperty* daisy_category = m_pg_options->AppendIn( feature_category, new wxStringProperty{ "DAISY descriptor", wxPG_LABEL, "<composed>" } );
        m_pg_options->AppendIn( daisy_category, new wxIntProperty{"Daisy radius",                   wxPG_LABEL,     m_options.daisy_radius } );
        m_pg_options->AppendIn( daisy_category, new wxIntProperty{"Daisy radius quantization",      wxPG_LABEL,     m_options.daisy_radius_quantization } );
        m_pg_options->AppendIn( daisy_category, new wxIntProperty{"Daisy angular quantization",     wxPG_LABEL,     m_options.daisy_angular_quantization } );
        m_pg_options->AppendIn( daisy_category, new wxIntProperty{"Daisy histogram quantization",   wxPG_LABEL,     m_options.daisy_histogram_quantization } );
        m_pg_options->GetProperty("DAISY descriptor")->SetExpanded(false);

        // SURF category
        wxPGProperty* surf_category = m_pg_options->AppendIn( feature_category, new wxStringProperty{ "SURF detector/descriptor", wxPG_LABEL, "<composed>" } );
        m_pg_options->AppendIn( surf_category, new wxIntProperty{"Surf octaves",                    wxPG_LABEL,     m_options.surf_common_octaves } );
        m_pg_options->AppendIn( surf_category, new wxIntProperty{"Surf octave layers",              wxPG_LABEL,     m_options.surf_common_octave_layers } );
        m_pg_options->AppendIn( surf_category, new wxFloatProperty{"Surf hessian threshold",        wxPG_LABEL,     m_options.surf_det_hessian_threshold } );
        m_pg_options->AppendIn( surf_category, new wxBoolProperty{"Surf extended",                  wxPG_LABEL,     m_options.surf_desc_extended } );
        m_pg_options->AppendIn( surf_category, new wxBoolProperty{"Surf upright",                   wxPG_LABEL,     m_options.surf_desc_upright } );
        m_pg_options->GetProperty("SURF detector/descriptor")->SetExpanded(false);

        // AKAZE category
        wxPGProperty* akaze_category = m_pg_options->AppendIn( feature_category, new wxStringProperty{ "AKAZE detector/descriptor", wxPG_LABEL, "<composed>" } );
        m_pg_options->AppendIn( akaze_category, new wxIntProperty{"Akaze descriptor size",          wxPG_LABEL,     m_options.akaze_descriptor_size } );
        m_pg_options->AppendIn( akaze_category, new wxFloatProperty{"Akaze threshold",              wxPG_LABEL,     m_options.akaze_threshold } );
        m_pg_options->GetProperty("AKAZE detector/descriptor")->SetExpanded(false);

    // Feature matching category
    m_pg_options->Append( new wxPropertyCategory{ "Feature matching" } );
    m_pg_options->Append( new wxIntProperty{"Trees",                            wxPG_LABEL,     m_options.matching_trees } );
    m_pg_options->Append( new wxIntProperty{"Checks",                           wxPG_LABEL,     m_options.matching_checks } );
    m_pg_options->Append( new wxFloatProperty{"Distance ratio",                 wxPG_LABEL,     m_options.matching_distance_ratio } );
    m_pg_options->Append( new wxIntProperty{"Min matches",                      wxPG_LABEL,     m_options.matching_min_matches } );
    m_pg_options->Append( new wxFloatProperty{"Fundamental RANSAC threshold",   wxPG_LABEL,     m_options.ransac_threshold_fundamental } );
    m_pg_options->Append( new wxFloatProperty{"Homography RANSAC threshold",    wxPG_LABEL,     m_options.ransac_threshold_homography } );

    // Structure from motion category
    m_pg_options->Append( new wxPropertyCategory{ "Structure from motion" } );
    m_pg_options->Append( new wxBoolProperty{"Add multiple images",                 wxPG_LABEL, m_options.add_multiple_images } );
    m_pg_options->Append( new wxFloatProperty{"5point RANSAC threshold",            wxPG_LABEL, m_options.ransac_threshold_five_point } );
    m_pg_options->Append( new wxIntProperty{"5point RANSAC rounds",                 wxPG_LABEL, m_options.ransac_rounds_five_point } );
    m_pg_options->Append( new wxIntProperty{"Minimum image matches",                wxPG_LABEL, m_options.min_max_matches } );
    m_pg_options->Append( new wxIntProperty{"Projection RANSAC rounds",             wxPG_LABEL, m_options.ransac_rounds_projection } );
    m_pg_options->Append( new wxFloatProperty{"Focal length constrain weight",      wxPG_LABEL, m_options.focal_length_constrain_weight } );
    m_pg_options->Append( new wxFloatProperty{"Distortion constrain weight",        wxPG_LABEL, m_options.distortion_constrain_weight } );
    m_pg_options->Append( new wxFloatProperty{"Projection estimation threshold",    wxPG_LABEL, m_options.projection_estimation_threshold } );
    m_pg_options->Append( new wxFloatProperty{"Min reprojection error threshold",   wxPG_LABEL, m_options.min_reprojection_error_threshold } );
    m_pg_options->Append( new wxFloatProperty{"Max reprojection error threshold",   wxPG_LABEL, m_options.max_reprojection_error_threshold } );
    m_pg_options->Append( new wxFloatProperty{"Ray angle threshold",                wxPG_LABEL, m_options.ray_angle_threshold } );
    m_pg_options->Append( new wxIntProperty{"Outlier threshold BA",                 wxPG_LABEL, m_options.outlier_threshold_ba } );
    m_pg_options->Append( new wxFloatProperty{"Outlier threshold radius",           wxPG_LABEL, m_options.outlier_threshold_radius } );

    // Ceres category
    wxPGChoices choices;
    choices.Add("Squared", 1); choices.Add("Huber", 2); choices.Add("SoftLOne", 3); choices.Add("Cauchy", 4); choices.Add("ArcTan", 5);
    m_pg_options->Append( new wxEnumProperty{"Loss function",                       wxPG_LABEL, choices, 1} );
    m_pg_options->Append( new wxFloatProperty{"Loss function scale",                wxPG_LABEL, m_options.loss_function_scale } );

    // Display category
    m_pg_options->Append( new wxPropertyCategory{ "Display options" } );
    m_pg_options->Append( new wxColourProperty{"Viewport gradient top", wxPG_LABEL, m_options.viewport_top_color } );
    m_pg_options->Append( new wxColourProperty{"Viewport gradient bot", wxPG_LABEL, m_options.viewport_bottom_color } );
    m_pg_options->Append( new wxBoolProperty{"Trackball visibility",    wxPG_LABEL, m_options.trackball_visibility } );
    m_pg_options->Append( new wxBoolProperty{"Grid visibility",         wxPG_LABEL, m_options.grid_visibility } );
    m_pg_options->Append( new wxBoolProperty{"Points visibility",       wxPG_LABEL, m_options.points_visibility } );
    m_pg_options->Append( new wxBoolProperty{"Cameras visibility",      wxPG_LABEL, m_options.cameras_visibility } );
    m_pg_options->Append( new wxBoolProperty{"Frustrum visibility",     wxPG_LABEL, m_options.frustrum_visibility } );
    m_pg_options->Append( new wxBoolProperty{"Draw matches only",       wxPG_LABEL, m_options.draw_matches_only } );
    m_pg_options->Append( new wxBoolProperty{"Draw coloured lines",     wxPG_LABEL, m_options.draw_coloured_lines } );
}

void MainFrame::OnOptionsChanged(wxPropertyGridEvent& event)
{
    m_options.feature_type                      = m_pg_options->GetProperty("Feature type")->GetValue().GetInteger();
    m_options.save_keys                         = m_pg_options->GetProperty("Save keys")->GetValue().GetBool();

    m_options.yape_radius                       = m_pg_options->GetProperty("YAPE detector.Yape radius")->GetValue().GetInteger();
    m_options.yape_threshold                    = m_pg_options->GetProperty("YAPE detector.Yape threshold")->GetValue().GetInteger();
    m_options.yape_octaves                      = m_pg_options->GetProperty("YAPE detector.Yape octaves")->GetValue().GetInteger();
    m_options.yape_views                        = m_pg_options->GetProperty("YAPE detector.Yape views")->GetValue().GetInteger();
    m_options.yape_base_feature_size            = m_pg_options->GetProperty("YAPE detector.Yape base feature size")->GetValue().GetDouble();
    m_options.yape_clustering_distance          = m_pg_options->GetProperty("YAPE detector.Yape clustering distance")->GetValue().GetDouble();

    m_options.daisy_radius                      = m_pg_options->GetProperty("DAISY descriptor.Daisy radius")->GetValue().GetInteger();
    m_options.daisy_radius_quantization         = m_pg_options->GetProperty("DAISY descriptor.Daisy radius quantization")->GetValue().GetInteger();
    m_options.daisy_angular_quantization        = m_pg_options->GetProperty("DAISY descriptor.Daisy angular quantization")->GetValue().GetInteger();
    m_options.daisy_histogram_quantization      = m_pg_options->GetProperty("DAISY descriptor.Daisy histogram quantization")->GetValue().GetInteger();

    m_options.surf_common_octaves               = m_pg_options->GetProperty("SURF detector/descriptor.Surf octaves")->GetValue().GetInteger();
    m_options.surf_common_octave_layers         = m_pg_options->GetProperty("SURF detector/descriptor.Surf octave layers")->GetValue().GetInteger();
    m_options.surf_det_hessian_threshold        = m_pg_options->GetProperty("SURF detector/descriptor.Surf hessian threshold")->GetValue().GetDouble();
    m_options.surf_desc_extended                = m_pg_options->GetProperty("SURF detector/descriptor.Surf extended")->GetValue().GetBool();
    m_options.surf_desc_upright                 = m_pg_options->GetProperty("SURF detector/descriptor.Surf upright")->GetValue().GetBool();

    m_options.akaze_descriptor_size             = m_pg_options->GetProperty("AKAZE detector/descriptor.Akaze descriptor size")->GetValue().GetInteger();
    m_options.akaze_threshold                   = m_pg_options->GetProperty("AKAZE detector/descriptor.Akaze threshold")->GetValue().GetDouble();

    m_options.matching_trees                    = m_pg_options->GetProperty("Trees")->GetValue().GetInteger();
    m_options.matching_checks                   = m_pg_options->GetProperty("Checks")->GetValue().GetInteger();
    m_options.matching_distance_ratio           = m_pg_options->GetProperty("Distance ratio")->GetValue().GetDouble();
    m_options.matching_min_matches              = m_pg_options->GetProperty("Min matches")->GetValue().GetInteger();
    m_options.ransac_threshold_fundamental      = m_pg_options->GetProperty("Fundamental RANSAC threshold")->GetValue().GetDouble();
    m_options.ransac_threshold_homography       = m_pg_options->GetProperty("Homography RANSAC threshold")->GetValue().GetDouble();

    m_options.add_multiple_images               = m_pg_options->GetProperty("Add multiple images")->GetValue().GetBool();
    m_options.ransac_threshold_five_point       = m_pg_options->GetProperty("5point RANSAC threshold")->GetValue().GetDouble();
    m_options.ransac_rounds_five_point          = m_pg_options->GetProperty("5point RANSAC rounds")->GetValue().GetInteger();
    m_options.min_max_matches                   = m_pg_options->GetProperty("Minimum image matches")->GetValue().GetInteger();
    m_options.ransac_rounds_projection          = m_pg_options->GetProperty("Projection RANSAC rounds")->GetValue().GetInteger();
    m_options.focal_length_constrain_weight     = m_pg_options->GetProperty("Focal length constrain weight")->GetValue().GetDouble();
    m_options.distortion_constrain_weight       = m_pg_options->GetProperty("Distortion constrain weight")->GetValue().GetDouble();
    m_options.projection_estimation_threshold   = m_pg_options->GetProperty("Projection estimation threshold")->GetValue().GetDouble();
    m_options.min_reprojection_error_threshold  = m_pg_options->GetProperty("Min reprojection error threshold")->GetValue().GetDouble();
    m_options.max_reprojection_error_threshold  = m_pg_options->GetProperty("Max reprojection error threshold")->GetValue().GetDouble();
    m_options.ray_angle_threshold               = m_pg_options->GetProperty("Ray angle threshold")->GetValue().GetDouble();
    m_options.outlier_threshold_ba              = m_pg_options->GetProperty("Outlier threshold BA")->GetValue().GetInteger();
    m_options.outlier_threshold_radius          = m_pg_options->GetProperty("Outlier threshold radius")->GetValue().GetDouble();
    int sel                                     = m_pg_options->GetProperty("Loss function")->GetValue().GetInteger();
    switch (sel)
    {
    case 1: m_options.selected_loss             = Options::loss_function::squared;  break;
    case 2: m_options.selected_loss             = Options::loss_function::huber;    break;
    case 3: m_options.selected_loss             = Options::loss_function::softlone; break;
    case 4: m_options.selected_loss             = Options::loss_function::cauchy;   break;
    case 5: m_options.selected_loss             = Options::loss_function::arctan;   break;
    default: break;
    }
    m_options.loss_function_scale               = m_pg_options->GetProperty("Loss function scale")->GetValue().GetDouble();

    wxAny top                                   = m_pg_options->GetProperty("Viewport gradient top")->GetValue();
    wxAny bot                                   = m_pg_options->GetProperty("Viewport gradient bot")->GetValue();
    m_options.viewport_top_color                = top.As<wxColour>();
    m_options.viewport_bottom_color             = bot.As<wxColour>();
    m_options.trackball_visibility              = m_pg_options->GetProperty("Trackball visibility")->GetValue().GetBool();
    m_options.grid_visibility                   = m_pg_options->GetProperty("Grid visibility")->GetValue().GetBool();
    m_options.points_visibility                 = m_pg_options->GetProperty("Points visibility")->GetValue().GetBool();
    m_options.cameras_visibility                = m_pg_options->GetProperty("Cameras visibility")->GetValue().GetBool();
    m_options.frustrum_visibility               = m_pg_options->GetProperty("Frustrum visibility")->GetValue().GetBool();
    m_options.draw_matches_only                 = m_pg_options->GetProperty("Draw matches only")->GetValue().GetBool();
    m_options.draw_coloured_lines               = m_pg_options->GetProperty("Draw coloured lines")->GetValue().GetBool();

    const auto t = m_options.viewport_top_color;
    const auto b = m_options.viewport_bottom_color;

    m_scene->GetBackground()->SetGradient(glm::vec3{t.Red() / 255.0f, t.Green() / 255.0f, t.Blue() / 255.0f},
                                          glm::vec3{b.Red() / 255.0f, b.Green() / 255.0f, b.Blue() / 255.0f});

    GenerateMatchImage();
    this->Refresh();
}
