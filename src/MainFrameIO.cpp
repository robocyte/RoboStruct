#include <fstream>
#include <iomanip>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "ImageData.hpp"
#include "MainFrame.hpp"
#include "Utilities.hpp"

bool MainFrame::ReadCamDBFile(const std::string& filename)
{
    m_camDB.clear();

    std::ifstream cam_db_file{filename};

    if (!cam_db_file)
    {
        wxLogMessage("Error: couldn't open file %s for reading", filename.c_str());
        return false;
    } else
    {
        std::string model;
        double      ccd_width;

        while (std::getline(cam_db_file, model, ':'))
        {
            cam_db_file >> ccd_width;
            cam_db_file.ignore();

            m_camDB.push_back(CamDBEntry{model, ccd_width});
        }
        return true;
    }
    return false;
}

void MainFrame::AddCamDBFileEntry()
{
    std::ofstream cam_db_file{"CamDB.txt"};

    if (!cam_db_file)
    {
        wxLogMessage("Error: couldn't open file CamDB.txt for writing");
    } else
    {
        std::sort(m_camDB.begin(), m_camDB.end());

        for (const auto &entry : m_camDB) cam_db_file << entry.first << ":" << entry.second << std::endl;
    }
}

void MainFrame::SaveMatchFile()
{
    const auto filename = m_path + R"(\Matches.txt)";
    std::ofstream match_file{filename};

    if (!match_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing matches to %s...", filename.c_str());
        //TODO!!
    }
}

void MainFrame::SaveTrackFile()
{
    const auto filename = m_path + R"(\Tracks.txt)";
    std::ofstream track_file{filename};

    if (!track_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing tracks to %s...", filename.c_str());

        // Print number of tracks
        track_file << (int)m_tracks.size() << std::endl;

        // Now write each track
        for (const auto& track : m_tracks)
        {
            track_file << track.m_views.size();
            for (const auto& view : track.m_views) track_file << " " << view.first << " " << view.second;
            track_file << std::endl;
        }
    }
}

void MainFrame::SaveProjectionMatrix(const std::string& path, int img_idx)
{
    if (!m_images[img_idx].m_camera.m_adjusted) return;

    auto filename = m_images[img_idx].m_filename_short;
    std::transform(filename.begin(), filename.end(), filename.begin(), std::tolower);
    filename.replace(filename.find(".jpg"), 4, ".txt");
    filename = path + R"(\)" + filename;

    std::ofstream txt_file{filename};

    if (!txt_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing projection matrix to %s...", filename.c_str());

        const double focal = m_images[img_idx].m_camera.m_focal_length;
        const auto& R      = m_images[img_idx].m_camera.m_R;
        const auto& t      = m_images[img_idx].m_camera.m_t;

        Mat3 K;
        K << -focal,   0.0,   0.5 * GetImageWidth(img_idx)  - 0.5,
                0.0, focal,   0.5 * GetImageHeight(img_idx) - 0.5,
                0.0,   0.0,   1.0;

        Mat34 Rigid;
        Rigid << R;
        Rigid.col(3) = -(R * t);

        const Mat34 P = -(K * Rigid);

        txt_file << "CONTOUR" << std::endl
                 << P(0, 0) << " " << P(0, 1) << " " << P(0, 2) << " " << P(0, 3) << std::endl
                 << P(1, 0) << " " << P(1, 1) << " " << P(1, 2) << " " << P(1, 3) << std::endl
                 << P(2, 0) << " " << P(2, 1) << " " << P(2, 2) << " " << P(2, 3) << std::endl;
    }
}

void MainFrame::SaveUndistortedImage(const std::string& path, int img_idx)
{
    if (!m_images[img_idx].m_camera.m_adjusted) return;

    const cv::Mat img = cv::imread(m_images[img_idx].m_filename);
    cv::Mat img_undist(img.size(), CV_8UC3);

    // Fill camera matrix
    const double focal = m_images[img_idx].m_camera.m_focal_length;
    cv::Mat_<double> camMat{3, 3};
    camMat << focal,    0.0,    0.5 * img.cols - 0.5,
                0.0,  focal,    0.5 * img.rows - 0.5,
                0.0,    0.0,    1.0;

    // Distortion coefficients
    const double k1 = m_images[img_idx].m_camera.m_k[0];
    const double k2 = m_images[img_idx].m_camera.m_k[1];
    cv::Mat_<double> distCoeffs{1, 5};
    distCoeffs << k1, k2, 0.0, 0.0, 0.0;

    // Undistort!
    cv::undistort(img, img_undist, camMat, distCoeffs);

    // Save the image
    std::string short_name{m_images[img_idx].m_filename_short};
    std::transform(short_name.begin(), short_name.end(), short_name.begin(), std::tolower);
    const std::string filename = path + R"(\)" + short_name;
    m_images[img_idx].m_filename_undistorted = filename;
    wxLogMessage("Saving undistorted image to %s...", filename.c_str());
    cv::imwrite(filename, img_undist);
}

void MainFrame::ExportToCMVS(const std::string& path)
{
    const std::string pmvs_dir{  path + R"(\cmvs)"};
    const std::string image_dir{ path + R"(\cmvs\visualize)"};
    const std::string txt_dir{   path + R"(\cmvs\txt)"};
    const std::string models_dir{path + R"(\cmvs\models)"};

    // Create directory structure
    if (!wxDirExists(pmvs_dir))
    {
        wxMkdir(pmvs_dir.c_str());
        wxMkdir(image_dir.c_str());
        wxMkdir(txt_dir.c_str());
        wxMkdir(models_dir.c_str());
    }

    SaveBundleFile(pmvs_dir);

    for (int i = 0; i < GetNumImages(); i++)
    {
        SaveProjectionMatrix(txt_dir,   i);
        SaveUndistortedImage(image_dir, i);
    }

    wxLogMessage("Export to CMVS complete!");
}

void MainFrame::SaveBundleFile(const std::string& path)
{
    const auto filename = path + R"(\bundle.rd.out)";
    std::ofstream bundle_file{filename};

    bundle_file << "# Bundle file v0.3" << std::endl;

    if (!bundle_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing geometry and cameras to %s...", filename.c_str());

        const int num_images = std::count_if(m_images.begin(), m_images.end(), [](const ImageData& img) { return img.m_camera.m_adjusted; });
        const int num_points = std::count_if(m_points.begin(), m_points.end(), [](const PointData& point) { return !point.m_views.empty(); });

        bundle_file << num_images << " " << num_points << std::endl;

        // Write cameras
        for (const auto& image : m_images)
        {
            if (!image.m_camera.m_adjusted) continue;

            // Focal length
            bundle_file << image.m_camera.m_focal_length << " 0.0 0.0" << std::endl;

            // Rotation
            const auto& R = image.m_camera.m_R;
            bundle_file << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << std::endl
                        << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << std::endl
                        << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << std::endl;

            // Translation
            const auto& t = image.m_camera.m_t;
            bundle_file << t.x() << " " << t.y() << " " << t.z() << std::endl;
        }

        // Write points
        for (const auto& point : m_points)
        {
            if (!point.m_views.empty())
            {
                bundle_file << point.m_pos[0] << " "
                            << point.m_pos[1] << " "
                            << point.m_pos[2] << std::endl;

                bundle_file << util::iround(point.m_color[0] * 255) << " "
                            << util::iround(point.m_color[1] * 255) << " "
                            << util::iround(point.m_color[2] * 255) << std::endl;

                // View data
                bundle_file << point.m_views.size();
                for (const auto& view : point.m_views)
                {
                    int img = view.first;
                    int key = view.second;

                    double x = m_images[img].m_keys[key].m_coords.x();
                    double y = m_images[img].m_keys[key].m_coords.y();

                    bundle_file << " " << img << " " << key << " " << x << " " << y;
                }
                bundle_file << std::endl;
            }
        }
    }
}

void MainFrame::SavePlyFile()
{
    const auto filename = m_path + R"(\Result.ply)";
    std::ofstream ply_file{filename};

    if (!ply_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing points and cameras to %s...", filename.c_str());

        const int num_cameras = std::count_if(m_images.begin(), m_images.end(), [](const ImageData& img) { return img.m_camera.m_adjusted; });
        const auto num_points = m_points.size();

        // Output the header
        ply_file << "ply" << std::endl
                 << "format ascii 1.0" << std::endl
                 << "element vertex " << num_points + 2 * num_cameras << std::endl
                 << "property float x" << std::endl
                 << "property float y" << std::endl
                 << "property float z" << std::endl
                 << "property uchar diffuse_red" << std::endl
                 << "property uchar diffuse_green" << std::endl
                 << "property uchar diffuse_blue" << std::endl
                 << "end_header" << std::endl;

        // Output the vertices
        for (const auto& point : m_points)
        {
            // Point positions
            ply_file    << point.m_pos.x() << " " << point.m_pos.y() << " " << point.m_pos.z() << " ";

            // Point colors
            ply_file    << util::iround(point.m_color[0] * 255) << " "
                        << util::iround(point.m_color[1] * 255) << " "
                        << util::iround(point.m_color[2] * 255) << std::endl;
        }

        // Output the cameras
        for (const auto& img : m_images)
        {
            if (!img.m_camera.m_adjusted) continue;

            const auto& R = img.m_camera.m_R;
            const auto& t = img.m_camera.m_t;

            ply_file << t.x() << " " << t.y() << " " << t.z() << " "
                     << 255   << " " << 0     << " " << 0     << std::endl;

            const Point3 p = R.transpose() * Point3{0.0, 0.0, -0.05} + t;

            ply_file << p.x() << " " << p.y() << " " << p.z() << " "
                     << 0     << " " << 255   << " " << 0     << std::endl;
        }
    }
}

void MainFrame::SaveMeshLabFile()
{
    // Export to CMVS first to get undistorted images
    ExportToCMVS(m_path);

    const auto filename = m_path + R"(\Result.mlp)";
    std::ofstream mlp_file{filename};

    if (!mlp_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing points and cameras to %s...", filename.c_str());

        // Write header and output mesh location
        mlp_file << R"(<!DOCTYPE MeshLabDocument>)" << std::endl
                 << R"(<MeshLabProject>)" << std::endl
                 << R"( <MeshGroup>)" << std::endl
                 << R"(  <MLMesh label="Result" filename="Result.ply">)" << std::endl
                 << R"(   <MLMatrix44>)" << std::endl
                 << R"(1 0 0 0 )" << std::endl
                 << R"(0 1 0 0 )" << std::endl
                 << R"(0 0 1 0 )" << std::endl
                 << R"(0 0 0 1 )" << std::endl
                 << R"(</MLMatrix44>)" << std::endl
                 << R"(  </MLMesh>)" << std::endl
                 << R"( </MeshGroup>)" << std::endl;

        // Write cameras as raster layers
        mlp_file << R"( <RasterGroup>)" << std::endl;

        for (const auto& img : m_images)
        {
            if (!img.m_camera.m_adjusted) continue;

            const auto& Rot =  img.m_camera.m_R;
            const auto& t   = -img.m_camera.m_t;

            mlp_file << R"(  <MLRaster label=")" << img.m_filename_short << R"(">)" << std::endl
                     << R"(   <VCGCamera TranslationVector=")" << t.x() << " " << t.y() << " " << t.z() << " " << 1 << R"(" )"
                     <<           R"(LensDistortion="0 0" )"
                     <<           R"(ViewportPx=")" << img.GetWidth() << " " << img.GetHeight() << R"(" )"
                     <<           R"(PixelSizeMm="1 1" )"
                     <<           R"(CenterPx=")" << (int)std::floor(img.GetWidth()) << " " << (int)std::floor(img.GetHeight()) << R"(" )"
                     <<           R"(FocalMm=")" << img.m_camera.m_focal_length << R"(" )"
                     <<           R"(RotationMatrix=")"
                     <<             Rot(0, 0) << " " << Rot(0, 1) << " " << Rot(0, 2) << " " << "0 "
                     <<             Rot(1, 0) << " " << Rot(1, 1) << " " << Rot(1, 2) << " " << "0 "
                     <<             Rot(2, 0) << " " << Rot(2, 1) << " " << Rot(2, 2) << " " << "0 "
                     <<             R"(0 0 0 1 "/>)" << std::endl
                     << R"(   <Plane semantic="" fileName="cmvs/visualize/)" << img.m_filename_short << R"("/>)" << std::endl
                     << R"(  </MLRaster>)" << std::endl;
        }

        mlp_file << R"( </RasterGroup>)" << std::endl;
        mlp_file << R"(</MeshLabProject>)" << std::endl;
    }
}

void MainFrame::SaveMayaFile()
{
    // Export to CMVS first to get undistorted images
    ExportToCMVS(m_path);

    const auto filename = m_path + R"(\Result.ma)";
    std::ofstream maya_file{filename};

    if (!maya_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    }
    else
    {
        wxLogMessage("Writing points and cameras to %s...", filename.c_str());

        const auto num_points = m_points.size();

        // Write header
        maya_file << R"(//Maya ASCII 2008 scene)" << std::endl
                  << R"(//Name: Result.ma)" << std::endl
                  << R"(//Codeset: 1252)" << std::endl
                  << R"(requires maya "2008";)" << std::endl
                  << R"(currentUnit -l centimeter -a degree -t film;)" << std::endl;

        // Write cameras
        for (std::size_t i = 0; i < m_images.size(); i++)
        {
            if (!m_images[i].m_camera.m_adjusted) continue;

            const double width      = m_images[i].GetWidth();
            const double height     = m_images[i].GetHeight();
            const double ccd_width  = m_images[i].m_ccd_width;
            const double ccd_height = ccd_width * height / width;
            const double focalpx    = m_images[i].m_camera.m_focal_length;
            const auto& R           = m_images[i].m_camera.m_R;
            const auto& t           = m_images[i].m_camera.m_t;
            double focalmm;
            double cap_w;   // cap = camera aperture
            double cap_h;

            if (width >= height)
            {
                focalmm = (focalpx * ccd_width) / width;
                cap_w   = ccd_width  / 25.4;    // mm to inches
                cap_h   = ccd_height / 25.4;    // mm to inches
            }
            else
            {
                focalmm = (focalpx * ccd_width) / height;
                cap_w   = ccd_height / 25.4;    // mm to inches
                cap_h   = ccd_width  / 25.4;    // mm to inches
            }

            maya_file << R"(createNode transform -s -n "Cam)" << std::setw(4) << std::setfill('0') << i << R"(";)" << std::endl
                      << R"(    setAttr -type "matrix" ".xformMatrix" )" << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << 0.0 << " "
                                                                         << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << 0.0 << " "
                                                                         << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << 0.0 << " "
                                                                         << t.x()   << " " << t.y()   << " " << t.z()   << " " << 1.0 << ";" << std::endl
                      << R"(    setAttr -l on ".tx";)" << std::endl
                      << R"(    setAttr -l on ".ty";)" << std::endl
                      << R"(    setAttr -l on ".tz";)" << std::endl
                      << R"(    setAttr -l on ".rx";)" << std::endl
                      << R"(    setAttr -l on ".ry";)" << std::endl
                      << R"(    setAttr -l on ".rz";)" << std::endl
                      << R"(    setAttr ".s" -type "double3" 0.1 0.1 0.1;)" << std::endl

                      << R"(createNode camera -s -n "Cam)" << std::setw(4) << std::setfill('0') << i << R"(Shape" -p "Cam)" << std::setw(4) << std::setfill('0') << i << R"(";)" << std::endl
                      << R"(    setAttr -k off ".v";)" << std::endl
                      << R"(    setAttr ".rnd" no;)" << std::endl
                      << R"(    setAttr ".cap" -type "double2" )" << cap_w << " " << cap_h << ";" << std::endl
                      << R"(    setAttr ".ff" 0;)" << std::endl
                      << R"(    setAttr ".fl" )" << focalmm << ";" << std::endl
                      << R"(    setAttr ".ncp" 0.01;)" << std::endl
                      << R"(    setAttr ".fcp" 10000;)" << std::endl
                      << R"(    setAttr ".ow" 30;)" << std::endl
                      << R"(    setAttr ".imn" -type "string" "Cam)" << std::setw(4) << std::setfill('0') << i << R"(";)" << std::endl
                      << R"(    setAttr ".den" -type "string" "Cam)" << std::setw(4) << std::setfill('0') << i << R"(_depth";)" << std::endl
                      << R"(    setAttr ".man" -type "string" "Cam)" << std::setw(4) << std::setfill('0') << i << R"(_mask";)" << std::endl;

            // Write image planes
            std::string path{m_images[i].m_filename_undistorted};
            std::string backslashes{R"(\)"};
            while ((path.find(backslashes)) != std::string::npos) path.replace(path.find(backslashes), backslashes.length(), "/");

            maya_file << R"(createNode imagePlane -n "ImagePlane)" << std::setw(4) << std::setfill('0') << i << R"(";)" << std::endl
                      << R"(    setAttr ".fc" 12;)" << std::endl
                      << R"(    setAttr ".imn" -type "string" ")" << path.c_str() << R"(";)" << std::endl
                      << R"(    setAttr ".cov" -type "short2" )" << m_images[i].GetWidth() << " " << m_images[i].GetHeight() << ";" << std::endl
                      << R"(    setAttr ".ag" 0.5;)" << std::endl
                      << R"(    setAttr ".d" 3;)" << std::endl
                      << R"(    setAttr ".s" -type "double2" )" << cap_w << " " << cap_h << ";" << std::endl
                      << R"(    setAttr ".c" -type "double3" 0.0 0.0 0.0;)" << std::endl
                      << R"(    setAttr ".w" 30;)" << std::endl
                      << R"(    setAttr ".h" 30;)" << std::endl;

            // Connect image plane to camera
            maya_file << R"(connectAttr "ImagePlane)" << std::setw(4) << std::setfill('0') << i << R"(.msg" ":Cam)" << std::setw(4) << std::setfill('0') << i << R"(Shape.ip" -na;)" << std::endl;
        }

        // Write particles
        maya_file << R"(createNode transform -n "particle1";)" << std::endl
                  << R"(createNode particle -n "particleShape1" -p "particle1";)" << std::endl
                  << R"(    addAttr -ci true -sn "lifespanPP" -ln "lifespanPP" -bt "life" -dt "doubleArray";)" << std::endl
                  << R"(    addAttr -ci true -h true -sn "lifespanPP0" -ln "lifespanPP0" -bt "life" -dt "doubleArray";)" << std::endl
                  << R"(    addAttr -ci true -sn "lifespan" -ln "lifespan" -bt "life" -at "double";)" << std::endl
                  << R"(    addAttr -ci true -sn "rgbPP" -ln "rgbPP" -dt "vectorArray";)" << std::endl
                  << R"(    addAttr -ci true -h true -sn "rgbPP0" -ln "rgbPP0" -dt "vectorArray";)" << std::endl
                  << R"(    addAttr -is true -ci true -sn "colorAccum" -ln "colorAccum" -min 0 -max 1 -at "bool";)" << std::endl
                  << R"(    addAttr -is true -ci true -sn "useLighting" -ln "useLighting" -min 0 -max 1 -at "bool";)" << std::endl
                  << R"(    addAttr -is true -ci true -sn "pointSize" -ln "pointSize" -dv 2 -min 1 -max 60 -at "long";)" << std::endl
                  << R"(    addAttr -is true -ci true -sn "normalDir" -ln "normalDir" -dv 2 -min 1 -max 3 -at "long";)" << std::endl
                  << R"(    setAttr -k off ".v";)" << std::endl
                  << R"(    setAttr ".gf" -type "Int32Array" 0 ;)" << std::endl;

        // Position
        maya_file << R"(    setAttr ".pos0" -type "vectorArray" )" << std::setw(4) << std::setfill('0') << num_points;
        for (const auto& p : m_points) maya_file << " " << p.m_pos.x() << " " << p.m_pos.y() << " " << p.m_pos.z();
        maya_file << ";" << std::endl;

        // Color
        maya_file << R"(    setAttr ".rgbPP0" -type "vectorArray" )" << std::setw(4) << std::setfill('0') << num_points;
        for (const auto& p : m_points) maya_file << " " << p.m_color[0] << " " << p.m_color[1] << " " << p.m_color[2];
        maya_file << ";" << std::endl;

        maya_file << R"(    setAttr -k on ".colorAccum";)" << std::endl
                  << R"(    setAttr -k on ".useLighting";)" << std::endl
                  << R"(    setAttr -k on ".pointSize" 3;)" << std::endl
                  << R"(    setAttr -k on ".normalDir";)" << std::endl
                  << "// End of Result.ma" << std::endl;
    }
}