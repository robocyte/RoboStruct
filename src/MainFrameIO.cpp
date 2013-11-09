#include <fstream>
#include <iomanip>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "ImageData.hpp"
#include "MainFrame.hpp"
#include "Utilities.hpp"

bool MainFrame::ReadCamDBFile(std::string filename)
{
    m_camDB.clear();

    std::ifstream cam_db_file(filename);

    if (!cam_db_file)
    {
        wxLogMessage("Error: couldn't open file %s for reading", filename.c_str());
        return false;
    } else
    {
        std::string	model;
        double		ccd_width;

        while (std::getline(cam_db_file, model, ':'))
        {
            cam_db_file >> ccd_width;
            cam_db_file.ignore();

            m_camDB.push_back(CamDBEntry(model, ccd_width));
        }

        cam_db_file.close();
        return true;
    }
    return false;
}

void MainFrame::AddCamDBFileEntry()
{
    std::ofstream cam_db_file("CamDB.txt");

    if (!cam_db_file)
    {
        wxLogMessage("Error: couldn't open file CamDB.txt for writing");
    } else
    {
        std::sort(m_camDB.begin(), m_camDB.end());

        for (const auto &entry : m_camDB) cam_db_file << entry.first << ":" << entry.second << std::endl;

        cam_db_file.close();
    }
}

void MainFrame::SaveMatchFile()
{
    std::string filename = m_path + "\\Matches.txt";
    std::ofstream match_file(filename);

    if (!match_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing matches to file %s...", filename.c_str());
        //TODO!!
        match_file.close();
    }
}

void MainFrame::SaveTrackFile()
{
    std::string filename = m_path + "\\Tracks.txt";
    std::ofstream track_file(filename);

    if (!track_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing tracks to file %s...", filename.c_str());

        // Print number of tracks
        track_file << (int)m_tracks.size() << std::endl;

        // Now write each track
        for (auto track : m_tracks)
        {
            track_file << track.m_views.size();
            for (const auto &view : track.m_views) track_file << " " << view.first << " " << view.second;
            track_file << std::endl;
        }

        track_file.close();
    }
}

void MainFrame::SaveMatchPics()
{
}

void MainFrame::SaveProjectionMatrix(const std::string &path, int img_idx)
{
    if (!m_images[img_idx].m_camera.m_adjusted) return;

    std::string filename = m_images[img_idx].m_filename_short;
    filename.replace(filename.find(".jpg"), 4, ".txt");
    filename = path + "\\" + filename;

    std::ofstream txt_file(filename);

    if (!txt_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        double focal = m_images[img_idx].m_camera.m_focal_length;
        const auto &R = m_images[img_idx].m_camera.m_R;

        Mat3 K;
        K << -focal,   0.0,   0.5 * this->GetImageWidth(img_idx) - 0.5,
                0.0, focal,   0.5 * this->GetImageHeight(img_idx) - 0.5,
                0.0,   0.0,   1.0;

        Mat34 Rigid;
        Rigid << R;
        Rigid.col(3) = -(R * m_images[img_idx].m_camera.m_t);

        Mat34 P = -(K * Rigid);

        txt_file << "CONTOUR" << std::endl;
        txt_file << P(0, 0) << " " << P(0, 1) << " " << P(0, 2) << " " << P(0, 3) << std::endl;
        txt_file << P(1, 0) << " " << P(1, 1) << " " << P(1, 2) << " " << P(1, 3) << std::endl;
        txt_file << P(2, 0) << " " << P(2, 1) << " " << P(2, 2) << " " << P(2, 3) << std::endl;

        txt_file.close();
    }
}

void MainFrame::SaveUndistortedImage(const std::string &path, int img_idx)
{
    if (!m_images[img_idx].m_camera.m_adjusted) return;

    const cv::Mat img = cv::imread(m_images[img_idx].m_filename);
    cv::Mat img_undist(img.size(), CV_8UC3);

    // Fill camera matrix
    double focal = m_images[img_idx].m_camera.m_focal_length;
    cv::Mat_<double> camMat(3, 3);
    camMat <<   focal,  0.0,    0.5 * img.cols - 0.5,
                0.0,    focal,  0.5 * img.rows - 0.5,
                0.0,    0.0,    1.0;

    // Distortion coefficients
    double k1 = m_images[img_idx].m_camera.m_k[0];
    double k2 = m_images[img_idx].m_camera.m_k[1];
    cv::Mat_<double> distCoeffs(1, 5);
    distCoeffs << k1, k2, 0.0, 0.0, 0.0;

    // Undistort!
    cv::undistort(img, img_undist, camMat, distCoeffs);

    // Save the image
    std::string filename = path + "\\" + m_images[img_idx].m_filename_short;
    m_images[img_idx].m_filename_undistorted = filename;
    cv::imwrite(filename, img_undist);
}

void MainFrame::ExportToCMVS()
{
    // Create directory structure
    if (!wxDirExists((m_dir_picker->GetPath()).append("\\pmvs")))
    {
        wxMkdir((m_dir_picker->GetPath()).append("\\pmvs"));
        wxMkdir((m_dir_picker->GetPath()).append("\\pmvs\\visualize"));
        wxMkdir((m_dir_picker->GetPath()).append("\\pmvs\\txt"));
        wxMkdir((m_dir_picker->GetPath()).append("\\pmvs\\models"));
    }

    // Save bundle file
    this->SaveBundleFile( ((m_dir_picker->GetPath()).append("\\pmvs")).ToStdString() );

    for (int i = 0; i < this->GetNumImages(); i++)
    {
        // Output projection matrices
        wxLogMessage("Generating txt file for image %i...", i);
        this->SaveProjectionMatrix(((m_dir_picker->GetPath()).append("\\pmvs\\txt")).ToStdString(), i);

        // Output undistorted images
        wxLogMessage("Generating undistorted image (%i)...", i);
        this->SaveUndistortedImage(((m_dir_picker->GetPath()).append("\\pmvs\\visualize")).ToStdString(), i);
    }

    wxLogMessage("Export to CMVS complete!");
}

void MainFrame::SaveBundleFile(const std::string &path)
{
    std::string filename = path + "\\bundle.rd.out";
    std::ofstream bundle_file(filename);

    bundle_file << "# Bundle file v0.3" << std::endl;

    if (!bundle_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing geometry and cameras to file %s...", filename.c_str());

        int num_points(m_points.size());
        int num_visible_points(0);
        int num_images = std::count_if(m_images.begin(), m_images.end(), [](const ImageData &img) { return img.m_camera.m_adjusted; });

        for (const auto &point : m_points) if (point.m_views.size() > 0) num_visible_points++;

        bundle_file << num_images << " " << num_visible_points << std::endl;

        // Write cameras
        for (const auto &image : m_images)
        {
            if (!image.m_camera.m_adjusted) continue;

            // Focal length
            bundle_file << image.m_camera.m_focal_length << " 0.0 0.0" << std::endl;

            // Rotation
            const auto &R = image.m_camera.m_R;
            bundle_file << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << std::endl
                        << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << std::endl
                        << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << std::endl;

            // Translation
            const auto &t = image.m_camera.m_t;
            bundle_file << t.x() << " " << t.y() << " " << t.z() << std::endl;
        }

        // Write points
        for (const auto &point : m_points)
        {
            if (point.m_views.size() > 0)
            {
                bundle_file << point.m_pos[0] << " "
                            << point.m_pos[1] << " "
                            << point.m_pos[2] << std::endl;

                bundle_file << util::iround(point.m_color[0]) << " "
                            << util::iround(point.m_color[1]) << " "
                            << util::iround(point.m_color[2]) << std::endl;

                // View data
                bundle_file << point.m_views.size();
                for (const auto &view : point.m_views)
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
        bundle_file.close();
    }
}

void MainFrame::SavePlyFile()
{
    std::string filename = m_path + "\\Result.ply";
    std::ofstream ply_file(filename);

    if (!ply_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing geometry and cameras to file %s...", filename.c_str());

        int num_cameras = std::count_if(m_images.begin(), m_images.end(), [](const ImageData &img) { return img.m_camera.m_adjusted; });
        int num_points = (int)m_points.size();

        // Output the header
        ply_file << "ply" << std::endl;
        ply_file << "format ascii 1.0" << std::endl;
        ply_file << "element vertex " << num_points + 2 * num_cameras << std::endl;
        ply_file << "property float x" << std::endl;
        ply_file << "property float y" << std::endl;
        ply_file << "property float z" << std::endl;
        ply_file << "property uchar diffuse_red" << std::endl;
        ply_file << "property uchar diffuse_green" << std::endl;
        ply_file << "property uchar diffuse_blue" << std::endl;
        ply_file << "end_header" << std::endl;

        // Output the vertices
        for (const auto &point : m_points)
        {
            // Point positions
            ply_file    << point.m_pos[0] << " " << point.m_pos[1] << " " << point.m_pos[2] << " ";

            // Point colors
            ply_file    << util::iround(point.m_color[0] * 255) << " "
                        << util::iround(point.m_color[1] * 255) << " "
                        << util::iround(point.m_color[2] * 255) << std::endl;
        }

        // Output the cameras
        for (const auto &img : m_images)
        {
            if (!img.m_camera.m_adjusted) continue;

            const auto &R = img.m_camera.m_R;
            const auto &t = img.m_camera.m_t;

            ply_file << t.x() << " " << t.y() << " " << t.z() << " ";
            ply_file << 255 << " " << 0 << " " << 0 << std::endl;

            Point3 p = R.inverse() * Point3(0.0, 0.0, -0.05) + t;

            ply_file << p.x() << " " << p.y() << " " << p.z() << " ";
            ply_file << 0 << " " << 255 << " " << 0 << std::endl;
        }

        ply_file.close();
    }
}

void MainFrame::SaveMayaFile()
{
    // Export to CMVS first in order to get undistorted images
    this->ExportToCMVS();

    std::string filename = m_path + "\\Result.ma";
    std::ofstream maya_file(filename);

    if (!maya_file)
    {
        wxLogMessage("Error: couldn't open file %s for writing", filename.c_str());
    } else
    {
        wxLogMessage("Writing geometry and cameras to file %s...", filename.c_str());

        int num_points = (int)m_points.size();
        int num_cameras = std::count_if(m_images.begin(), m_images.end(), [](const ImageData &img) { return img.m_camera.m_adjusted; });

        // Write header and default cameras
        maya_file << "//Maya ASCII 2008 scene\n";
        maya_file << "//Name: Result.ma\n";
        maya_file << "//Codeset: 1252\n";
        maya_file << "requires maya \"2008\";\n";
        maya_file << "currentUnit -l centimeter -a degree -t film;\n";
        maya_file << "fileInfo \"application\" \"maya\";\n";
        maya_file << "fileInfo \"product\" \"Maya Unlimited 2008\";\n";
        maya_file << "fileInfo \"version\" \"2008 Service Pack 1 x64\";\n";
        maya_file << "fileInfo \"cutIdentifier\" \"200802250523-718081\";\n";
        maya_file << "fileInfo \"osv\" \"" << (wxGetOsDescription()).c_str() << "\";\n";
        maya_file << "createNode transform -s -n \"persp\";\n";
        maya_file << "\tsetAttr \".v\" no;\n";
        maya_file << "\tsetAttr \".t\" -type \"double3\" -12.539952725595253 9.5501149256283853 25.940138118868703 ;\n";
        maya_file << "\tsetAttr \".r\" -type \"double3\" -18.338352729714941 -385.80000000001149 8.8317459951165346e-016 ;\n";
        maya_file << "createNode camera -s -n \"perspShape\" -p \"persp\";\n";
        maya_file << "\tsetAttr -k off \".v\" no;\n";
        maya_file << "\tsetAttr \".fl\" 34.999999999999993;\n";
        maya_file << "\tsetAttr \".fcp\" 10000;\n";
        maya_file << "\tsetAttr \".coi\" 30.353679761750993;\n";
        maya_file << "\tsetAttr \".imn\" -type \"string\" \"persp\";\n";
        maya_file << "\tsetAttr \".den\" -type \"string\" \"persp_depth\";\n";
        maya_file << "\tsetAttr \".man\" -type \"string\" \"persp_mask\";\n";
        maya_file << "\tsetAttr \".hc\" -type \"string\" \"viewSet -p %%camera\";\n";
        maya_file << "createNode transform -s -n \"top\";\n";
        maya_file << "\tsetAttr \".v\" no;\n";
        maya_file << "\tsetAttr \".t\" -type \"double3\" 0 100.1 0 ;\n";
        maya_file << "\tsetAttr \".r\" -type \"double3\" -89.999999999999986 0 0 ;\n";
        maya_file << "createNode camera -s -n \"topShape\" -p \"top\";\n";
        maya_file << "\tsetAttr -k off \".v\" no;\n";
        maya_file << "\tsetAttr \".rnd\" no;\n";
        maya_file << "\tsetAttr \".fcp\" 10000;\n";
        maya_file << "\tsetAttr \".coi\" 100.1;\n";
        maya_file << "\tsetAttr \".ow\" 30;\n";
        maya_file << "\tsetAttr \".imn\" -type \"string\" \"top\";\n";
        maya_file << "\tsetAttr \".den\" -type \"string\" \"top_depth\";\n";
        maya_file << "\tsetAttr \".man\" -type \"string\" \"top_mask\";\n";
        maya_file << "\tsetAttr \".hc\" -type \"string\" \"viewSet -t %%camera\";\n";
        maya_file << "\tsetAttr \".o\" yes;\n";
        maya_file << "createNode transform -s -n \"front\";\n";
        maya_file << "\tsetAttr \".v\" no;\n";
        maya_file << "\tsetAttr \".t\" -type \"double3\" 0 0 100.1 ;\n";
        maya_file << "createNode camera -s -n \"frontShape\" -p \"front\";\n";
        maya_file << "\tsetAttr -k off \".v\" no;\n";
        maya_file << "\tsetAttr \".rnd\" no;\n";
        maya_file << "\tsetAttr \".fcp\" 10000;\n";
        maya_file << "\tsetAttr \".coi\" 100.1;\n";
        maya_file << "\tsetAttr \".ow\" 30;\n";
        maya_file << "\tsetAttr \".imn\" -type \"string\" \"front\";\n";
        maya_file << "\tsetAttr \".den\" -type \"string\" \"front_depth\";\n";
        maya_file << "\tsetAttr \".man\" -type \"string\" \"front_mask\";\n";
        maya_file << "\tsetAttr \".hc\" -type \"string\" \"viewSet -f %%camera\";\n";
        maya_file << "\tsetAttr \".o\" yes;\n";
        maya_file << "createNode transform -s -n \"side\";\n";
        maya_file << "\tsetAttr \".v\" no;\n";
        maya_file << "\tsetAttr \".t\" -type \"double3\" 100.1 0 0 ;\n";
        maya_file << "\tsetAttr \".r\" -type \"double3\" 0 89.999999999999986 0 ;\n";
        maya_file << "createNode camera -s -n \"sideShape\" -p \"side\";\n";
        maya_file << "\tsetAttr -k off \".v\" no;\n";
        maya_file << "\tsetAttr \".rnd\" no;\n";
        maya_file << "\tsetAttr \".fcp\" 10000;\n";
        maya_file << "\tsetAttr \".coi\" 100.1;\n";
        maya_file << "\tsetAttr \".ow\" 30;\n";
        maya_file << "\tsetAttr \".imn\" -type \"string\" \"side\";\n";
        maya_file << "\tsetAttr \".den\" -type \"string\" \"side_depth\";\n";
        maya_file << "\tsetAttr \".man\" -type \"string\" \"side_mask\";\n";
        maya_file << "\tsetAttr \".hc\" -type \"string\" \"viewSet -s %%camera\";\n";
        maya_file << "\tsetAttr \".o\" yes;\n";
	
        // Write cameras
        for (int i = 0; i < m_images.size(); i++)
        {
            if (!m_images[i].m_camera.m_adjusted) continue;

            double width = static_cast<double>(m_images[i].GetWidth());
            double height = static_cast<double>(m_images[i].GetHeight());
            double ccd_width = m_images[i].m_ccd_width;
            double ccd_height = ccd_width * height / width;
            double focalpx = m_images[i].m_camera.m_focal_length;
            double focalmm;
            double cap_w;	// cap = camera aperture
            double cap_h;
            const auto &R = m_images[i].m_camera.m_R;
            const auto &t = m_images[i].m_camera.m_t;

            if (width >= height)
            {
                focalmm = (focalpx * ccd_width)/width;
                cap_w = ccd_width/25.4;     // mm to inches
                cap_h = ccd_height/25.4;    // mm to inches
            } else
            {
                focalmm = (focalpx * ccd_width)/height;
                cap_w = ccd_height/25.4;    // mm to inches
                cap_h = ccd_width/25.4;     // mm to inches
            }

            maya_file << "createNode transform -s -n \"Cam" << std::setw(4) << std::setfill('0') << i << "\";\n";
            maya_file << "\tsetAttr -type \"matrix\" \".xformMatrix\" " << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << 0.0 << " "
                                                                        << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << 0.0 << " "
                                                                        << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << 0.0 << " "
                                                                        << t.x() << " " << t.y() << " " << t.z() << " " << 1.0 << ";\n";
            maya_file << "\tsetAttr -l on \".tx\";\n";
            maya_file << "\tsetAttr -l on \".ty\";\n";
            maya_file << "\tsetAttr -l on \".tz\";\n";
            maya_file << "\tsetAttr -l on \".rx\";\n";
            maya_file << "\tsetAttr -l on \".ry\";\n";
            maya_file << "\tsetAttr -l on \".rz\";\n";
            maya_file << "\tsetAttr \".s\" -type \"double3\" 0.1 0.1 0.1;\n";

            maya_file << "createNode camera -s -n \"Cam" << std::setw(4) << std::setfill('0') << i << "Shape\" -p \"Cam" << std::setw(4) << std::setfill('0') << i << "\";\n";
            maya_file << "\tsetAttr -k off \".v\";\n";
            maya_file << "\tsetAttr \".rnd\" no;\n";
            maya_file << "\tsetAttr \".cap\" -type \"double2\" " << cap_w << " " << cap_h << ";\n";
            maya_file << "\tsetAttr \".ff\" 0;\n";
            maya_file << "\tsetAttr \".fl\" " << focalmm << ";\n";
            maya_file << "\tsetAttr \".ncp\" 0.01;\n";
            maya_file << "\tsetAttr \".fcp\" 10000;\n";
            maya_file << "\tsetAttr \".ow\" 30;\n";
            maya_file << "\tsetAttr \".imn\" -type \"string\" \"Cam" << std::setw(4) << std::setfill('0') << i << "\";\n";
            maya_file << "\tsetAttr \".den\" -type \"string\" \"Cam" << std::setw(4) << std::setfill('0') << i << "_depth\";\n";
            maya_file << "\tsetAttr \".man\" -type \"string\" \"Cam" << std::setw(4) << std::setfill('0') << i << "_mask\";\n";

            // Write image planes
            std::string path = m_images[i].m_filename_undistorted;
            std::string backslashes = "\\";
            while ((path.find(backslashes)) != std::string::npos) path.replace(path.find(backslashes), backslashes.length(), "/");

            maya_file << "createNode imagePlane -n \"ImagePlane" << std::setw(4) << std::setfill('0') << i << "\";\n";
            maya_file << "\tsetAttr \".fc\" 12;\n";
            maya_file << "\tsetAttr \".imn\" -type \"string\" \"" << path.c_str() << "\";\n";
            maya_file << "\tsetAttr \".cov\" -type \"short2\" " << m_images[i].GetWidth() << " " << m_images[i].GetHeight() << ";\n";
            maya_file << "\tsetAttr \".ag\" 0.5;\n";
            maya_file << "\tsetAttr \".d\" 3;\n";
            maya_file << "\tsetAttr \".s\" -type \"double2\" " << cap_w << " " << cap_h << ";\n";
            maya_file << "\tsetAttr \".c\" -type \"double3\" 0.0 0.0 0.0;\n";
            maya_file << "\tsetAttr \".w\" 30;\n";
            maya_file << "\tsetAttr \".h\" 30;\n";

            // Connect image plane to camera
            maya_file   << "connectAttr \"ImagePlane" << std::setw(4) << std::setfill('0') << i << ".msg\" \":Cam"
                        << std::setw(4) << std::setfill('0') << i << "Shape.ip\" -na;\n";
        }

        // Write particles
        maya_file << "createNode transform -n \"particle1\";\n";
        maya_file << "createNode particle -n \"particleShape1\" -p \"particle1\";\n";
        maya_file << "\taddAttr -ci true -sn \"lifespanPP\" -ln \"lifespanPP\" -bt \"life\" -dt \"doubleArray\";\n";
        maya_file << "\taddAttr -ci true -h true -sn \"lifespanPP0\" -ln \"lifespanPP0\" -bt \"life\" -dt \"doubleArray\";\n";
        maya_file << "\taddAttr -ci true -sn \"lifespan\" -ln \"lifespan\" -bt \"life\" -at \"double\";\n";
        maya_file << "\taddAttr -ci true -sn \"rgbPP\" -ln \"rgbPP\" -dt \"vectorArray\";\n";
        maya_file << "\taddAttr -ci true -h true -sn \"rgbPP0\" -ln \"rgbPP0\" -dt \"vectorArray\";\n";
        maya_file << "\taddAttr -is true -ci true -sn \"colorAccum\" -ln \"colorAccum\" -min 0 -max 1 -at \"bool\";\n";
        maya_file << "\taddAttr -is true -ci true -sn \"useLighting\" -ln \"useLighting\" -min 0 -max 1 -at \"bool\";\n";
        maya_file << "\taddAttr -is true -ci true -sn \"pointSize\" -ln \"pointSize\" -dv 2 -min 1 -max 60 -at \"long\";\n";
        maya_file << "\taddAttr -is true -ci true -sn \"normalDir\" -ln \"normalDir\" -dv 2 -min 1 -max 3 -at \"long\";\n";
        maya_file << "\tsetAttr -k off \".v\";\n";
        maya_file << "\tsetAttr \".gf\" -type \"Int32Array\" 0 ;\n";

        // Position
        maya_file << "\tsetAttr \".pos0\" -type \"vectorArray\" " << std::setw(4) << std::setfill('0') << num_points;
        for (const auto &point : m_points)
        {
            maya_file   << " " << point.m_pos.x() << " " << point.m_pos.y() << " " << point.m_pos.z();
        }
        maya_file << ";\n";

        // Color
        maya_file << "\tsetAttr \".rgbPP0\" -type \"vectorArray\" " << std::setw(4) << std::setfill('0') << num_points;
        for (const auto &point : m_points)
        {
            maya_file   << " " << point.m_color[0] << " " << point.m_color[1] << " " << point.m_color[2];
        }
        maya_file << ";\n";

        maya_file << "\tsetAttr -k on \".colorAccum\";\n";
        maya_file << "\tsetAttr -k on \".useLighting\";\n";
        maya_file << "\tsetAttr -k on \".pointSize\" 3;\n";
        maya_file << "\tsetAttr -k on \".normalDir\";\n";

        maya_file << "// End of Result.ma\n";
        maya_file.close();
    }
}