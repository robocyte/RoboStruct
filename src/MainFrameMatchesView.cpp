#include "wx/graphics.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/legacy.hpp"

#include "MainFrame.hpp"

void MainFrame::OnSelectMatchImage(wxCommandEvent& event)
{
    GenerateMatchImage();
    m_pane_matches_view->Refresh();
}

void MainFrame::GenerateMatchImage()
{
    if (!m_sfm_done) return;
    int left_idx  = m_cb_matches_left->GetSelection();
    int right_idx = m_cb_matches_right->GetSelection();

    if (left_idx == -1 || right_idx == -1 || (left_idx == right_idx)) return;

    auto left_img  = cv::imread(m_images[left_idx].m_filename);
    auto right_img = cv::imread(m_images[right_idx].m_filename);

    cv::cvtColor(left_img, left_img, CV_BGR2RGB);
    cv::cvtColor(right_img, right_img, CV_BGR2RGB);

    cv::Mat matches_img;
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> kpts1, kpts2;

    const auto& left_keys  = m_images[left_idx].m_keys;
    const auto& right_keys = m_images[right_idx].m_keys;

    kpts1.reserve(left_keys.size());
    kpts2.reserve(right_keys.size());

    float left_x_correction_factor  = 0.5 * m_images[left_idx].GetWidth();
    float left_y_correction_factor  = 0.5 * m_images[left_idx].GetHeight()  - 1.0;
    float right_x_correction_factor = 0.5 * m_images[right_idx].GetWidth();
    float right_y_correction_factor = 0.5 * m_images[right_idx].GetHeight() - 1.0;

    for (const auto &key : left_keys)
    {
        float x = key.m_coords.x() + left_x_correction_factor;
        float y = left_y_correction_factor - key.m_coords.y();
        kpts1.push_back(cv::KeyPoint(x, y, 1.0f));
    }

    for (const auto &key : right_keys)
    {
        float x = key.m_coords.x() + right_x_correction_factor;
        float y = right_y_correction_factor - key.m_coords.y();
        kpts2.push_back(cv::KeyPoint(x, y, 1.0f));
    }

    const auto &list = m_matches.GetMatchList(GetMatchIndex(left_idx, right_idx));

    matches.reserve(list.size());
    for (const auto &match : list) matches.push_back(cv::DMatch(match.first, match.second, 0.0f));

    cv::Scalar_<double> line_colour;
    if (m_options.draw_coloured_lines)  line_colour = cv::Scalar_<double>::all(-1);
    else                                line_colour = cv::Scalar_<double>::all(255);


    if (m_options.draw_matches_only)
    {
        cv::drawMatches(left_img, kpts1, right_img, kpts2, matches, matches_img, line_colour, cv::Scalar_<double>::all(80.0), cv::Mat{}, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    } else
    {
        cv::drawMatches(left_img, kpts1, right_img, kpts2, matches, matches_img, line_colour, cv::Scalar_<double>::all(80.0));
    }

    m_matches_image.Destroy();
    m_matches_image = wxImage{matches_img.cols, matches_img.rows, matches_img.data, true}.Copy();
}

void MainFrame::OnMatchesViewPaint(wxPaintEvent& event)
{
    wxPaintDC dc{m_pane_matches_view};
    DrawMatches(dc);
}

void MainFrame::OnMatchesViewResize(wxSizeEvent& event)
{
    m_pane_matches_view->Refresh();
    event.Skip();
}

void MainFrame::DrawMatches(wxDC& dc)
{
    if (!dc.IsOk()) return;

    dc.Clear();
    int win_width  = m_pane_matches_view->GetClientSize().GetX();
    int win_height = m_pane_matches_view->GetClientSize().GetY();

    int left_idx   = m_cb_matches_left->GetSelection();
    int right_idx  = m_cb_matches_right->GetSelection();

    if (left_idx != -1 && right_idx != -1 && (left_idx != right_idx))
    {
        int img_width  = m_matches_image.GetWidth();
        int img_height = m_matches_image.GetHeight();

        int new_img_width  = (img_width  * win_height)     / img_height;
        int new_img_height = (img_height * win_width)      / img_width;
        int width_diff     = (win_width  - new_img_width)  / 2;
        int height_diff    = (win_height - new_img_height) / 2;

        if (height_diff >= 0)
        {
            dc.DrawBitmap(wxBitmap{m_matches_image.Scale(win_width, new_img_height, wxIMAGE_QUALITY_HIGH)}, 0, height_diff);
        } else
        {
            dc.DrawBitmap(wxBitmap{m_matches_image.Scale(new_img_width, win_height, wxIMAGE_QUALITY_HIGH)}, width_diff, 0);
        }
    } else
    {
        auto gc = wxGraphicsContext::Create(m_pane_matches_view);
        gc->SetPen(*wxLIGHT_GREY_PEN);

        auto path = gc->CreatePath();
        path.MoveToPoint(0.0, 0.0);
        path.AddLineToPoint(win_width, win_height);
        path.MoveToPoint(0.0, win_height);
        path.AddLineToPoint(win_width, 0.0);

        gc->StrokePath(path);

        double txt_width;
        gc->SetFont(wxFont{wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT)}, *wxBLACK);
        gc->GetTextExtent("Select two different images", &txt_width, nullptr);
        double offset = (win_width - (int)txt_width) / 2.0;
        gc->DrawText("Select two different images", offset, 5.0);

        delete gc;
    }
}
