#include "wx/graphics.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/legacy.hpp"

#include "MainFrame.hpp"

void MainFrame::OnSelectPreviewImage(wxListEvent& event)
{
    GeneratePreviewImage(event.GetIndex());
}

void MainFrame::GeneratePreviewImage(int img_idx)
{
    const int width  = GetImageWidth(img_idx);
    const int height = GetImageHeight(img_idx);

    auto image = cv::imread(m_images[img_idx].m_filename);
    cv::cvtColor(image, image, CV_BGR2RGB);

    m_preview_image.Destroy();
    m_preview_image = wxImage{width, height, image.data, true}.Copy();

    DrawImagePreview(wxClientDC{m_window_image_preview});
}

void MainFrame::OnImagePreviewMouse(wxMouseEvent& event)
{
    //TODO: Implement useful stuff
}

void MainFrame::OnImagePreviewPaint(wxPaintEvent& event)
{
    DrawImagePreview(wxPaintDC{m_window_image_preview});
}

void MainFrame::OnImagePreviewResize(wxSizeEvent& event)
{
    m_window_image_preview->Refresh();
    event.Skip();
}

void MainFrame::DrawImagePreview(wxDC& dc)
{
    if (!dc.IsOk()) return;

    dc.Clear();
    const int win_width  = m_window_image_preview->GetClientSize().GetWidth();
    const int win_height = m_window_image_preview->GetClientSize().GetHeight();

    if (m_img_ctrl->GetSelectedItemCount())
    {
        const int img_width  = m_preview_image.GetWidth();
        const int img_height = m_preview_image.GetHeight();

        const int new_img_width  = (img_width  * win_height)     / img_height;
        const int new_img_height = (img_height * win_width)      / img_width;
        const int width_diff     = (win_width  - new_img_width)  / 2;
        const int height_diff    = (win_height - new_img_height) / 2;

        if (height_diff >= 0)
        {
            dc.DrawBitmap(wxBitmap{m_preview_image.Scale(win_width, new_img_height, wxIMAGE_QUALITY_HIGH)}, 0, height_diff);
        } else
        {
            dc.DrawBitmap(wxBitmap{m_preview_image.Scale(new_img_width, win_height, wxIMAGE_QUALITY_HIGH)}, width_diff, 0);
        }
    } else
    {
        auto gc = wxGraphicsContext::Create(m_window_image_preview);
        gc->SetPen(*wxLIGHT_GREY_PEN);

        auto path = gc->CreatePath();
        path.MoveToPoint(0.0, 0.0);
        path.AddLineToPoint(win_width, win_height);
        path.MoveToPoint(0.0, win_height);
        path.AddLineToPoint(win_width, 0.0);

        gc->StrokePath(path);

        double txt_width;
        gc->SetFont(wxFont{wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT)}, *wxBLACK);
        gc->GetTextExtent("Select image", &txt_width, nullptr);
        const double offset = (win_width - (int)txt_width) / 2.0;
        gc->DrawText("Select image", offset, 5.0);

        delete gc;
    }
}
