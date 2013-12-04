#include <wx/graphics.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/legacy.hpp"

#include "MainFrame.hpp"
#include "ImageData.hpp"

void MainFrame::OnSelectPreviewImage(wxListEvent& event)
{
    this->GeneratePreviewImage(event.GetIndex());
}

void MainFrame::GeneratePreviewImage(int img_idx)
{
    int width =  GetImageWidth(img_idx);
    int height = GetImageHeight(img_idx);

    auto image = cv::imread(m_images[img_idx].m_filename);
    cv::cvtColor(image, image, CV_BGR2RGB);

    m_preview_image.Destroy();
    m_preview_image = wxImage(width, height, image.data, true).Copy();

    wxClientDC dc(m_window_image_preview);
    DrawImgPreview(dc);
}

void MainFrame::OnImagePreviewMouse(wxMouseEvent& event)
{
    //TODO: Implement useful stuff
}

void MainFrame::OnImagePreviewPaint(wxPaintEvent& event)
{
    wxPaintDC dc(m_window_image_preview);
    DrawImgPreview(dc);
}

void MainFrame::OnImagePreviewResize(wxSizeEvent& event)
{
    m_window_image_preview->Refresh();
    event.Skip();
}

void MainFrame::DrawImgPreview(wxDC& dc)
{
    if (!dc.IsOk()) return;

    dc.Clear();
    int win_width = m_window_image_preview->GetClientSize().GetX();
    int win_height = m_window_image_preview->GetClientSize().GetY();

    if (m_img_ctrl->GetSelectedItemCount())
    {
        int img_width = m_preview_image.GetWidth();
        int img_height = m_preview_image.GetHeight();

        int new_img_width  = (img_width  * win_height)     / img_height;
        int new_img_height = (img_height * win_width)      / img_width;
        int width_diff     = (win_width  - new_img_width)  / 2;
        int height_diff    = (win_height - new_img_height) / 2;

        if (height_diff >= 0)
        {
            dc.DrawBitmap(wxBitmap(m_preview_image.Scale(win_width, new_img_height, wxIMAGE_QUALITY_HIGH)), 0, height_diff);
        } else
        {
            dc.DrawBitmap(wxBitmap(m_preview_image.Scale(new_img_width, win_height, wxIMAGE_QUALITY_HIGH)), width_diff, 0);
        }
    } else
    {
        auto *gc = wxGraphicsContext::Create(m_window_image_preview);
        gc->SetPen(*wxLIGHT_GREY_PEN);

        auto path = gc->CreatePath();
        path.MoveToPoint(0.0, 0.0);
        path.AddLineToPoint(win_width, win_height);
        path.MoveToPoint(0.0, win_height);
        path.AddLineToPoint(win_width, 0.0);

        gc->StrokePath(path);

        double txt_width;
        gc->SetFont(wxFont(wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT)), *wxBLACK);
        gc->GetTextExtent("Select image", &txt_width, nullptr);
        double offset = (win_width - (int)txt_width) / 2.0;
        gc->DrawText("Select image", offset, 5.0);

        delete gc;
    }
}
