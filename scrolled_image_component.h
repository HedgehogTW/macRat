#ifndef SCROLLEDIMAGECOMPONENT_H
#define SCROLLEDIMAGECOMPONENT_H
#include <wx/scrolwin.h>
#include <wx/bitmap.h>
#include <wx/log.h> 
#include <wx/dc.h>
#include <opencv2/opencv.hpp>

class ScrolledImageComponent: public wxScrolledWindow
{
public:
    wxBitmap* m_bitmap;
   
public:

	ScrolledImageComponent(wxWindow* parent, wxWindowID id,  const wxPoint &pos, const wxSize &size, long style);     
    ~ScrolledImageComponent();
	
	void setImage(cv::Mat& mat);
    void OnDraw(wxDC& dc);

	int m_nWidth;
	int m_nHeight;
};

#endif // SCROLLEDIMAGECOMPONENT_H
