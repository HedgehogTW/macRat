#include "scrolled_image_component.h"
#include "MainFrame.h"
#include <wx/dcclient.h>

using namespace cv;

ScrolledImageComponent::ScrolledImageComponent(wxWindow* parent, wxWindowID id,  const wxPoint &pos, const wxSize &size, long style)
		: wxScrolledWindow(parent, id, pos, size, style)
{
	m_bitmap = NULL;
	//this->Bind(wxEVT_MOTION, &MyFrame::OnMouseMotion, this);
}

ScrolledImageComponent::~ScrolledImageComponent()
{
	if(m_bitmap != NULL)
		delete m_bitmap;
}
    
void ScrolledImageComponent::setImage(cv::Mat& mat)
{
	
	wxImage wxIm;
	cv::Mat  rgbOutput;
	int type = mat.type();
	int channel = mat.channels();
	
	bool ret = false;
	
	if(type ==CV_8UC1) {
		cvtColor(mat, rgbOutput, CV_GRAY2RGB);
		ret = wxIm.Create(mat.cols, mat.rows, rgbOutput.data, true);				
	}else if(type == CV_8UC3) {
			cvtColor(mat, rgbOutput, CV_BGR2RGB);
			ret = wxIm.Create(mat.cols, mat.rows, rgbOutput.data, true);			
	}else if(channel ==1){
		Mat  m8UC1;
		double min, max, a;
		cv::minMaxLoc(mat, &min, &max);
		a = 255./(max - min);
		mat.convertTo(m8UC1, CV_8UC1, a, -min*a );
		cvtColor(m8UC1, rgbOutput, CV_GRAY2RGB);
		ret = wxIm.Create(mat.cols, mat.rows, rgbOutput.data, true);		
	}

	if(ret)	{
		if(m_bitmap)  delete m_bitmap;
	
		m_bitmap = new wxBitmap( wxIm );	
		int w = wxIm.GetWidth();
		int h = wxIm.GetHeight();

		/* init scrolled area size, scrolling speed, etc. */
		//SetScrollbars(1,1, w, h, 0, 0);	
		SetVirtualSize( w, h );
		Refresh();
	}else wxLogMessage(wxT("wxIm.Create failed"));	
}
void ScrolledImageComponent::OnDraw(wxDC& dc)
{
	/* render the image - in a real app, if your scrolled area
	   is somewhat big, you will want to draw only visible parts,
	   not everything like below */
	if(m_bitmap)
		dc.DrawBitmap(*m_bitmap, 0, 0, false);
	
	// also check wxScrolledWindow::PrepareDC
	
	deque<Point> & eyePts = MainFrame::m_pThis->getEyePts();
	if(eyePts.size()>0) {
		dc.SetPen(*wxRED_PEN);
		dc.SetBrush(*wxRED_BRUSH);
		for(int i=0; i<eyePts.size(); i++) {
			dc.DrawCircle(eyePts[i].x, eyePts[i].y, 2);
		}
	}
	
	deque<Point> & earPts = MainFrame::m_pThis->getEarPts();
	if(earPts.size()>0) {
		dc.SetPen(*wxGREEN_PEN);
		dc.SetBrush(*wxGREEN_BRUSH);
		for(int i=0; i<earPts.size(); i++) {
			dc.DrawCircle(earPts[i].x, earPts[i].y, 2);
		}
	}	
	
	deque<Point> & abdoPts = MainFrame::m_pThis->getAbdoPts();
	if(abdoPts.size()>0) {
		dc.SetPen(*wxCYAN_PEN);
		dc.SetBrush(*wxGREEN_BRUSH);
		for(int i=0; i<abdoPts.size(); i++) {
			dc.DrawCircle(abdoPts[i].x, abdoPts[i].y, 2);
		}
	}	
}


