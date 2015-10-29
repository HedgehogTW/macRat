#include "scrolled_image_component.h"
#include "MainFrame.h"
#include <wx/dcclient.h>

using namespace cv;

ScrolledImageComponent::ScrolledImageComponent(wxWindow* parent, wxWindowID id,  const wxPoint &pos, const wxSize &size, long style)
		: wxScrolledWindow(parent, id, pos, size, style)
{
	m_bitmap = NULL;
	m_nWidth = 0;
	m_nHeight = 0;
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

		m_nWidth = w;
		m_nHeight = h;
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
	int nCageline = MainFrame::m_pThis->getCageline();
	bool bCropped = MainFrame::m_pThis->getCroppedStatus();
	if(bCropped==false &&  nCageline >=0) {
		dc.SetPen(*wxYELLOW_PEN);
		dc.DrawLine(wxPoint(0, nCageline), wxPoint(m_nWidth, nCageline));
	}
    
    bool bViewMarks = MainFrame::m_pThis->isViewMarks();
    if(! bViewMarks)  return;
    
	Point 	ptEyeL, ptEyeR, ptEarL, ptEarR;
	
	MainFrame::m_pThis->getEyePts(ptEyeL, ptEyeR);
	if(ptEyeL.x != 0 ) {
		dc.SetPen(*wxRED_PEN);
		dc.SetBrush(*wxRED_BRUSH);
		dc.DrawCircle(ptEyeL.x, ptEyeL.y, 2);
		dc.DrawCircle(ptEyeR.x, ptEyeR.y, 2);
	}
	
	MainFrame::m_pThis->getEarPts(ptEarL, ptEarR);
	if(ptEarL.x != 0 ) {
		dc.SetPen(*wxYELLOW_PEN);
		dc.SetBrush(*wxYELLOW_BRUSH);
		dc.DrawCircle(ptEarL.x, ptEarL.y, 3);
		dc.DrawCircle(ptEarR.x, ptEarR.y, 3);
	}	

/*
	deque<cv::Point>& dqBellyPts =  MainFrame::m_pThis->getBellyPts();
	int sz = dqBellyPts.size();
	if(sz > 1) {
		wxPoint * pBellyPts = new wxPoint[sz];
		for(int i=0; i<sz; i++) {
			pBellyPts[i].x = dqBellyPts[i].x;
			pBellyPts[i].y = dqBellyPts[i].y;
		}
		dc.SetPen(*wxCYAN_PEN);
		dc.SetBrush(*wxTRANSPARENT_BRUSH);
		dc.DrawPolygon(sz, pBellyPts);
		delete [] pBellyPts;
	
	}
*/	
    Point 	ptBellyRed, ptBellyCyan;
	int bigRedPdf = MainFrame::m_pThis->getBellyPts(ptBellyRed, ptBellyCyan);
	if(ptBellyRed.x != 0 ) {
        if(bigRedPdf==-1 || bigRedPdf==1) {
            dc.SetPen(*wxRED_PEN);
            dc.SetBrush(*wxRED_BRUSH);
            dc.DrawCircle(ptBellyRed.x, ptBellyRed.y, 2);		
        }
        if(bigRedPdf==-1 || bigRedPdf==0) {
            dc.SetPen(*wxCYAN_PEN);
            dc.SetBrush(*wxCYAN_BRUSH);
            dc.DrawCircle(ptBellyCyan.x, ptBellyCyan.y, 2);
        }
	}	
	 
	//MainFrame:: myMsgOutput("ptBellyBo y %d\n", ptBellyBo.y);
}


