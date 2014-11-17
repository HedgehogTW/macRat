#ifndef MAINFRAME_H
#define MAINFRAME_H
#include "wxcrafter.h"
#include <wx/bitmap.h>
#include <wx/filehistory.h>

#include <deque>
#include "itkMetaImageIO.h"
#include "itkImageFileReader.h"

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Rat.h"

using namespace std;
using namespace cv;

#define PALETTE_SIZE 256


class MainFrame : public MainFrameBaseClass
{
public:
	MainFrame(wxWindow* parent);
	virtual ~MainFrame();

	inline int		getNumSlices() { return m_nSlices; }
	inline Size	    getImgSize()  { return m_szOutImg; }
	inline int		getBpp() { return m_nBpp; }
	inline uchar*	getPixelAddr() { return m_mOut.data; }
	inline int		getStep() { return m_mOut.step[0]; }

	void	updateOutData(Mat& mOut);
	Mat &   getCurrentMat(int idx) { return m_Rat.getSrcImg(idx); }
	Mat &   getResultMat(int idx) { return m_Rat.getResultImg(idx); }
	int		getCageHeight() { return m_Rat.m_nCageLineY; }

	void	recognizeLeftRight(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR);
	
	void 	smoothData(vector<double>& inData, vector<double>& outData, int bw=5);

	
	deque<Point>& 	getEyePts() { return m_dqEyePts; }
	deque<Point>& 	getEarPts() { return m_dqEarPts; }
	
	void 	gnuplotShow(const char* title, int nBeginLight, int nTwoLight, vector<double>& earLM, vector<double>& earRM, vector<double>& eye);
	
	static void myMsgOutput(wxString szFormat,...) {
		wxString strMsg;
		va_list argList;
		va_start(argList, szFormat);
		strMsg.PrintfV(szFormat, argList);
		va_end(argList); // 以上這幾行是用來處理類似printf 的參數

		m_pThis->m_richTextMsg->AppendText(strMsg);
		m_pThis->m_richTextMsg->ShowPosition(m_pThis->m_richTextMsg->GetLastPosition());
	}

	void OnExit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);

	void DeleteContents();
	
	static MainFrame *	m_pThis;
	
	
protected:
    virtual void OnRatLoadResult(wxCommandEvent& event);
    virtual void OnRatShowResults(wxCommandEvent& event);
    virtual void OnRatProcess(wxCommandEvent& event);
    virtual void OnMarkEars(wxCommandEvent& event);
    virtual void OnLeftButtonDown(wxMouseEvent& event);
    virtual void OnMarkEyes(wxCommandEvent& event);
	virtual void OnMouseMotion(wxMouseEvent& event);
	virtual void OnUpdateViewMsgPane(wxUpdateUIEvent& event);
	virtual void OnViewMsgPane(wxCommandEvent& event);
	virtual void OnMouseLDown(wxMouseEvent& event);
	virtual void OnFileOpen(wxCommandEvent& event);
	void OnMRUFile(wxCommandEvent& event);
	void openFile(wxString &dirName);

	wxFileHistory* 		m_FileHistory;
	wxString			m_Filename;


//////////////// Rat
//	RGBQUAD	m_Palette[PALETTE_SIZE];
	wxString m_strSourcePath;
	Mat		m_mOut;
	int		m_nBpp;
	int		m_nChannel;
	int		m_nDepth;
	Size	m_szOutImg;
	CRat	m_Rat;
	int		m_nSlices ;

////////////////////////////////mark eyes and ears
	bool    m_bMarkEye;
	bool    m_bMarkEar;
	deque<Point>  m_dqEyePts;
	deque<Point>  m_dqEarPts;
	bool	m_bFirstEyeIsLeft;
	bool	m_bFirstEarIsLeft;
	

	long m_nFrameSteps;	
};
#endif // MAINFRAME_H
