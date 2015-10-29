#ifndef MAINFRAME_H
#define MAINFRAME_H
#include "wxcrafter.h"
#include <wx/bitmap.h>
#include <wx/filehistory.h>

#include <deque>
//#include "itkMetaImageIO.h"
//#include "itkImageFileReader.h"

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Rat.h"

using namespace std;
//using namespace cv;

#define PALETTE_SIZE 256
 

class MainFrame : public MainFrameBaseClass
{
public:
	MainFrame(wxWindow* parent);
	virtual ~MainFrame();

	inline int		getNumSlices() { return m_nSlices; }
	inline cv::Size	getImgSize()  { return m_szOutImg; }
	inline int		getBpp() { return m_nBpp; }
	inline uchar*	getPixelAddr() { return m_mOut.data; }
	inline int		getStep() { return m_mOut.step[0]; }

	bool	inputDialog();
	void	writeMarks();
	void	getConfigData(MyConfigData& data)  { data = m_configData; }
	void	setConfigData(MyConfigData& data)  { m_configData = data; }
	
	void	updateOutData(cv::Mat& mOut);
	cv::Mat &   getCurrentMat(int idx) { return m_Rat.getSrcImg(idx); }
	cv::Mat &   getResultMat(int idx) { return m_Rat.getResultImg(idx); }
//	Mat &   getFlowMat(int idx) { return m_Rat.getFlowImg(idx); }
	int		getCageHeight() { return m_Rat.m_nCageLine; }

	void	recognizeLeftRight(cv::Point& ptEyeL, cv::Point& ptEyeR, cv::Point& ptEarL, cv::Point& ptEarR);
	
	void 	getEyePts(cv::Point& eyeL, cv::Point& eyeR) { eyeL = m_ptEyeL; eyeR = m_ptEyeR; }
	void 	getEarPts(cv::Point& earL, cv::Point& earR) { earL = m_ptEarL; earR = m_ptEarR;}
	int 	getBellyPts(cv::Point& abRed, cv::Point& abCyan) { abRed = m_ptBellyRed; abCyan = m_ptBellyCyan; return m_Rat.m_BigRedPdf; }
	deque<cv::Point>& getBellyPts(	cv::Point& ptMostBelly) { 	ptMostBelly = m_ptMostBelly; return m_dqBellyPts1; }
	
	bool    isViewMarks() { return m_bViewMarks; };
    
	int		getCageline() { return m_nCageLine; }
	bool	getCroppedStatus()  { return m_bCutTop; }
	bool	preprocessing();
	void 	readMarks(wxString &dirName);
	void	readDirList(wxArrayString& dataDirs);
	
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
    virtual void OnBatchProcess(wxCommandEvent& event);
    virtual void OnRatProcess(wxCommandEvent& event);
    virtual void OnUpdateViewMarks(wxUpdateUIEvent& event);
    virtual void OnViewMarks(wxCommandEvent& event);
    virtual void OnRatAbdomen(wxCommandEvent& event);
    virtual void OnToolsCleanOutput(wxCommandEvent& event);
    virtual void OnMouseLButtonDown(wxMouseEvent& event);
    virtual void OnMouseRButtonDown(wxMouseEvent& event);
    virtual void OnMarkCageline(wxCommandEvent& event);
    virtual void OnRatBelly(wxCommandEvent& event);
    virtual void OnMarkBelly(wxCommandEvent& event);
    virtual void OnViewFolderImage(wxCommandEvent& event);
    virtual void OnView2DData(wxCommandEvent& event);
    virtual void OnView3DData(wxCommandEvent& event);
    virtual void OnViewResultSeries(wxCommandEvent& event);
    virtual void OnUpdateViewSeries(wxUpdateUIEvent& event);
    virtual void OnViewSeries(wxCommandEvent& event);
    virtual void OnEditClearMarks(wxCommandEvent& event);
    virtual void OnRatLoadResult(wxCommandEvent& event);
    virtual void OnRatShowResults(wxCommandEvent& event);
    virtual void OnMarkEars(wxCommandEvent& event);

    virtual void OnMarkEyes(wxCommandEvent& event);
	virtual void OnMouseMotion(wxMouseEvent& event);
	virtual void OnUpdateViewMsgPane(wxUpdateUIEvent& event);
	virtual void OnViewMsgPane(wxCommandEvent& event);
//	virtual void OnMouseLDown(wxMouseEvent& event);
	virtual void OnFileOpen(wxCommandEvent& event);
	void OnMRUFile(wxCommandEvent& event);
	void openFile(wxString &dirName);

	wxFileHistory* 		m_FileHistory;
	wxString			m_Filename;

///////////////////// config
	MyConfigData	m_configData;
	wxLog* 			m_logger;
	wxLog*			m_old_logger;
	FILE* 			m_fpLog;

	wxString		m_strBatchDir;
//////////////// Rat
//	RGBQUAD	m_Palette[PALETTE_SIZE];
	wxString m_strSourcePath;
	cv::Mat		m_mOut;
	int		m_nBpp;
	int		m_nChannel;
	int		m_nDepth;
	cv::Size	m_szOriSize;
	cv::Size	m_szOutImg;
	CRat	m_Rat;
	int		m_nSlices ;
	int		m_nCageLine;
	int		m_nUserLED2;
	bool	m_bCutTop;
	bool	m_bHasCrop;
////////////////////////////////mark eyes and ears
	bool    m_bMarkEye;
	bool    m_bMarkEar;
	bool    m_bMarkBelly;
	bool    m_bMarkCageline;
    bool    m_bViewMarks;
	
//	double	m_gainHead;
//	double	m_gainBelly;
	
	deque<cv::Point>  m_dqEyePts;
	deque<cv::Point>  m_dqEarPts;
	deque<cv::Point>  m_dqBellyPts;
	deque<cv::Point>  m_dqBellyPts1;
	cv::Point 	m_ptEyeL, m_ptEyeR, m_ptEarL, m_ptEarR;
	cv::Point 	m_ptBellyRed, m_ptBellyCyan;
	cv::Point  m_ptMostBelly;


};
#endif // MAINFRAME_H
