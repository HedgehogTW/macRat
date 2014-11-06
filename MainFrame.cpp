#include "MainFrame.h"
#include <wx/aboutdlg.h>
#include <wx/filedlg.h>
#include <wx/bitmap.h>
#include <wx/log.h>
#include <wx/config.h>
#include <opencv2/opencv.hpp>
#include <wx/dcclient.h>
#include <wx/dirdlg.h> 
#include <wx/msgdlg.h> 
#include <wx/filename.h>

#include "DlgOptical.h"
#include "gnuplot_i.hpp"

using namespace std;
using namespace cv;

MainFrame *	MainFrame::m_pThis=NULL;


MainFrame::MainFrame(wxWindow* parent)
	: MainFrameBaseClass(parent)
{
#if defined(__WXMAC__)
/*
	wxIcon  icon;
	wxBitmap bitmap(wxT("ratty32.png"), wxBITMAP_TYPE_PNG); 
	icon.CopyFromBitmap(bitmap); 
	SetIcon(icon);
*/
#else
	SetIcon(wxICON(frame_icon));
#endif

	m_pThis = this;
	m_menuItemViewMsgPane->Check(true);

	int statusWidth[4] = {250, 110, 40, 140};
	m_statusBar->SetFieldsCount(4, statusWidth);
	
	wxConfigBase *pConfig = wxConfigBase::Get();
	m_FileHistory = new wxFileHistory(9);
	m_FileHistory->UseMenu(m_menuFile);
	m_FileHistory->AddFilesToMenu(m_menuFile);
	m_FileHistory->Load(*pConfig);
	
	this->Connect(wxID_FILE1, wxID_FILE9, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnMRUFile), NULL, this);
	
	SetSize(700, 650);
	Center();
	
	// optical flow
	m_nIter = 3;
	m_nNeighbor = 3;
	m_nPyrLayers= 1;
	m_dblSigma = 0.8;
	m_nWinSize = 3;
	m_nFrameSteps = 2;	
	
	DeleteContents();

}

MainFrame::~MainFrame()
{
	DeleteContents();
	
	wxConfigBase *pConfig = wxConfigBase::Get();
	m_FileHistory->Save(*pConfig);
	delete m_FileHistory;	
	delete wxConfigBase::Set((wxConfigBase *) NULL);
}

void MainFrame::DeleteContents()
{
	// TODO: Add your specialized code here and/or call the base class
	m_nSlices = 0;
	
	m_bFirstEyeIsLeft = true;
	m_bFirstEarIsLeft = true;
	
	m_bMarkEye = false;
	m_bMarkEar = false;
	
	m_dqEyePts.clear();
	m_dqEarPts.clear();
}
void MainFrame::OnMRUFile(wxCommandEvent& event)
{
	wxString f(m_FileHistory->GetHistoryFile(event.GetId() - wxID_FILE1));
	if (!f.empty())  openFile(f);
}

void MainFrame::OnExit(wxCommandEvent& event)
{
	wxUnusedVar(event);
	Close();
}

void MainFrame::OnAbout(wxCommandEvent& event)
{
	wxUnusedVar(event);
	wxAboutDialogInfo info;
	info.SetCopyright(_("My MainFrame"));
	info.SetLicence(_("GPL v2 or later"));
	info.SetDescription(_("Short description goes here"));
	::wxAboutBox(info);
}
void MainFrame::OnFileOpen(wxCommandEvent& event)
{	
	wxString  strHistoryFile = wxEmptyString;
	if(m_FileHistory->GetCount() >0) 
		strHistoryFile= m_FileHistory->GetHistoryFile(0);
	
	static wxString strInitDir (strHistoryFile);
	wxString dirName = wxDirSelector("Choose a folder", strInitDir); //"E:\\Image_Data\\Mouse\\");
	if(dirName.empty())  return;
	m_FileHistory->AddFileToHistory(dirName);	
	strInitDir = dirName;
	openFile(dirName);
}
void MainFrame::openFile(wxString &dirName)
{
	DeleteContents();

	m_strSourcePath = dirName;
	myMsgOutput( "Load " + dirName + "\n");

	int num;
	bool  bRet = false;
	num = m_Rat.readData(dirName);

	if(num <=0)  {
		wxLogMessage("read images failed");
		return;
	}
	
	bRet = m_Rat.horizontalLine();

	if(bRet ==false) {
		wxLogMessage("cannot detect cage line");
		return;
	}
	
//	m_Rat.verticalLine();

	m_Rat.detectTwoLight();
	m_Rat.prepareData();
	m_nSlices = m_Rat.getNumFrames();	
	
	updateOutData(m_Rat.getSrcImg(0));


	myMsgOutput("After preprocessing, %d frames are used, size w%d, h%d\n",
		m_nSlices, m_Rat.m_szImg.width, m_Rat.m_szImg.height );	
	
}
void MainFrame::updateOutData(Mat& mOut)
{
	mOut.copyTo(m_mOut);

	m_nDepth = m_mOut.depth();
	m_nChannel = m_mOut.channels();
    m_nBpp = m_mOut.elemSize() * 8;
	m_szOutImg = Size(m_mOut.cols, m_mOut.rows);

	m_scrollWin->setImage(mOut);

	wxString strSize;
	strSize.Printf("%d x %d", mOut.cols, mOut.rows);
	
	m_statusBar->SetStatusText(strSize, 1);

	wxString strFormat;
	wxString strDepth;
	switch (m_nDepth) {
		case CV_8U: strDepth = "8U";  break;
		case CV_8S: strDepth = "8S";  break;
		case CV_16U: strDepth = "16U";  break;
		case CV_16S: strDepth = "16S";  break;
		case CV_32S: strDepth = "32S";  break;
		case CV_32F: strDepth = "32F";  break;
		case CV_64F: strDepth = "64F";  break;
		default: strDepth = "unknown"; break;
	}
	strFormat.Printf("%sC%d", strDepth, m_nChannel);
	m_statusBar->SetStatusText(strFormat, 2);

}


void MainFrame::OnMouseLDown(wxMouseEvent& event)
{
//	wxPoint pt = m_staticBitmap->GetLogicalPosition

}
void MainFrame::OnViewMsgPane(wxCommandEvent& event)
{

	wxAuiPaneInfo &pane =  m_auimgr11->GetPane(wxT("MsgPane"));
	pane.Show(!pane.IsShown());

	m_auimgr11->Update();
}

void MainFrame::OnUpdateViewMsgPane(wxUpdateUIEvent& event)
{
	wxAuiPaneInfo &pane =  m_auimgr11->GetPane(wxT("MsgPane"));
	event.Check(pane.IsShown());
}

void MainFrame::OnMouseMotion(wxMouseEvent& event)
{
	if (getNumSlices() <= 0)  {
		return;
	}
	
	wxClientDC *pDC = new wxClientDC(this);
	wxPoint pt1 = event.GetLogicalPosition(*pDC);
	wxPoint pt;
	m_scrollWin->CalcUnscrolledPosition(pt1.x, pt1.y, &pt.x, &pt.y);	

	Mat mat = getCurrentMat(0);
	
	if (pt.x >= mat.cols || pt.y >= mat.rows)
		return;
	
	int type = mat.type();
	wxString str;

	if(type ==CV_8UC3) {
		cv::Vec3b c;
		c = mat.at<cv::Vec3b>(pt.y, pt.x);
		uchar b = c.val[0];
		uchar g = c.val[1];
		uchar r = c.val[2];
		str.Printf("(%d, %d)  [%d %d %d]", pt.x, pt.y, r & 0xFF, g & 0xFF, b & 0xFF);
	}else if(type ==CV_8UC4) {
		cv::Vec4b c;
		c = mat.at<cv::Vec4b>(pt.y, pt.x);
		uchar b = c.val[0];
		uchar g = c.val[1];
		uchar r = c.val[2];
		uchar u = c.val[3];
		str.Printf("(%d, %d)  [%d %d %d %d]", pt.x, pt.y, r&0xFF, g&0xFF, b&0xFF, u&0xFF);
	}else if (type == CV_8UC1) {
		short v1 =  mat.at<uchar>(pt.y, pt.x);
		str.Printf("(%d, %d) idx %d", pt.x, pt.y, v1);
	}
	
	m_statusBar->SetStatusText(str, 3);
}

void MainFrame::OnMarkEyes(wxCommandEvent& event)
{
	if(event.IsChecked()) {
		if(m_bMarkEar)  {
			m_toggleButtonMarkEars->SetValue(false);
			m_bMarkEar = false;
		}
		m_bMarkEye = true;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_CROSS ));
	}else {
		m_bMarkEye = false;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_ARROW ));
	}	
}
void MainFrame::OnMarkEars(wxCommandEvent& event)
{
	if(event.IsChecked()) {
		if(m_bMarkEye) { 
			m_toggleButtonMarkEyes->SetValue(false);
			m_bMarkEye = false;
		}
		m_bMarkEar = true;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_CROSS ));
	}else {
		m_bMarkEar = false;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_ARROW ));
	}		
}
void MainFrame::OnLeftButtonDown(wxMouseEvent& event)
{
	if(m_bMarkEye==false && m_bMarkEar == false)  return;
	if (getNumSlices() <= 0)  return;
	
	wxClientDC *pDC = new wxClientDC(this);
	wxPoint pt1 = event.GetLogicalPosition(*pDC);
	wxPoint pt;
	m_scrollWin->CalcUnscrolledPosition(pt1.x, pt1.y, &pt.x, &pt.y);
	
	Mat mat = getCurrentMat(0);
	
	if (pt.x >= mat.cols || pt.y >= mat.rows)
		return;
	if(m_bMarkEye) {
		Point ptEye = Point(pt.x, pt.y);
		m_dqEyePts.push_back(ptEye);
		if(m_dqEyePts.size()>2) 
			 m_dqEyePts.pop_front();
	}else if(m_bMarkEar) {
		Point ptEar = Point(pt.x, pt.y);
		m_dqEarPts.push_back(ptEar);
		if(m_dqEarPts.size()>2) 
			 m_dqEarPts.pop_front();
	}
	
	Refresh();
}
void MainFrame::OnRatProcess(wxCommandEvent& event)
{
	if(m_dqEyePts.size()!=2) {
		wxMessageBox("Select eye position", "error");
		return;
	}
	if(m_dqEarPts.size()!=2) {
		wxMessageBox("Select ear position", "error");
		return;
	}
	
	DlgOptical  dlg(m_nPyrLayers, m_nWinSize, m_nIter, m_nNeighbor, m_dblSigma, m_nFrameSteps, this);
	if(dlg.ShowModal()!=wxID_OK)  return;
	
	m_nIter  = dlg.getIter();
	m_nNeighbor = dlg.getNeighbor();
	m_nPyrLayers = dlg.getPyrLayers();
	m_dblSigma = dlg.getSigma();
	m_nWinSize = dlg.getWinSize();
	m_nFrameSteps = dlg.getFrameSteps();	

	myMsgOutput("Opticalflow: iter %d, neighbor %d, PyrLayers %d, Sigma %.2f, winSize %d, frame steps %d\n",
		m_nIter, m_nNeighbor, m_nPyrLayers,  m_dblSigma, m_nWinSize, m_nFrameSteps);
		
		
	Point ptEyeL, ptEyeR, ptEarL, ptEarR;
	ptEyeL = m_dqEyePts[0];
	ptEyeR = m_dqEyePts[1];
	ptEarL = m_dqEarPts[0];
	ptEarR = m_dqEarPts[1];
	recognizeLeftRight(ptEyeL, ptEyeR, ptEarL, ptEarR);	
	
	
	int nFrameNum = -1;
	wxBeginBusyCursor();

	m_Rat.findMouseEyes(nFrameNum, ptEyeL, ptEyeR);
	m_Rat.findMouseEars(nFrameNum, ptEarL, ptEarR);

	m_Rat.Opticalflow(m_nPyrLayers, m_nWinSize, m_nIter, m_nNeighbor, m_dblSigma, m_nFrameSteps );
	m_Rat.motionAnalysis(m_nFrameSteps);
	//m_Rat.graylevelDiff(m_nFrameSteps);
	updateOutData(m_Rat.getResultImg(0));
	wxEndBusyCursor();

	wxFileName fileName = m_strSourcePath;
	wxString  fName = fileName.GetName();
	wxFileName dataName(m_strSourcePath, "_"+fName+ "_motion.csv");
	
//	int idx = m_strSourcePath.rfind('\\');
//	wxString pathName = m_strSourcePath.Right(m_strSourcePath.Len() - idx-1);
//	wxString  fname;
//	fname.Printf("%s\\_%s_motion.csv", m_strSourcePath, pathName);
	myMsgOutput("output result file: " + dataName.GetFullPath() + "\n");
	
	FILE* fp = fopen(dataName.GetFullPath(), "w");
	if(fp==NULL) {
		wxMessageBox("Open output file failed:"+dataName.GetFullName(), "error");
		return;
	}
	int  n = m_Rat.m_vecLEarMotion.size();

	fprintf(fp, "%d, %d, %d\n",n, m_Rat.m_idxLightBegin, m_Rat.m_idxTwoLight);
	for(int i=0;i<n; i++) {
		fprintf(fp, "%.3f, %.3f, %.3f\n", 
			m_Rat.m_vecLEarMotion[i],
			m_Rat.m_vecREarMotion[i],
			m_Rat.m_vecEyeMotion[i]);
	}
	fclose(fp);	
}

void MainFrame::recognizeLeftRight(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR)
{
	Point eyeCenter = (ptEyeL + ptEyeR) *0.5;
	Point earCenter = (ptEarL + ptEarR) *0.5;

	cv::Point u, v;
	float angle;

	// for ear
	u = earCenter - eyeCenter;
	v = ptEarL - eyeCenter;
	angle = asin(u.cross(v) / (norm(u)*norm(v)));
	if (angle < 0) {
		Point t = ptEarR;
		ptEarR = ptEarL;
		ptEarL = t;
		m_bFirstEarIsLeft = false;
	}

	// for eyes
	v = ptEyeL - earCenter;
	angle = asin(u.cross(v) / (norm(u)*norm(v)));
	if (angle < 0) {
		Point t = ptEyeR;
		ptEyeR = ptEyeL;
		ptEyeL = t;
		m_bFirstEyeIsLeft = false;
	}
//	gpOutput->ShowMessage("angle %.2f\n", angle);
}


void MainFrame::gnuplotShow(const char* title, int nBeginLight, int nTwoLight, 
	vector<double>& earLM, vector<double>& earRM, vector<double>& eye)
{
	int max_y;
	if (earLM.size() <= 0 || earRM.size() <= 0 || eye.size() <= 0) {
		wxMessageBox("gnuplotShow:: no data", "Error");
		return;
	}
	double maxY1 = *std::max_element(earLM.begin(), earLM.end());
	double maxY2 = *std::max_element(earRM.begin(), earRM.end());
	max_y = MAX(maxY1, maxY2);
	max_y = MAX(max_y, 10);

	try{
		static Gnuplot g1("lines");
		g1.reset_all();

		g1.set_title(title);
		g1.set_grid().set_yrange(0, max_y);

		g1.set_style("lines").plot_x(eye, "Eye");
		g1.set_style("lines").plot_x(earLM, "Left Ear");
		g1.set_style("lines").plot_x(earRM, "Right Ear");
		//g1.set_style("lines").plot_x(earLG, "LEarGrayDiff");
		//g1.set_style("lines").plot_x(earRG, "REarGrayDiff");

		if (nBeginLight > 0 && nTwoLight>0) {
			std::vector<double> x_light;
			std::vector<double> y;
			y.push_back(max_y);
			y.push_back(max_y);
			x_light.push_back(nBeginLight);
			x_light.push_back(nTwoLight);
			g1.set_style("impulses").plot_xy(x_light, y, "lantern");
		}
	}
	catch (...) {
		wxMessageBox("gnuplot error, Do you install gnuplot, and set PATH correctly?", "Error");
		return;
	}
}
void MainFrame::OnRatShowResults(wxCommandEvent& event)
{
	if (m_Rat.m_vecEyeMotion.empty()) {
		wxMessageBox("no data", "Error");
		return;
	}

	wxFileName fileName = m_strSourcePath;
	gnuplotShow(fileName.GetName(), m_Rat.m_idxLightBegin, m_Rat.m_idxTwoLight,
		m_Rat.m_vecLEarMotion, m_Rat.m_vecREarMotion, m_Rat.m_vecEyeMotion);
}
void MainFrame::OnRatLoadResult(wxCommandEvent& event)
{
	wxString wildcard  = "csv files (*.csv)|*.csv";
	wxString filename = wxFileSelector("Choose a file to open",wxEmptyString, wxEmptyString, wxEmptyString, wildcard, wxFD_FILE_MUST_EXIST|wxFD_OPEN);
	if ( filename.empty() )  return;
	

	int nBeginLight, nTwoLight, numRec;
	FILE* fp = fopen(filename, "r");
	if (fp == NULL) return;
	int r = fscanf(fp, "%d, %d, %d", &numRec, &nBeginLight, &nTwoLight);
	if (r != 3) {
		fclose(fp);		
		wxMessageBox("read lantern info error", "Error");
		return;
	}

	int num = 0;
	double a, b, c;
	while (!feof(fp)) {
		int r = fscanf(fp, "%lf, %lf, %lf", &a, &b, &c);
		if (r != 3) break;
		num++;
	}

	myMsgOutput("read %d records\n", num);
	if (num != numRec)  {
		fclose(fp);
		wxMessageBox("read motion data error", "Error");
		return;
	}

	vector <double> earLM, earRM, eye;

	rewind(fp);
	fscanf(fp, "%d, %d, %d", &numRec, &nBeginLight, &nTwoLight);
	num = 0;
	for (int i = 0; i<numRec; i++) {
		int r = fscanf(fp, "%lf, %lf, %lf", &a, &b, &c);
		if (r != 3) break;
		earLM.push_back(a);
		earRM.push_back(b);
		eye.push_back(c);
		num++;
	}
	fclose(fp);
	if (num != numRec)  {
		fclose(fp);
		wxMessageBox("read motion data error", "Error");
		return;
	}

	wxFileName fileName = filename;

	gnuplotShow(fileName.GetName(), nBeginLight, nTwoLight, earLM, earRM, eye);	
}
