#include "MainFrame.h"
#include <wx/aboutdlg.h>
#include <wx/filedlg.h>
#include <wx/bitmap.h>
#include <wx/log.h>
#include <wx/config.h>

#include <wx/dcclient.h>
#include <wx/dirdlg.h> 
#include <wx/msgdlg.h> 
#include <wx/filename.h>
#include <wx/utils.h> 
#include <wx/dir.h>

#include <opencv2/opencv.hpp>
#include "KDE.h"
#include "MyUtil.h"
#include "gnuplot_i.h"

using namespace std;
using namespace cv;

vector <Mat> gvecMat;

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
	wxBell();
}
void MainFrame::openFile(wxString &dirName)
{
	DeleteContents();

	m_strSourcePath = dirName;
	myMsgOutput( "Load " + dirName + "\n");

	bool  bRet = false;
	if(m_Rat.readData(dirName) <=0) 
		return;

	bRet = m_Rat.horizontalLine();

	if(bRet ==false) {
		wxLogMessage("cannot detect cage line");
		return;
	}
	
//	m_Rat.verticalLine(); 

	m_Rat.detectTwoLight();
	m_Rat.prepareData();
	m_nSlices = m_Rat.getNumFrames();	
	
	wxFileName fileName = dirName;
	wxFileName dataName(dirName, "_eye_earMarks.txt");
	if(dataName.IsFileReadable() ==true) { 	
		Point ptEyeL, ptEyeR, ptEarL, ptEarR;
		int n;
		FILE* fp = fopen(dataName.GetFullPath(), "r");
		n = fscanf(fp, "%d %d %d %d\n", &ptEyeL.x, &ptEyeL.y, &ptEyeR.x, &ptEyeR.y );
		if(n==4) {
			m_dqEyePts.push_back(ptEyeL);
			m_dqEyePts.push_back(ptEyeR);
		}
			
		n = fscanf(fp, "%d %d %d %d\n", &ptEarL.x, &ptEarL.y, &ptEarR.x, &ptEarR.y);
		if(n==4) {
			m_dqEarPts.push_back(ptEarL);
			m_dqEarPts.push_back(ptEarR);
		}	
		fclose(fp);
	}
	
	updateOutData(m_Rat.getSrcImg(0));


	wxString title = dirName.AfterLast('\\');
	SetTitle(wxString("Dataset: ") << title);

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
void MainFrame::OnEditClearMarks(wxCommandEvent& event)
{
	m_bMarkEar = false;
	m_bMarkEye = false;
	m_toggleButtonMarkEars->SetValue(false);
	m_toggleButtonMarkEyes->SetValue(false);
	
	m_dqEyePts.clear();
	m_dqEarPts.clear();
	Refresh();
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
	
	Point ptEyeL, ptEyeR, ptEarL, ptEarR;
	ptEyeL = m_dqEyePts[0];
	ptEyeR = m_dqEyePts[1];
	ptEarL = m_dqEarPts[0];
	ptEarR = m_dqEarPts[1];
	
	wxBeginBusyCursor();
	m_Rat.process1(ptEyeL, ptEyeR, ptEarL, ptEarR);
	
//	m_Rat.findMouseEyes(nFrameNum, ptEyeL, ptEyeR);
//	m_Rat.findMouseEars(nFrameNum, ptEarL, ptEarR);
	
	
	updateOutData(m_Rat.getResultImg(0));
	
//	m_Rat.saveEarImage();
	
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
/*	
	int  n = m_Rat.m_vecLEarMotion.size();

	fprintf(fp, "%d, %d, %d\n",n, m_Rat.m_idxLightBegin, m_Rat.m_idxTwoLight);
	for(int i=0;i<n; i++) {
		fprintf(fp, "%.3f, %.3f, %.3f\n", 
			m_Rat.m_vecLEarMotion[i],
			m_Rat.m_vecREarMotion[i],
			m_Rat.m_vecEyeMotion[i]);
	}
*/ 
	fclose(fp);	
	

		//m_Rat.m_vecLEarGrayDiff, m_Rat.m_vecREarGrayDiff, m_Rat.m_vecLEyeGrayDiff);
	wxBell();	
}



void MainFrame::OnRatShowResults(wxCommandEvent& event)
{

	if (m_Rat.m_vecLEarGrayDiff.empty()) {
		wxMessageBox("no data", "Error");
		return;
	}
	
	Gnuplot gnuPlot("lines");
	wxFileName fileName = m_strSourcePath;
	_gnuplotLED(gnuPlot, fileName.GetName(), m_Rat.m_idxLightBegin, m_Rat.m_idxTwoLight);
	_gnuplotLine(gnuPlot, "Left Ear", m_Rat.m_vecLEarGrayDiff);
	//m_Rat.m_vecLEarGrayDiff, m_Rat.m_vecREarGrayDiff, m_Rat.m_vecLEyeGrayDiff);
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
	float a, b, c;
	while (!feof(fp)) {
		int r = fscanf(fp, "%f, %f, %f", &a, &b, &c);
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
		int r = fscanf(fp, "%f, %f, %f", &a, &b, &c);
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

	Gnuplot gnuPlot("lines");
	wxFileName fileName = filename;	
	_gnuplotLED(gnuPlot, fileName.GetName(), nBeginLight, nTwoLight);
	_gnuplotLine(gnuPlot, "Left Ear", earLM);
	_gnuplotLine(gnuPlot, "Right Ear", earRM);
}


void VolumeSlice(int pos, void *param)
{
	MainFrame *pMainFrame = (MainFrame*)param;
	Mat &mSrc = pMainFrame->getCurrentMat(pos);	
	Mat mShow ;
	cvtColor(mSrc, mShow, CV_GRAY2BGR);
	
	deque<Point> & eyePts = pMainFrame->getEyePts();
	if(eyePts.size()>0) {
		for(int i=0; i<eyePts.size(); i++) {
			circle(mShow, Point(eyePts[i].x, eyePts[i].y), 3, Scalar(0, 0, 255), -1);
		}
	}
	
	deque<Point> & earPts = pMainFrame->getEarPts();
	if(earPts.size()>0) {
		for(int i=0; i<earPts.size(); i++) {
			circle(mShow, Point(earPts[i].x, earPts[i].y), 3, Scalar(0, 255, 0), -1);
		}
	}		
	cv::imshow("OriginalData", mShow);
}
void onCVMouse(int event, int x, int y, int flags, void* param)
{
	MainFrame *pMainFrame = (MainFrame*)param;

	if (event == CV_EVENT_RBUTTONDOWN) {
		int pos = cv::getTrackbarPos("slice", "OriginalData");
		Mat &mSrc = pMainFrame->getCurrentMat(pos);
		pMainFrame->updateOutData(mSrc);
		
		MainFrame::myMsgOutput("extract slice [0-%d]: %d\n", pMainFrame->getNumSlices()-1, pos);
	}
}
void MainFrame::OnViewSeries(wxCommandEvent& event)
{
	if(m_nSlices <=0) {
		wxLogMessage("no data\n");
		return;
	}

	cv::namedWindow("OriginalData");
	Mat &mSrc = getCurrentMat(0);
	cv::imshow("OriginalData", mSrc);
	int pos = 0;

	cv::createTrackbar("slice", "OriginalData", &pos, m_nSlices-1, VolumeSlice, this);
	cv::setMouseCallback("OriginalData", onCVMouse, this);
	
}
void MainFrame::OnUpdateViewSeries(wxUpdateUIEvent& event)
{
	event.Enable(m_nSlices >0);
}

void ResultSlice(int pos, void *param)
{
	MainFrame *pMainFrame = (MainFrame*)param;
	Mat &mSrc = pMainFrame->getResultMat(pos);	
		
	cv::imshow("ResultSeries", mSrc);
}

void MainFrame::OnViewResultSeries(wxCommandEvent& event)
{
	if(m_nSlices <=0) {
		wxLogMessage("no data\n");
		return;
	}

	cv::namedWindow("ResultSeries");
	Mat &mSrc = getResultMat(0);
	cv::imshow("ResultSeries", mSrc);
	int pos = 0;
	cv::createTrackbar("slice", "ResultSeries", &pos, m_nSlices-1, ResultSlice, this);
	//cv::setMouseCallback("ResultSeries", onCVMouse, this);
		
}


void FolderImageSlice(int pos, void *param)
{
	Mat &mSrc = gvecMat[pos];	
	cv::imshow("ImageSeries", mSrc);
}
void MainFrame::OnViewFolderImage(wxCommandEvent& event)
{
	wxString  strHistoryFile = wxEmptyString;
	if(m_FileHistory->GetCount() >0) 
		strHistoryFile= m_FileHistory->GetHistoryFile(0);
	
	static wxString strInitDir (strHistoryFile);
	wxString dirName = wxDirSelector("Choose a folder", strInitDir); //"E:\\Image_Data\\Mouse\\");
	if(dirName.empty())  return;

	myMsgOutput( "Load " + dirName + "\n");

	wxString fileSpec = _T("*.*");
	wxArrayString  	files;
	wxDir::GetAllFiles(dirName, &files,  fileSpec, wxDIR_FILES  );
	
	gvecMat.clear();
	for(unsigned int i=0; i<files.size(); i++ ) {
		wxFileName fileName = files[i];
		wxString  fName = fileName.GetName();
		wxString  fExtName = fileName.GetExt();
		if(!fExtName.IsSameAs("bmp", false) && !fExtName.IsSameAs("png", false))  continue;
		char secondChar = fName[1];
		if(secondChar !='_') continue;	
		
		//MainFrame::myMsgOutput(files[i]+ "\n");
		std::string strStd = files[i].ToStdString();
		cv::Mat	cvMat = cv::imread(strStd, CV_LOAD_IMAGE_UNCHANGED);
		//gpOutput->ShowMessage("%s", castr);
		if(cvMat.data == NULL) {
			wxString str;
			str.Printf("read file %s ERROR\n", strStd);
			myMsgOutput(str);
			break;
		}
		try	{
			gvecMat.push_back(cvMat);		
		}catch(std::exception &e){
			wxString str;
			str.Printf("Bad allocation: %s", e.what() );
			wxLogMessage( str);
		}
	}
	int nSlices = gvecMat.size();

	if(nSlices <=0) {
		wxLogMessage("no data\n");
		return;
	}

	cv::namedWindow("ImageSeries");
	Mat &mSrc = gvecMat[0];
	int pos = 0;
	cv::imshow("ImageSeries", mSrc);	
	cv::createTrackbar("slice", "ImageSeries", &pos, nSlices-1, FolderImageSlice, this);
	
}

void MainFrame::OnView2DData(wxCommandEvent& event)
{
	wxFileDialog openFileDialog(this, _("Open csv file"), "", "",
					"csv files (*.csv)|*.csv", wxFD_OPEN|wxFD_FILE_MUST_EXIST);
	if (openFileDialog.ShowModal() == wxID_CANCEL)
		return; // the user changed idea...
	// proceed loading the file chosen by the user;
	// this can be done with e.g. wxWidgets input streams:
	wxString fname = openFileDialog.GetPath();	
	
	std::ostringstream cmdstr;
	cmdstr << "plot '" << fname.ToAscii() << "' " << "with dots";
	Gnuplot gPlot2D("dots");
	gPlot2D.set_grid();
    gPlot2D.cmd(cmdstr.str());
	wxMessageBox("Hit Enter to continue", "continue...");
}
void MainFrame::OnView3DData(wxCommandEvent& event)
{
	wxFileDialog openFileDialog(this, _("Open csv file"), "", "",
					"csv files (*.csv)|*.csv", wxFD_OPEN|wxFD_FILE_MUST_EXIST);
	if (openFileDialog.ShowModal() == wxID_CANCEL)
		return; // the user changed idea...
	// proceed loading the file chosen by the user;
	// this can be done with e.g. wxWidgets input streams:
	wxString fname = openFileDialog.GetPath();	
	
	std::ostringstream cmdstr;
	cmdstr << "splot '" << fname.ToAscii() << "' " << "matrix using 1:2:3 with lines";
	
	Gnuplot gPlot3D("lines");
    gPlot3D.cmd(cmdstr.str());	
	wxMessageBox("Hit Enter to continue", "continue...");
}
