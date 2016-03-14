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
#include <wx/msgdlg.h>
#include <wx/sound.h>
#include <wx/filefn.h> 

#include <opencv2/opencv.hpp>
#include "KDE.h"
#include "MyUtil.h"
#include "gnuplot_i.h"
#include "DlgSelectFolder.h"
#include "DlgOpticalInput.h"

using namespace std;
//using namespace cv;

//Gnuplot gPlotCSV("lines");
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
	
	////////////////////////////
	m_configData.m_frameStep = pConfig->ReadLong("/optical/frameStep", 0);
	m_configData.m_threshold = pConfig->ReadDouble("/optical/threshold", 0.001);
	m_configData.m_bLED = pConfig->ReadBool("/optical/bLED", true);    
    m_configData.m_bBigHead = pConfig->ReadBool("/optical/bBigHead", false);
	m_configData.m_bVerLine = pConfig->ReadBool("/optical/bVerLine", false);
	
	m_configData.m_bEyeMove = pConfig->ReadBool("/optical/bEyeMove", true);
	m_configData.m_bGrayDiff = pConfig->ReadBool("/optical/bGrayDiff", false);
    m_configData.m_bEar = pConfig->ReadBool("/optical/bEar", true);
	m_configData.m_bBelly = pConfig->ReadBool("/optical/bBelly", true);
	m_configData.m_bOpticalPDF = pConfig->ReadBool("/optical/bOpticalPDF", true);
	m_configData.m_bOpFlowV1 = pConfig->ReadBool("/optical/bOpFlowV1", true);
	m_configData.m_bAccumulate = pConfig->ReadBool("/optical/bAccumulate", true);
	m_configData.m_bSaveFile = pConfig->ReadBool("/optical/bSaveFile", false);
	
	m_configData.m_verLine = pConfig->ReadDouble("/optical/verLine", 190);
	m_configData.m_ymin = pConfig->ReadDouble("/optical/ymin", -1);
	m_configData.m_ymax = pConfig->ReadDouble("/optical/ymax", 3);
//	m_configData.m_szROIEar = pConfig->ReadLong("/optical/ROIEar", 80);
//  m_configData.m_szROIAPB = pConfig->ReadLong("/optical/ROIAPB", 80);
    m_configData.m_referFrame = pConfig->ReadLong("/optical/referFrame", 0);
//	m_configData.m_gainHead = pConfig->ReadDouble("/optical/gainHead", 1);
//	m_configData.m_gainBelly = pConfig->ReadDouble("/optical/gainBelly", 1);
	

	this->Connect(wxID_FILE1, wxID_FILE9, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnMRUFile), NULL, this);
	
	SetSize(550, 650);
	Center();
/*	
	m_fpLog = fopen("_log.txt", "w+");
	
	m_logger = new wxLogStderr(m_fpLog);
	m_old_logger = wxLog::GetActiveTarget();

	wxLog::SetActiveTarget(m_logger);
//	wxLog::AddTraceMask(wxTRACE_Messages);
	wxLogDebug("Logging opened.");
	wxLogTrace("%s", "log trace\n");
	 * */
	DeleteContents();

}

MainFrame::~MainFrame()
{
	DeleteContents();
/*	
	wxLogDebug("Attempting to stop logger.");
	wxLog::SetActiveTarget(m_old_logger);
	delete m_logger;
	fclose(m_fpLog);
	*/
	wxConfigBase *pConfig = wxConfigBase::Get();
	m_FileHistory->Save(*pConfig);
	delete m_FileHistory;	
	delete wxConfigBase::Set((wxConfigBase *) NULL);
}

void MainFrame::DeleteContents()
{
	// TODO: Add your specialized code here and/or call the base class
	wxConfigBase *pConfig = wxConfigBase::Get();
	pConfig->Write("/optical/frameStep", m_configData.m_frameStep);
	pConfig->Write("/optical/threshold", m_configData.m_threshold);
	pConfig->Write("/optical/bLED", m_configData.m_bLED);
	pConfig->Write("/optical/bBigHead", m_configData.m_bBigHead);
	pConfig->Write("/optical/bVerLine", m_configData.m_bVerLine);

	pConfig->Write("/optical/bEyeMove", m_configData.m_bEyeMove);
	pConfig->Write("/optical/bEar", m_configData.m_bEar);
	pConfig->Write("/optical/bBelly", m_configData.m_bBelly);
	pConfig->Write("/optical/bGrayDiff", m_configData.m_bGrayDiff); 
	pConfig->Write("/optical/bOpticalPDF", m_configData.m_bOpticalPDF);
	pConfig->Write("/optical/bOpFlowV1", m_configData.m_bOpFlowV1);
	pConfig->Write("/optical/bAccumulate", m_configData.m_bAccumulate);
	pConfig->Write("/optical/bSaveFile", m_configData.m_bSaveFile);

	pConfig->Write("/optical/verLine", m_configData.m_verLine);
	pConfig->Write("/optical/ymin", m_configData.m_ymin);
	pConfig->Write("/optical/ymax", m_configData.m_ymax);
//	pConfig->Write("/optical/ROIEar", m_configData.m_szROIEar);
//	pConfig->Write("/optical/ROIAPB", m_configData.m_szROIAPB);
	pConfig->Write("/optical/referFrame", m_configData.m_referFrame);

//	pConfig->Write("/optical/gainHead", m_configData.m_gainHead);
//	pConfig->Write("/optical/gainBelly", m_configData.m_gainBelly);	
	
	m_nSlices = 0;
	m_nCageLine = -1;
	m_nUserLED2 = -1;
	m_bCutTop = false;
	m_bHasCrop = false;
	
	m_bMarkEye = false;
	m_bMarkEar = false;
	m_bMarkBelly = false;
	m_bMarkCageline = false;
    m_bViewMarks = true;
	m_bViewBellyROI = false;
	
	m_bAlreadyCrop = false;
	
	m_dqEyePts.clear();
	m_dqEarPts.clear();
	m_dqBellyPts.clear();
	m_dqBellyPts1.clear();	
	
	m_ptMostBellyOld = m_ptMostBelly = m_ptEyeL = m_ptEyeR = m_ptEarL = m_ptEarR = Point(0,0);
	
	m_bmpToggleBtnMarkEyes->SetValue(false);
	m_bmpToggleBtnMarkEars->SetValue(false);	
	m_bmpToggleBtnMarkBelly->SetValue(false);
	
	m_menuItemViewMarks->Check(m_bViewMarks);
	m_menuItemViewBellyROI->Check(m_bViewBellyROI);
}
void MainFrame::OnMRUFile(wxCommandEvent& event)
{
	wxString f(m_FileHistory->GetHistoryFile(event.GetId() - wxID_FILE1));
	if (f.empty())  return;
	
	openFile(f);
	m_FileHistory->AddFileToHistory(f);	
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

void MainFrame::readMarks(wxString &dirName)
{
	//myMsgOutput("dirname " +dirName+"\n");
	wxString last3 = dirName.Right(3);
	wxString markName;
	wxFileName fileName = dirName;
    wxUniChar sep = fileName.GetPathSeparator();
	
	if(last3.CmpNoCase("csv")==0) {
		wxString  s1 = dirName.BeforeLast('_');
		wxString  s2 = s1.Right(2);
		markName = s2 + "_Marks.txt";	
	}else {
		wxString folderName =  dirName.AfterLast(sep);
		markName = "_" + folderName + "_Marks.txt";	
	}
	

    wxString 	strParentPath =  dirName.BeforeLast(sep);
    wxString  newMarkerName = markName; //"_"+fileName.GetName()+"_Marks.txt";
    wxFileName newFullMarkerName(strParentPath, newMarkerName);
    wxFileName oldMarkerName(strParentPath, "_Marks.txt");
	
    //myMsgOutput("dirname " + newFullMarkerName.GetFullPath() +"\n");
    //myMsgOutput("oldname " + oldMarkerName.GetFullPath() +"\n");
 
	
    bool bNewMarker = true;
    if(newFullMarkerName.IsFileReadable() ==false) {
        bNewMarker = false;
        if(oldMarkerName.IsFileReadable() ==false) 
            return;
    }

	Point ptEyeL, ptEyeR, ptEarL, ptEarR;
	Point ptBellyRed, ptBellyCyan;
	int n, line, led2;
	char  type;
	FILE* fp;
    if(bNewMarker) {
        fp = fopen(newFullMarkerName.GetFullPath(), "r");
        if(fp==NULL) {
            myMsgOutput("cannot open marker file: " + newFullMarkerName.GetName() + "\n");
            return;
        }
    }else {
        fp = fopen(oldMarkerName.GetFullPath(), "r");
        if(fp==NULL) {
            myMsgOutput("cannot open marker file: " + oldMarkerName.GetName() + "\n");
            return;
        }        
    }


	do{
		fscanf(fp, "%c", &type);
		switch(type) {
			case 'E': 
				n = fscanf(fp, "%d %d %d %d\n", &ptEarL.x, &ptEarL.y, &ptEarR.x, &ptEarR.y);
				if(n==4) {
					m_dqEarPts.push_back(ptEarL);
					m_dqEarPts.push_back(ptEarR);
					m_ptEarL = m_dqEarPts[0];
					m_ptEarR = m_dqEarPts[1];
				}	
				//myMsgOutput("E %d %d %d\n", ptEarL.x, ptEarL.y, ptEarR.x, ptEarR.y);
				break;	
			case 'Y':
				n = fscanf(fp, "%d %d %d %d\n", &ptEyeL.x, &ptEyeL.y, &ptEyeR.x, &ptEyeR.y );
				if(n==4) {
					m_dqEyePts.push_back(ptEyeL);
					m_dqEyePts.push_back(ptEyeR);
					m_ptEyeL = m_dqEyePts[0];
					m_ptEyeR = m_dqEyePts[1];
				}
				//myMsgOutput("Y %d %d %d %d\n", ptEyeL.x, ptEyeL.y, ptEyeR.x, ptEyeR.y );
				break;
			case 'A':
				n = fscanf(fp, "%d %d %d %d\n", &ptBellyRed.x, &ptBellyRed.y, &ptBellyCyan.x, &ptBellyCyan.y );
				if(n==4) {
					m_dqBellyPts.push_back(ptBellyRed);
					m_dqBellyPts.push_back(ptBellyCyan);
//					m_ptBellyRed = m_dqBellyPts[0];
//					m_ptBellyCyan = m_dqBellyPts[1];
				}		
				//myMsgOutput("A Red[%d %d], Cyan[%d %d]\n", ptBellyRed.x, ptBellyRed.y, ptBellyCyan.x, ptBellyCyan.y );
				break;	
			case 'B':
				n = fscanf(fp, "%d %d\n", &m_ptMostBelly.x, &m_ptMostBelly.y);
				//myMsgOutput("m_ptMostBelly (%d, %d)\n", m_ptMostBelly.x, m_ptMostBelly.y);				
				break;					
			case 'C':
				n = fscanf(fp, "%d\n", &line);
				if(n==1)  m_nCageLine = line;
				else m_nCageLine = -1;
				//myMsgOutput("C %d\n", line);
				break;
			case 'L':
				n = fscanf(fp, "%d\n", &led2);
				if(n==1)  m_nUserLED2 = led2;
				else m_nUserLED2 = -1;
				//myMsgOutput("L %d\n", led2);
				break;	
			case 'G': // head, belly gain, m_xSD
				float gainHead, gainBelly, xSD;
				n = fscanf(fp, "%f %f %f\n", &gainHead, &gainBelly, &xSD);
				if(n==3) {
					m_configData.m_gainHead = gainHead;
					m_configData.m_gainBelly = gainBelly;
					m_configData.m_xSD = xSD;
					//myMsgOutput("head, belly gain %.2f %.2f, xSD %.2f\n", gainHead, gainBelly, xSD);
				}else
					myMsgOutput("head, belly gain load error\n");
				break;		
			case 'S': // ear, belly size
				int szEar, szBelly;
				n = fscanf(fp, "%d %d\n", &szEar, &szBelly);
				if(n==2) {
					m_configData.m_szROIEar = szEar;
					m_configData.m_szROIBelly = szBelly;
					//myMsgOutput("ear, belly size %d %d\n", szEar, szBelly);
				}else
					myMsgOutput("ear, belly size load error\n");
				break;	
			case 'R': // m_refSignal
				int refSignal;
				n = fscanf(fp, "%d\n", &refSignal);
				if(n==1) {
					m_configData.m_refSignal = refSignal;
				}else
					myMsgOutput("m_refSignal load error\n");
				break;	
			case 'm': // m_refSignal
				float  smoothWidth;
				n = fscanf(fp, "%f\n", &smoothWidth);
				if(n==1) {
					m_configData.m_smoothWidth = smoothWidth;
				}else
					myMsgOutput("m_smoothWidth load error\n");
				break;					
			case 'y': // m_ymin, m_ymax
				float ymin, ymax, ysnd;
				n = fscanf(fp, "%f %f %f\n", &ymin, &ymax, &ysnd);
				if(n==2) {
					m_configData.m_ymin = ymin;
					m_configData.m_ymax = ymax;
					//myMsgOutput("head, belly gain %.2f %.2f\n", gainHead, gainBelly);
				}else if(n==3) {
					m_configData.m_ymin = ymin;
					m_configData.m_ymax = ymax;		
					m_configData.m_ysnd = ysnd;	
				}else
					myMsgOutput("m_ymin, m_ymax load error\n");
				break;	
			case 'z': // interval m_ymin, m_ymax
				float intvymin, intvymax, intvysnd;
				n = fscanf(fp, "%f %f %f\n", &intvymin, &intvymax, &intvysnd);
				if(n==3) {
					m_configData.m_intvymin = intvymin;
					m_configData.m_intvymax = intvymax;		
					m_configData.m_intvysnd = intvysnd;	
				}else
					myMsgOutput("interval m_ymin, m_ymax load error\n");
				break;					
		}
		
	}while(!feof(fp));

	fclose(fp);
	
}
void MainFrame::openFile(wxString &dirName)
{
	wxBeginBusyCursor();
	DeleteContents();

	m_strSourcePath = dirName;
	myMsgOutput("++++++++++++++++++++++++++++++++++++++++++++++++\n");
	myMsgOutput( "Load ... " + dirName + "\n");


	bool  bRet = false;
	if(m_Rat.readData(dirName) <=0) {
		wxEndBusyCursor(); 
		return;
	}
		
	
	m_configData.Init();
	readMarks(dirName);
	
	m_nSlices = m_Rat.getNumFrames();
	m_szOriSize = m_Rat.getImgSize();
	updateOutData(m_Rat.getSrcImg(0));

	wxFileName fileName = dirName;
	wxString title =fileName.GetName();
	int lenTitle = strlen(title);
	if(lenTitle <=3) {
		wxUniChar sep = fileName.GetPathSeparator();
		int len = dirName.Len();
		int p1 = dirName.find_last_of(sep, len -lenTitle-1);
		int p2 = dirName.find_last_of(sep, p1-1);
		int p3 = dirName.find_last_of(sep, p2-1);
		title = dirName.Right(len-p3);
		title.Replace("\\", "/");
//		MainFrame:: myMsgOutput("strUpperPath "+ m_strSrcPath.Right(len-p3)+ "\n" );
	}

	//wxString title = dirName.AfterLast('\\');
	SetTitle(wxString("Dataset: ") << title);
	
	wxEndBusyCursor(); 
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

/*
void MainFrame::OnMouseLDown(wxMouseEvent& event)
{
//	wxPoint pt = m_staticBitmap->GetLogicalPosition

}
 * */
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
	m_bMarkBelly = false;
	m_bmpToggleBtnMarkEars->SetValue(false);
	m_bmpToggleBtnMarkEyes->SetValue(false);
	m_bmpToggleBtnMarkBelly->SetValue(false);
	
	m_dqEyePts.clear();
	m_dqEarPts.clear();
	m_dqBellyPts.clear();
	m_dqBellyPts1.clear();
	Refresh();
}
void MainFrame::OnMarkEyes(wxCommandEvent& event)
{
	if(event.IsChecked()) {
		if(m_bMarkEar)  {
			m_bmpToggleBtnMarkEars->SetValue(false);
			m_bMarkEar = false;
		}
		if(m_bMarkBelly)  {
			m_bmpToggleBtnMarkBelly->SetValue(false);
			m_bMarkBelly = false;
		}	
		if(m_bMarkCageline)  {
			m_bmpToggleBtnMarkCageLine->SetValue(false);
			m_bMarkCageline = false;
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
			m_bmpToggleBtnMarkEyes->SetValue(false);
			m_bMarkEye = false;
		}
		if(m_bMarkBelly)  {
			m_bmpToggleBtnMarkBelly->SetValue(false);
			m_bMarkBelly = false;
		}	
		if(m_bMarkCageline)  {
			m_bmpToggleBtnMarkCageLine->SetValue(false);
			m_bMarkCageline = false;
		}	
		m_bMarkEar = true;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_CROSS ));
	}else {
		m_bMarkEar = false;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_ARROW ));
	}		
}
void MainFrame::OnMarkBelly(wxCommandEvent& event)
{
	if(event.IsChecked()) {
		if(m_bMarkEye) { 
			m_bmpToggleBtnMarkEyes->SetValue(false);
			m_bMarkEye = false;
		}
		if(m_bMarkEar)  {
			m_bmpToggleBtnMarkEars->SetValue(false);
			m_bMarkEar = false;
		}	
		if(m_bMarkCageline)  {
			m_bmpToggleBtnMarkCageLine->SetValue(false);
			m_bMarkCageline = false;
		}
		m_bMarkBelly = true;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_CROSS ));
		m_ptMostBellyOld = m_ptMostBelly;
	}else {
		m_bMarkBelly = false;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_ARROW ));
	}	
}
void MainFrame::OnMarkCageline(wxCommandEvent& event)
{
	if(event.IsChecked()) {
		if(m_bMarkEye) { 
			m_bmpToggleBtnMarkEyes->SetValue(false);
			m_bMarkEye = false;
		}
		if(m_bMarkEar)  {
			m_bmpToggleBtnMarkEars->SetValue(false);
			m_bMarkEar = false;
		}	
		if(m_bMarkBelly)  {
			m_bmpToggleBtnMarkBelly->SetValue(false);
			m_bMarkBelly = false;
		}	
		m_bMarkCageline = true;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_CROSS ));
	}else {
		m_bMarkCageline = false;
		m_scrollWin->SetCursor(wxCursor(wxCURSOR_ARROW ));
	}		
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
	}

	// for eyes
	v = ptEyeL - earCenter;
	angle = asin(u.cross(v) / (norm(u)*norm(v)));
	if (angle < 0) {
		Point t = ptEyeR;
		ptEyeR = ptEyeL;
		ptEyeL = t;
	}
//	gpOutput->ShowMessage("angle %.2f\n", angle);
}

bool MainFrame::preprocessing()
{
	if(m_nCageLine<=0) {
		wxMessageBox("set cage horizontal line", "error");
		return false;
	}	
 

	if(m_bHasCrop ==false) {
		if(m_nUserLED2 <0) {
			if(m_Rat.detectLED(m_nCageLine)==false) {
				wxString str;
				str.Printf("detectLED error (1-based), LED1 %d, LED2 %d, LED end %d", m_Rat.m_nLED1+1, m_Rat.m_nLED2, m_Rat.m_nLED_End+1);
				wxLogMessage(str);
				m_nUserLED2= m_Rat.m_nLED1+1;
				//return false;
			}else m_nUserLED2= m_Rat.m_nLED2+1;
		}
	}
	if(!m_bHasCrop ) {
		m_Rat.cropImage(m_bCutTop, m_nCageLine);
		updateOutData(m_Rat.getSrcImg(0));	
		m_bHasCrop = true;
	}
	
//	myMsgOutput("After preprocessing, %d frames are used, cage size w%d, h%d\n",
//		m_nSlices, m_Rat.m_szImg.width, m_Rat.m_szImg.height );	
	
	return true;
}

bool MainFrame::inputDialog()
{
	int frameStep = m_configData.m_frameStep;	
	double threshold = m_configData.m_threshold;
	bool bLEDLine = m_configData.m_bLED;
	bool bBigHead = m_configData.m_bBigHead;
	bool bVerLine = m_configData.m_bVerLine;

	bool bEyeMove = m_configData.m_bEyeMove;
	bool bGrayDiff = m_configData.m_bGrayDiff;
	bool bBelly = m_configData.m_bBelly;
	bool bEar = m_configData.m_bEar;
	bool bOpticalPDF = m_configData.m_bOpticalPDF;
	bool bOpFlowV1 = m_configData.m_bOpFlowV1;
	bool bSaveFile = m_configData.m_bSaveFile;
	bool bSaveSignalPlot = m_configData.m_bSaveSignalPlot;
	bool bShowPeaks = m_configData.m_bShowPeaks;
	bool bShowAxisLabel = m_configData.m_bShowAxisLabel;
	
	double verLine = m_configData.m_verLine;
	double ymin = m_configData.m_ymin;
	double ymax = m_configData.m_ymax;
	double ysnd = m_configData.m_ysnd;

	double intvymin = m_configData.m_intvymin;
	double intvymax = m_configData.m_intvymax;
	double intvysnd = m_configData.m_intvysnd;
	
	double  smoothWidth = m_configData.m_smoothWidth;
	long ROIEar = m_configData.m_szROIEar;
	long ROIBelly = m_configData.m_szROIBelly;
	long referFrame = m_configData.m_referFrame;
	double gainHead = m_configData.m_gainHead;
	double gainBelly = m_configData.m_gainBelly;
	double xSD = m_configData.m_xSD;
	int	refSignal = m_configData.m_refSignal;
	
	int nLED2 = -1;
	bool bUserLED2;
	if(m_nUserLED2 >0)  {
		bUserLED2 = true;
		nLED2 = m_nUserLED2; // 0-based -> 1-based
	}
	else bUserLED2 = false;
	
	DlgOpticalInput dlg(frameStep, threshold, this);
	
	dlg.setVerticalLine(bLEDLine, bBigHead, bUserLED2, nLED2, bVerLine, verLine);
	dlg.setSeriesLine(bEyeMove, bEar, bGrayDiff, bBelly);
	dlg.setOptions(bOpticalPDF, bOpFlowV1, bSaveFile, bSaveSignalPlot, bShowPeaks, bShowAxisLabel, refSignal);
	dlg.setAmpYRange(ymin, ymax, ysnd, ROIEar, ROIBelly, referFrame);
	dlg.setIntervalYRange(intvymin, intvymax, intvysnd);
	dlg.setGain(gainHead, gainBelly, xSD);
	dlg.setSmooth(smoothWidth);

	if(dlg.ShowModal() !=  wxID_OK) return false;
	frameStep = dlg.getFrameSteps();
	threshold = dlg.getThreshold();
	dlg.getVerticalLine(bLEDLine, bBigHead, bUserLED2, nLED2, bVerLine, verLine);
	dlg.getSeriesLine(bEyeMove, bEar, bGrayDiff, bBelly);
	dlg.getOptions(bOpticalPDF, bOpFlowV1, bSaveFile, bSaveSignalPlot, bShowPeaks, bShowAxisLabel, refSignal);
	dlg.getAmpYRange(ymin, ymax, ysnd, ROIEar, ROIBelly, referFrame);
	dlg.getIntervalYRange(intvymin, intvymax, intvysnd);
	dlg.getGain(gainHead, gainBelly, xSD);
	dlg.getSmooth(smoothWidth);
//	dlg.Destroy();
	
	if(bUserLED2 && nLED2 > 0) {
		m_nUserLED2 = nLED2;
		MainFrame::myMsgOutput("User-specified LED2 %d (0-based)\n", m_nUserLED2-1);
	}

	m_configData.m_frameStep = frameStep;	
	m_configData.m_threshold = threshold;
	m_configData.m_bLED = bLEDLine;
	m_configData.m_bBigHead = bBigHead;
	m_configData.m_bVerLine = bVerLine;

	m_configData.m_bEyeMove = bEyeMove;
	m_configData.m_bGrayDiff = bGrayDiff;
	m_configData.m_bEar = bEar;
	m_configData.m_bBelly = bBelly;
	m_configData.m_bOpticalPDF = bOpticalPDF;
	m_configData.m_bOpFlowV1 = bOpFlowV1;
	m_configData.m_bSaveFile = bSaveFile;
	m_configData.m_bSaveSignalPlot = bSaveSignalPlot;
	m_configData.m_bShowPeaks = bShowPeaks;	
	m_configData.m_bShowAxisLabel = bShowAxisLabel;

	m_configData.m_verLine = verLine;
	m_configData.m_ymin = ymin;
	m_configData.m_ymax = ymax;
	m_configData.m_ysnd = ysnd;
	
	m_configData.m_intvymin = intvymin;
	m_configData.m_intvymax = intvymax;
	m_configData.m_intvysnd = intvysnd;
	
	m_configData.m_smoothWidth = smoothWidth;
	m_configData.m_szROIEar = ROIEar;
	m_configData.m_szROIBelly = ROIBelly;
	m_configData.m_referFrame = referFrame;
	m_configData.m_gainHead = gainHead;
	m_configData.m_gainBelly = gainBelly;
	m_configData.m_xSD = xSD;
	m_configData.m_refSignal = refSignal;
	
	return true;
}
void MainFrame::OnRatProcess(wxCommandEvent& event)
{
	if(m_dqEyePts.size()!=2) {
		wxMessageBox("Select eye points", "error");
		return;
	}
	if(m_dqEarPts.size()!=2) {
		wxMessageBox("Select ear points", "error");
		return;
	}
	if(m_dqBellyPts.size()<2 && m_ptMostBelly == Point(0, 0)) {
		wxMessageBox("Select abdomen points >=2", "error");
		return;
	}	
	m_ptEyeL = m_dqEyePts[0];
	m_ptEyeR = m_dqEyePts[1];
	m_ptEarL = m_dqEarPts[0];
	m_ptEarR = m_dqEarPts[1];
//	m_ptBellyRed = m_dqBellyPts[0];
//	m_ptBellyCyan = m_dqBellyPts[1];
	
	recognizeLeftRight(m_ptEyeL, m_ptEyeR, m_ptEarL, m_ptEarR);	
	
	if(preprocessing()==false)  { // crop image
		//wxLogMessage("preprocessing error");
		return;
	}
	if(m_bCutTop) {
 		m_ptEyeL.y -= m_nCageLine;
		m_ptEyeR.y -= m_nCageLine;
		m_ptEarL.y -= m_nCageLine;
		m_ptEarR.y -= m_nCageLine;	 
		if(m_ptMostBelly != Point(0, 0) && !m_bAlreadyCrop) {
			m_ptMostBelly.y -= m_nCageLine;
			m_bAlreadyCrop = true;
		}
//		m_ptBellyCyan.y -= m_nCageLine;	      
    }
	
	int sz = m_dqBellyPts.size();
	m_dqBellyPts1.resize(sz);
	for(int i=0; i<sz; i++) {
		m_dqBellyPts1[i] = m_dqBellyPts[i];
		if(m_bCutTop)  m_dqBellyPts1[i].y -= m_nCageLine;
	}		
	
	//bool bRet = m_Rat.processEar(m_ptEyeL, m_ptEyeR, m_ptEarL, m_ptEarR);
	if(inputDialog()==false)  return;
	int nLed2 = m_nUserLED2-1;
	if(m_ptMostBelly == Point(0, 0)) {
		m_ptMostBelly = m_Rat.findMostSignificantPt(m_dqBellyPts1, nLed2, m_configData.m_szROIBelly);
		myMsgOutput("m_ptMostBelly: (%d, %d)\n", m_ptMostBelly.x, m_ptMostBelly.y);
	}
		
	bool bRet = m_Rat.process(m_ptEyeL, m_ptEyeR, m_ptEarL, m_ptEarR, m_ptMostBelly, nLed2);
	if(bRet ==false) return;		
	updateOutData(m_Rat.getResultImg(0));

//	Refresh();
	wxBell();	
	
	writeMarks();
	myMsgOutput("-------------------------------------------------\n");	


}

void MainFrame::writeMarks()
{

    wxFileName fileName = m_strSourcePath;
    wxUniChar sep = fileName.GetPathSeparator();
    wxString 	strParentPath =  m_strSourcePath.BeforeLast(sep);
    wxString  newMarkerName = "_"+fileName.GetName()+"_Marks.txt";
    wxFileName newFullMarkerName(strParentPath, newMarkerName);

	//wxFileName dataName(m_strSourcePath, "_Marks.txt");
	FILE* fp = fopen(newFullMarkerName.GetFullPath(), "w");
	if(fp!=NULL) {
		if(m_dqEyePts.size()>=2) {
			fprintf(fp, "Y%d %d %d %d\n", m_dqEyePts[0].x, m_dqEyePts[0].y, m_dqEyePts[1].x, m_dqEyePts[1].y );
//			myMsgOutput("Left Eye: [%d, %d], Right Eye: [%d, %d]\n", m_dqEyePts[0].x, m_dqEyePts[0].y, m_dqEyePts[1].x, m_dqEyePts[1].y );
		}
		if(m_dqEarPts.size()>=2) {
			fprintf(fp, "E%d %d %d %d\n", m_dqEarPts[0].x, m_dqEarPts[0].y, m_dqEarPts[1].x, m_dqEarPts[1].y );
//			myMsgOutput("Left Ear: [%d, %d], Right Ear: [%d, %d]\n", m_dqEarPts[0].x, m_dqEarPts[0].y, m_dqEarPts[1].x, m_dqEarPts[1].y);
		}
//		if(m_dqBellyPts.size()>=2) {
//			fprintf(fp, "A%d %d %d %d\n", m_dqBellyPts[0].x, m_dqBellyPts[0].y, m_dqBellyPts[1].x, m_dqBellyPts[1].y );
//			myMsgOutput("Belly [%d, %d], [%d, %d]\n",m_dqBellyPts[0].x, m_dqBellyPts[0].y, m_dqBellyPts[1].x, m_dqBellyPts[1].y );
//		}
		if(m_ptMostBelly != Point(0, 0)) {
			int y = m_ptMostBelly.y;
			if(m_bCutTop)  y += m_nCageLine;
			fprintf(fp, "B%d %d\n", m_ptMostBelly.x, y);
//			myMsgOutput("Belly [%d, %d], [%d, %d]\n",m_dqBellyPts[0].x, m_dqBellyPts[0].y, m_dqBellyPts[1].x, m_dqBellyPts[1].y );
		}
		if(m_nCageLine >0)
			fprintf(fp, "C%d\n", m_nCageLine );
		if(m_nUserLED2 >0)
			fprintf(fp, "L%d\n", m_nUserLED2 );	

		fprintf(fp, "G%f %f %f\n", m_configData.m_gainHead, m_configData.m_gainBelly, m_configData.m_xSD);	
		fprintf(fp, "S%d %d\n", m_configData.m_szROIEar, m_configData.m_szROIBelly);
		fprintf(fp, "R%d\n", m_configData.m_refSignal);
		fprintf(fp, "m%f\n", m_configData.m_smoothWidth);
		fprintf(fp, "y%f %f %f\n", m_configData.m_ymin, m_configData.m_ymax, m_configData.m_ysnd);	
		fprintf(fp, "z%f %f %f\n", m_configData.m_intvymin, m_configData.m_intvymax, m_configData.m_intvysnd);	
		fclose(fp);
	}		
}

void MainFrame::OnRatShowResults(wxCommandEvent& event)
{
/*
	if (m_Rat.m_vecLEarGrayDiff0.empty()) {
		wxMessageBox("no data", "Error");
		return;
	}
	
	Gnuplot gnuPlot("lines");
	wxFileName fileName = m_strSourcePath;
	_gnuplotLED(gnuPlot, fileName.GetName(), m_Rat.m_idxLightBegin, m_Rat.m_idxTwoLight);
	_gnuplotLine(gnuPlot, "Left Ear", m_Rat.m_vecLEarGrayDiff0);
	//m_Rat.m_vecLEarGrayDiff, m_Rat.m_vecREarGrayDiff, m_Rat.m_vecLEyeGrayDiff);
	 */ 
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

	vector <float> earLM, earRM, eye;

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
	_gnuplotInit(gnuPlot, fileName.GetName());
	_gnuplotLED(gnuPlot,  nBeginLight, nTwoLight);
	_gnuplotLine(gnuPlot, "Left Ear", earLM);
	_gnuplotLine(gnuPlot, "Right Ear", earRM);
}


void VolumeSlice(int pos, void *param)
{
	MainFrame *pMainFrame = (MainFrame*)param;
	Mat &mSrc = pMainFrame->getCurrentMat(pos);	
	Mat mShow ;
	cvtColor(mSrc, mShow, CV_GRAY2BGR);
	
	Point 	ptEyeL, ptEyeR, ptEarL, ptEarR;
	Point 	ptBellyBo, ptBellyIn;
	
	pMainFrame->getEyePts(ptEyeL, ptEyeR);
	if(ptEyeL.x > 0) {
		circle(mShow, Point(ptEyeL.x, ptEyeL.y), 3, Scalar(0, 0, 255), -1);
		circle(mShow, Point(ptEyeR.x, ptEyeR.y), 3, Scalar(0, 0, 255), -1);
	}
	
	pMainFrame->getEarPts(ptEarL, ptEarR);
	if(ptEarL.x > 0) {
		circle(mShow, Point(ptEarL.x, ptEarL.y), 3, Scalar(0, 255, 0), -1);
		circle(mShow, Point(ptEarR.x, ptEarR.y), 3, Scalar(0, 255, 0), -1);
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
    
    cv::namedWindow("xxx");
	for(int i=0; i<m_nSlices; i++)	{
        Mat &mSrc = getResultMat(i);
        cv::imshow("xxx", mSrc);  
        waitKey(16);     
    }
	wxBell(); 
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
void MainFrame::OnMouseLButtonDown(wxMouseEvent& event)
{
	if(m_bMarkEye==false && m_bMarkEar == false && m_bMarkBelly == false && m_bMarkCageline == false)  return;
	if (getNumSlices() <= 0)  return;
	
	wxClientDC *pDC = new wxClientDC(this);
	wxPoint pt1 = event.GetLogicalPosition(*pDC);
	wxPoint pt;
	m_scrollWin->CalcUnscrolledPosition(pt1.x, pt1.y, &pt.x, &pt.y);
	
	Mat mat = getCurrentMat(0);
//	int imgH = m_szOriSize.height;
	
	if (pt.x >= mat.cols || pt.y >= mat.rows)
		return;
        
	if(m_bCutTop) {
		pt.y += m_nCageLine;
	}
    
	if(m_bMarkEye) {
		Point ptEye = Point(pt.x, pt.y);
		m_dqEyePts.push_back(ptEye);
		int sz = m_dqEyePts.size();
        if(sz>2) 	{
            m_dqEyePts.pop_front();	
            sz--;
        }
        for(int i=0; i<sz; i++) {
            if(i==0)	{
                m_ptEyeL = m_dqEyePts[0];
                if(m_bCutTop)  m_ptEyeL.y -= m_nCageLine;
            }
            if(i==1)	{
                m_ptEyeR = m_dqEyePts[1];	
                if(m_bCutTop)  m_ptEyeR.y -= m_nCageLine;		
            }
		}
	}else if(m_bMarkEar) {
		Point ptEar = Point(pt.x, pt.y);
		m_dqEarPts.push_back(ptEar);
		int sz = m_dqEarPts.size();
        if(sz>2) 	{
            m_dqEarPts.pop_front();	
            sz--;
        }
        for(int i=0; i<sz; i++) {
            if(i==0){
                m_ptEarL = m_dqEarPts[0];
                if(m_bCutTop)  m_ptEarL.y -= m_nCageLine;
            }
            if(i==1) {
                m_ptEarR = m_dqEarPts[1];	
                if(m_bCutTop)  m_ptEarR.y -= m_nCageLine;
            }
		}	
	}else if(m_bMarkBelly) {

		m_ptMostBelly = Point(0, 0);
		
		Point ptB = Point(pt.x, pt.y);
		m_dqBellyPts.push_back(ptB);
		

		if(m_bCutTop) ptB.y -= m_nCageLine;
		m_dqBellyPts1.push_back(ptB);
	}else if(m_bMarkCageline) {
		m_nCageLine = pt.y;
		myMsgOutput("cage line %d\n", m_nCageLine);
	}
	
	Refresh();    
}
void MainFrame::OnMouseRButtonDown(wxMouseEvent& event)
{
	m_dqBellyPts.clear();
	m_dqBellyPts1.clear();	
	m_ptMostBelly = Point(0, 0);
	
	Refresh(); 
}
void MainFrame::OnToolsCleanOutput(wxCommandEvent& event)
{
	wxString  strHistoryFile = wxEmptyString;
	if(m_FileHistory->GetCount() >0) 
		strHistoryFile= m_FileHistory->GetHistoryFile(0);
	
	static wxString strInitDir (strHistoryFile);
    
    wxDirDialog dlg(NULL, "Choose directory", strInitDir,
                wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST);

    if(dlg.ShowModal()!=wxID_OK)  return;
    
    wxString inputPath =dlg.GetPath();
 	if(inputPath.empty())  return;   
 
	wxString fileSpec = _T("*");
	wxArrayString  	topDirs;
	//wxDir::GetAllFiles(inputPath, &files,  fileSpec, wxDIR_DIRS );
	wxDir dir(inputPath);
    
    wxString dirName;
    bool cont = dir.GetFirst(&dirName, fileSpec, wxDIR_DIRS);
    while ( cont )
    {   
        wxString subDirName = inputPath + "\\" + dirName;
        topDirs.Add(subDirName);
        cont = dir.GetNext(&dirName);
    }
    
    // remove subdir
    bool bConfirm = false;
    int count = topDirs.GetCount();
    for(int i=0; i<count; i++) {
        MainFrame::myMsgOutput(topDirs[i]+ "\n");
        wxDir subdir(topDirs[i]);
        wxString subName;
        bool bCont = subdir.GetFirst(&subName, fileSpec, wxDIR_DIRS);
        
        if(bConfirm==false) {
            wxString str = "Remove " + subName + "??";
            wxMessageDialog dlg(this, str, "Remove Dir", wxOK|wxCANCEL);
            if(dlg.ShowModal()==wxID_CANCEL) break;
            bConfirm = true;
        }

        while ( bCont ) {
            wxString deleteDirName = topDirs[i]+ "\\" + subName;
            wxFileName::Rmdir(deleteDirName, wxPATH_RMDIR_RECURSIVE);
            MainFrame::myMsgOutput("   -->remove "+subName+ "\n");
            bCont = subdir.GetNext(&subName);
        }  
    }

    // remove files "_xxx"
    for(int i=0; i<count; i++) {
        MainFrame::myMsgOutput(topDirs[i]+ "\n");
        wxDir subdir(topDirs[i]);
        wxString subName;
        bool bCont = subdir.GetFirst(&subName, "_*", wxDIR_FILES );
        while ( bCont ) {
            if(subName.Find("_Marks.txt")==wxNOT_FOUND) {
                wxString deleteDirName = topDirs[i]+ "\\" + subName;
                wxRemoveFile(deleteDirName);
                MainFrame::myMsgOutput("   -->remove "+subName+ "\n");
            }
            bCont = subdir.GetNext(&subName);
        }  
    }
}
void MainFrame::OnRatGenRefFrame(wxCommandEvent& event)
{

	if(m_ptMostBelly == Point(0,0)) {
		wxMessageBox("Select abdomen points", "error");
		return;
	}	


	if(preprocessing()==false)  {
		//wxLogMessage("preprocessing error");
		return;
	}
	if(m_bCutTop) {
        m_ptMostBelly.y -= m_nCageLine; 
    }
	bool bRet = m_Rat.genReferenceFrameSignal(m_ptMostBelly, m_nUserLED2);
		
	wxBell();	
	
   myMsgOutput("-------------------------------------------------\n");	

}
void MainFrame::OnViewMarks(wxCommandEvent& event)
{/*
	if(event.IsChecked()) {
 //       m_bmpToggleBtnViewMark->SetValue(false);
        m_bViewMarks = false;
        //MainFrame::myMsgOutput("is checked\n");
    }else{
 //       m_bmpToggleBtnViewMark->SetValue(true);
        m_bViewMarks = true;   
        //MainFrame::myMsgOutput("no checked\n");
    }
	*/
	m_bViewMarks = !m_bViewMarks;
	m_bmpToggleBtnViewMark->SetValue(m_bViewMarks);
    Refresh();
}
void MainFrame::OnUpdateViewMarks(wxUpdateUIEvent& event)
{
    
    m_menuItemViewMarks->Check(m_bViewMarks);
}


void MainFrame::OnViewBellyROI(wxCommandEvent& event)
{
	m_bViewBellyROI = !m_bViewBellyROI;
	Refresh();
}
void MainFrame::OnUpdateViewBellyROI(wxUpdateUIEvent& event)
{
	m_menuItemViewBellyROI->Check(m_bViewBellyROI);
}


void MainFrame::readDirList(wxString& basepath, wxArrayString& dataDirs)
{
	wxString fileSpec = _T("*");
	wxArrayString  	topDirs;
	wxArrayString  	secondDirs;

	
	wxDir dir(basepath);
    
	// top level
    wxString dirName;
    bool cont = dir.GetFirst(&dirName, fileSpec, wxDIR_DIRS);
    while ( cont )
    {   
        
		wxString subDirName;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
		subDirName = basepath + "\\" + dirName;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
		subDirName = basepath + "/" + dirName;;
#endif		
        topDirs.Add(subDirName);
        cont = dir.GetNext(&dirName);
    }
	if(dir.HasFiles("*Marks.txt")) {
		dataDirs = topDirs;
	}else{	
		// second level subdir
		int count = topDirs.GetCount();
		for(int i=0; i<count; i++) {
	//        MainFrame::myMsgOutput(topDirs[i]+ "\n");
			wxDir subdir(topDirs[i]);
			wxString subName;
			bool bCont = subdir.GetFirst(&subName, fileSpec, wxDIR_DIRS);
			while ( bCont ) {
				wxString secondLevelName;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
				secondLevelName = topDirs[i]+ "\\" + subName;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
				secondLevelName = topDirs[i]+ "/" + subName;
#endif				
				secondDirs.Add(secondLevelName);
				bCont = subdir.GetNext(&subName);
			}  
		}
		dataDirs = secondDirs;
	}

	//count = secondDirs.GetCount();
    for(int i=0; i<dataDirs.GetCount(); i++) {
        MainFrame::myMsgOutput(dataDirs[i]+ "\n");
		wxFileName fileName = dataDirs[i];
		wxUniChar sep = fileName.GetPathSeparator();
		wxString  strParentPath =  dataDirs[i].BeforeLast(sep);
		wxString  markName = "_"+fileName.GetName()+"_Marks.txt";
		wxFileName fullMarkName(strParentPath, markName);
		
		bool bHasFile = wxFileName::Exists(fullMarkName.GetFullPath());
		if(bHasFile)
			MainFrame::myMsgOutput(fullMarkName.GetFullPath()+ "\n");
		else {
			MainFrame::myMsgOutput("no file\n");
			dataDirs.RemoveAt(i);
			i--;
		}
	}	
}
void MainFrame::OnBatchProcess(wxCommandEvent& event)
{
	wxString basepath;
//	static wxString strIniDir="";
	DlgSelectFolder dlg(this, m_strBatchDir);
	int ret = dlg.ShowModal();
	if(ret == wxID_OK) {
		m_strBatchDir = dlg.m_strDir;
		basepath = dlg.m_strDir;
		myMsgOutput("Batch process dir: " + m_strBatchDir +"\n") ;
	}else return;
	
	wxArrayString  	dataDirs;
	readDirList(basepath, dataDirs);
	
	//MyConfigData	oldConfigData;
	m_configData.Init();
	readMarks(dataDirs[0]);
	//oldConfigData = m_configData;
	
	if(inputDialog()==false)  return;
	for(int i=0; i<dataDirs.GetCount(); i++) {
//		wxLogMessage(secondDirs[i]);
		openFile(dataDirs[i]);	
		//m_configData = oldConfigData;
		
		if(m_dqEyePts.size()!=2) {
			wxMessageBox("Select eye points", "error");
			return;
		}
		if(m_dqEarPts.size()!=2) {
			wxMessageBox("Select ear points", "error");
			return;
		}
		if(m_dqBellyPts.size()<2 && m_ptMostBelly == Point(0, 0)) {
			wxMessageBox("Select abdomen points >=2", "error");
			return;
		}	
		m_ptEyeL = m_dqEyePts[0];
		m_ptEyeR = m_dqEyePts[1];
		m_ptEarL = m_dqEarPts[0];
		m_ptEarR = m_dqEarPts[1];
//		m_ptBellyRed = m_dqBellyPts[0];
//		m_ptBellyCyan = m_dqBellyPts[1];
		
		recognizeLeftRight(m_ptEyeL, m_ptEyeR, m_ptEarL, m_ptEarR);	
		
		if(preprocessing()==false)  {
			//wxLogMessage("preprocessing error");
			return;
		}
		if(m_bCutTop) {
			m_ptEyeL.y -= m_nCageLine;
			m_ptEyeR.y -= m_nCageLine;
			m_ptEarL.y -= m_nCageLine;
			m_ptEarR.y -= m_nCageLine;	
			
			if(m_ptMostBelly != Point(0, 0) && !m_bAlreadyCrop) {
				m_ptMostBelly.y -= m_nCageLine;
				m_bAlreadyCrop = true;
			}
//			m_ptBellyRed.y -= m_nCageLine;
//			m_ptBellyCyan.y -= m_nCageLine;	      
		}

		int sz = m_dqBellyPts.size();
		m_dqBellyPts1.resize(sz);
		for(int i=0; i<sz; i++) {
			m_dqBellyPts1[i] = m_dqBellyPts[i];
			if(m_bCutTop)  m_dqBellyPts1[i].y -= m_nCageLine;
		}		
		
		int nLed2 = m_nUserLED2-1;
		if(m_ptMostBelly == Point(0, 0))
			m_ptMostBelly = m_Rat.findMostSignificantPt(m_dqBellyPts1, nLed2, m_configData.m_szROIBelly);
		
		bool bRet = m_Rat.process(m_ptEyeL, m_ptEyeR, m_ptEarL, m_ptEarR, m_ptMostBelly, nLed2);
		if(bRet ==false) return;		
		//updateOutData(m_Rat.getResultImg(0));
		writeMarks();
		myMsgOutput("-------------------------------------------------\n");		
		wxBell();
	}
	waitKey(100); 
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
	if(wxSound::Play("..\\Gong.wav")==false)
		myMsgOutput("no sound\n");
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
	if(wxSound::Play("../Gong.wav")==false)
		myMsgOutput("no sound\n");
#endif	

	myMsgOutput("Batch process finish: "+m_strBatchDir + "\n");
}
void MainFrame::OnRatCheckAPB(wxCommandEvent& event)
{
	static double smoothWidth = 3;
	static bool  bShowSymbol = true;
	static bool  bCheckOnlyFirst = false;
	wxString inputPath;
	static wxString strIniDir="";
	DlgSelectFolder dlg(this, strIniDir);
	dlg.setParam(smoothWidth, bCheckOnlyFirst);
	dlg.setShowSymbol(bShowSymbol);
	int ret = dlg.ShowModal();
	if(ret == wxID_OK) {
		inputPath = dlg.m_strDir;
		strIniDir = dlg.m_strDir;
		dlg.getParam(smoothWidth, bCheckOnlyFirst);
		bShowSymbol = dlg.showSymbol();
		wxString str1;
		str1.Printf("OnShowCSV dir: %s, sigma %.2f\n", inputPath, smoothWidth );
		myMsgOutput(str1) ;
	}else return;	
	
	wxString fileSpec = _T("*_Belly.csv");
	wxArrayString  	files;
	wxDir::GetAllFiles(inputPath, &files,  fileSpec, wxDIR_FILES );
	int nFiles = files.size();
	if(nFiles==0)  return;
	
	wxArrayString  	dataDirs;
	readDirList(inputPath, dataDirs);

	char str[100];
	int ampUpCounter = 0;
	int ampLowCounter = 0;
	int periodInc = 0;
	int periodDec = 0;
	int bothAcc = 0;
    for(unsigned int i=0; i<nFiles; i++ ) {
		wxFileName fileName = files[i];
		wxString  fName = fileName.GetName();
		wxString s;
		s.Printf("Read [%d/%d], %s, sigma %.3f\n", i+1, nFiles, fName, smoothWidth);
		myMsgOutput(s);
		
		vector<float> vSignal;
		int szSignal = _readCSVFile(files[i].ToStdString(), vSignal);
		if(szSignal< 0 ) {
			myMsgOutput("read data ERROR\n");
			continue;
		}
		
		float minValue = *std::min_element(vSignal.begin(),vSignal.end());
		//readMarks(dataDirs[i]);
		m_configData.Init();
		readMarks(files[i]);
		double xSD = m_configData.m_xSD;
		double ymin = m_configData.m_ymin; // -0.8
		double ymax = m_configData.m_ymax; // 0.8 
		double ysnd = m_configData.m_ysnd;
	
		double intvymin = m_configData.m_intvymin;
		double intvymax = m_configData.m_intvymax;
		double intvysnd = m_configData.m_intvysnd;
	
//		myMsgOutput("m_nUserLED2 %d, Smooth %.2f, minValue %f\n", m_nUserLED2, smoothWidth, minValue);	
					
		vector <float>  vecBellySmooth;
		vector<Point2f> peakBelly;
		vector<Point2f> peakMinMax;
		vector<Point2f> vPeakDist;
		vector<float> 	vPeakDistX;
		vector<float> 	vPeakDistY;
		float meanPeriod, sdPeriod, meanAmp, sdAmp;
	
		CRat::smoothData(vSignal, vecBellySmooth, smoothWidth);	
		CRat::findPeaks(vSignal, vecBellySmooth, peakBelly);
		
		FILE* fp = fopen("_tmp.csv", "w");
		for(int i=0; i<vSignal.size(); i++)
			fprintf(fp, "%f, %f\n", vSignal[i], vecBellySmooth[i]);
		fclose(fp);

		int nLedPeriod = CRat::peakPeriodAnalysis(peakBelly, vPeakDistX, vPeakDistY, m_nUserLED2, meanPeriod, sdPeriod);
		int nLedPeak = CRat::peakAmplitudeAnalysis(peakBelly, m_nUserLED2, meanAmp, sdAmp);
		
		int both = -1;
		
		string sColorAmp= "#0000FF";
		string sColorPeriod= "#0000FF";
		for(int i=nLedPeak; i<peakBelly.size()-1; i++) {
			float upper = meanAmp + xSD*sdAmp;
			float lower = meanAmp - xSD*sdAmp;
			if(peakBelly[i].y >= upper) {
				sColorAmp = "#FF0022";
				ampUpCounter ++;
				both ++;
				break;
			}else if(peakBelly[i].y <= lower) {
				sColorAmp = "#FF0022";
				ampLowCounter ++;
				both ++;
				break;				
			}
			
			if(bCheckOnlyFirst)  break;
		}
		for(int i=nLedPeriod; i<vPeakDistY.size()-1; i++) {
			float upper = meanPeriod + xSD*sdPeriod;
			float lower = meanPeriod - xSD*sdPeriod;
			if(vPeakDistY[i] >= upper ) {
				sColorPeriod = "#FF0022";
				periodInc ++;
				both ++;				
				break;
			}else if(vPeakDistY[i] <= lower) {
				sColorPeriod = "#FF0022";
				periodDec ++;
				both ++;				
				break;
			}
			if(bCheckOnlyFirst)  break;
		}
		if(both>=1) bothAcc++;
		//myMsgOutput("--amp mean %f, sd %f, [%f, %f]\n", meanAmp, sdAmp,  meanAmp- xSD*sdAmp, meanAmp+ xSD*sdAmp);
		myMsgOutput("     [%d]-LED Peak %d %d --- amp (%d, %d), period (%d, %d), both %d ", 
			i+1, nLedPeak, nLedPeriod, ampUpCounter, ampLowCounter, periodInc, periodDec, bothAcc);
		if(both == -1)  myMsgOutput("\n");
		else myMsgOutput(" **\n");
		
		///////////G N U P L O T/////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////
		double upperSD = meanPeriod + xSD*sdPeriod;
		double lowerSD = meanPeriod - xSD*sdPeriod;
		if(intvymax <= upperSD +10) intvymax = upperSD + 30;		
		if(intvymin >= lowerSD -10) intvymin = lowerSD - 30;
		
		float highAmp = (ymax-ymin) / 35.;
		float highIntv = (intvymax-intvymin)/35.;
		if(ysnd <= ymin || ysnd >= ymax) ysnd = ymin + 0.2;
		if(intvysnd >= lowerSD ) intvysnd = lowerSD - 15;
		if(intvysnd < intvymin)  intvymin -=10;
		
		int xStart = floor((0-m_nUserLED2)/25.) * 25;
		int xEnd = ceil((szSignal-m_nUserLED2)/25.) * 25;
//		myMsgOutput("xstart %d, %d\n", xStart, xEnd);
		
		_gnuplotInit(gPlotR, fName.ToAscii(), ymin, ymax);
		_gnuplotInit(gPlotP, fName.ToAscii(), intvymin, intvymax);
		
		gPlotR.set_legend("left");	
		gPlotP.set_legend("left");	

		gPlotR.cmd("set termoption noenhanced");
		gPlotP.cmd("set termoption noenhanced");
		
		gPlotP.set_xrange(xStart, xEnd);
		gPlotR.set_xrange(xStart, xEnd);
		gPlotR.set_xlabel("Peri-stimulus Frame");
		gPlotR.set_ylabel("Movement index (arb. unit)");

		gPlotP.set_xlabel("Peri-stimulus Frame");
		gPlotP.set_ylabel("Inter-peak interval (arb. unit)");

		_gnuplotSoundOnset(gPlotR, m_nUserLED2, szSignal, ysnd, highAmp, 100);	// -0.65
		_gnuplotSoundOnset(gPlotP, m_nUserLED2, szSignal, intvysnd, highIntv, 100);
//		myMsgOutput(" ymin %f %f, intvymin %f %f %f, sd [%f, %f]\n", ymin, ysnd, intvymin, intvymax, intvysnd, lowerSD, upperSD);
		wxString strLine;
		strLine.Printf("%.1fxSD", xSD);
		
		vector<float> 	vxTicks(szSignal);
		for(int i=0; i<szSignal; i++)
			vxTicks[i] = i- m_nUserLED2;
		
		_gnuplotLineXY(gPlotR, "Abdomen", vxTicks, vSignal, sColorAmp.c_str()); //"#00008000");			
		_gnuplotHoriLine(gPlotR, xStart, xEnd, meanAmp + sdAmp*xSD, "#00100800", "..-", strLine);	
		_gnuplotHoriLine(gPlotR, xStart, xEnd, meanAmp - sdAmp*xSD, "#00100800",  "..-");
		
		if(bShowSymbol) {
			for(int i=0; i<peakBelly.size(); i++)
				peakBelly[i].x -= m_nUserLED2;
			_gnuplotPoint(gPlotR, peakBelly, "#00008f00" );
		}
		for(int i=0; i<vPeakDistX.size(); i++)
			vPeakDistX[i] -= m_nUserLED2;
		_gnuplotSteps(gPlotP, vPeakDistX, vPeakDistY, sColorPeriod.c_str(), "Inter-peak interval");
		//_gnuplotHoriLine(gPlotP, n, meanPeriod, "#000088FF");						
		_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriod+xSD*sdPeriod, "#00100800", "..-", strLine);			
		_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriod-xSD*sdPeriod, "#00100800", "..-");			
		//_gnuplotVerticalLine(gPlotP, m_nUserLED2, "#000000FF");
		
		if(i < nFiles-1)
			wxMessageBox("press anykey ...");
    }
	 

}
