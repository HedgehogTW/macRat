
#include <wx/dir.h>
#include <wx/utils.h> 

#include "MainFrame.h"
#include "Rat.h"
#include "KDE.h"

#include "itkOpenCVImageBridge.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"

//#include "MyContour.h"
#include "MyUtil.h"
#include <gsl/gsl_fit.h>
#include "BendPoint.h"
#include "DlgOpticalInput.h"

#define PDF_SIZE    50

using namespace cv;


Gnuplot gPlotL("lines");
Gnuplot gPlotR("lines");

CRat::CRat()
{
	
	m_offsetEar = Point(40, 40);

	m_nCageLine = -1;

	m_mean = m_stddev = 0;
	m_nProcessFrame = -1;
	m_classifier = 0;
	
	clearData();
	_redirectStandardOutputToFile("_cout.txt", true);
}


CRat::~CRat(void)
{
}
void CRat::clearData()
{
	m_vecMat.clear();
	m_vecDest.clear();	
	m_vecFlow.clear();
	
	m_vecEyeL.clear();
	m_vecEyeR.clear();
	
//	m_vecLEyeGrayDiff.clear();
//	m_vecREyeGrayDiff.clear();

	m_vFilenames.clear();
	m_nSlices = 0;

	m_nLED1 = -1;
	m_nLED2 = -1;
	m_nLED_End = -1;
	m_nCageLine = -1;
//	m_bCutTop = false;
	m_bLED_OK = false;
}

int CRat::readData(wxString inputPath)
{
	clearData();
	
	m_strSrcPath = inputPath;	
	
	wxString fileSpec = _T("*.bmp");
	wxArrayString  	files;
	wxDir::GetAllFiles(inputPath, &files,  fileSpec, wxDIR_FILES );
	
	cv::Mat	 cvMat;
	m_vecMat.clear();
	m_vFilenames.clear();
	for(unsigned int i=0; i<files.size(); i++ ) {
		wxFileName fileName = files[i];
		wxString  fName = fileName.GetName();
		char firstChar = fName[0];
		if(firstChar=='_') continue;
		//MainFrame::myMsgOutput(files[i]+ "\n");
		std::string strStd = files[i].ToStdString();
		cvMat = cv::imread(strStd, CV_LOAD_IMAGE_UNCHANGED);
		//gpOutput->ShowMessage("%s", castr);
		if(cvMat.data == NULL) {
			wxString str;
			str.Printf("read file %s ERROR\n", strStd);
			MainFrame::myMsgOutput(str);
			break;
		}
		m_vecMat.push_back(cvMat);
		m_vFilenames.push_back(strStd);
		
	}
	
	m_nSlices = m_vecMat.size();
	if(m_nSlices <=0) {
		wxLogMessage("cannot read images");
		return 0;
	}
	m_szImg = Size(m_vecMat[0].cols, m_vecMat[0].rows );
	MainFrame:: myMsgOutput("read %d bmp files, size w%d, h%d\n",m_nSlices, m_szImg.width, m_szImg.height );
	return m_nSlices;
}

void CRat::saveResult(const char *subpath, vector <Mat> &vecM)
{
	
	cv::Mat	 cvMat;
	cv::Mat	 cvMatGray;
	wxString outpath; 

#if defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__) 
	outpath = m_strSrcPath + "/" + subpath;
#else
	outpath = m_strSrcPath + "\\" + subpath;
#endif	
	wxFileName fileName = m_strSrcPath;
	wxString  fName = fileName.GetName();
	wxFileName dataName(m_strSrcPath, "_"+fName+ "_motion.csv");


	if(vecM.size() ==0) return;

	if(wxDirExists(outpath)==false) {
		if(wxMkdir(outpath)==false) {
			wxString str;
			str.Printf("Create output directory error, %s", outpath);
			wxLogMessage(str);
			return;
		}
	}
	int sz = vecM.size();
	
	for(int i=0; i<sz; i++) {
		wxFileName fileName = wxString(m_vFilenames[i]);
		fileName.AppendDir(subpath);
		wxFileName savePath = fileName.GetPath();
		wxFileName saveName(savePath.GetFullPath(), fileName.GetFullName());
		
		if(i>=m_nLED1 && i<m_nLED_End) 
			cv::circle(vecM[i], Point(15, 15), 7, cv::Scalar(255), -1);
		if(i==m_nLED2) 
			cv::circle(vecM[i], Point(vecM[i].cols-15, 15), 7, cv::Scalar(255), -1);
	
		imwrite(saveName.GetFullPath().ToStdString(), vecM[i]);
	}

	return ;
}
Mat CRat::accumulate()
{
	if (m_nSlices == 0) return Mat();

	cv::Mat  mDest;
	cv::Mat  mAccu(m_vecMat[0].size(), CV_32FC1, Scalar(0));
	int kSize = 5;

	for (int i = 0; i < m_nSlices; i+=5) {
		cv::Mat dx;
		cv::Mat dy;
		cv::Mat edge;
		
		cv::Sobel(m_vecMat[i], dx, CV_32F, 1, 0, kSize);
		cv::Sobel(m_vecMat[i], dy, CV_32F, 0, 1, kSize);
		edge = abs(dx) + abs(dy);

		mAccu += edge;
	}
	double  min, max, a;
	cv::minMaxLoc(mAccu, &min, &max);
	a = 255./(max - min);
	cv::convertScaleAbs(mAccu, mDest, a, -min*a);
	//imshow("mAccu", mDest);
	return mDest;
}

bool CRat::detectLED(int nCageLine)
{
	if(nCageLine <=0)  return false;
	if(m_bLED_OK)  return true;
	
	double minArea =35; //330;
	double maxArea = 1600;
	double minRatio = 0.45; //0.65;
	double maxCompact = 14;

	int i;
	m_nLED1 = -1;
	m_nLED2 = -1;
	m_nLED_End = -1;
	
	m_nCageLine = nCageLine;
	
	int h = m_vecMat[0].rows;

	int  newCutLine = -1;
	bool bTopDown;
	Mat mIn, mDest, mRedu;
	if(m_nCageLine < h/2) {
		mIn = m_vecMat[0].rowRange(0, m_nCageLine);
		bTopDown = true;
	}else {
		mIn = m_vecMat[0].rowRange(m_nCageLine, h);
		bTopDown = false;
	}
	cv::threshold(mIn, mDest, 253, 255, cv::THRESH_BINARY);  
	Mat kernel = Mat::ones(3, 3, CV_8U);
	morphologyEx(mDest, mDest, MORPH_OPEN, kernel);
	cv::reduce(mDest, mRedu, 1, CV_REDUCE_AVG, CV_32F);
	mRedu /=(255);
//	_OutputMat(mRedu, "_profile.csv", true);
	if(bTopDown) {
		for(int i=0; i<mRedu.rows; i++)
			if(mRedu.at<float>(i, 0) > 0.1)  {
				newCutLine = i;
				break;
			}
	}else {
		for(int i=mRedu.rows-1; i>=0; i--)
			if(mRedu.at<float>(i, 0) > 0.1)  {
				newCutLine = i;
				break;
			}		
	}

	m_vecDest.clear();

	for(i=0; i<m_nSlices; i++) {
//Mat mColor;
		Mat mIn, mDest;
		if(m_nCageLine < h/2) {
			if(newCutLine >0)
				mIn = m_vecMat[i].rowRange(0, newCutLine);
			else
				mIn = m_vecMat[i].rowRange(0, m_nCageLine);
		}else{
			if(newCutLine >0)
				mIn = m_vecMat[i].rowRange(newCutLine, h);
			else
				mIn = m_vecMat[i].rowRange(m_nCageLine, h);
		}
		cv::threshold(mIn, mDest, 253, 255, cv::THRESH_BINARY);  

		Mat kernel = Mat::ones(3, 3, CV_8U);
		morphologyEx(mDest, mDest, MORPH_OPEN, kernel);

//cvtColor( mDest, mColor, CV_GRAY2BGR );

		// find body mask
		vector<vector<cv::Point> > vecContour;
		findContours(mDest, vecContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


		for(int k=0; k<vecContour.size(); k++) {
			Scalar m = cv::mean(vecContour[k]);
			if(m[1] < 10) { // too close to m_nCageLineY, 50
				vecContour.erase (vecContour.begin()+k);
				k--;
				continue;
			}
		}

		for(int k=0; k<vecContour.size(); k++) {
			double area = cv::contourArea(vecContour[k]);	
			int numPts = vecContour[k].size();
			double compact = numPts * numPts / area;
			if(area >= maxArea || area < minArea || compact > maxCompact) {
				vecContour.erase (vecContour.begin()+k);
				k--;
				continue;
			}
		}
//cv::drawContours(mColor, vecContour, -1, cv::Scalar(0, 0, 255), CV_FILLED, 8);		
		for(int k=0; k<vecContour.size(); k++) {			
			double ratio, angle;
			aspectRatio(vecContour[k], ratio, angle);
			if(ratio <minRatio) {
				vecContour.erase (vecContour.begin()+k);
				k--;
				continue;
			}
		}

//char strAngle[40];
//sprintf(strAngle, "slice %04d", i+1);
//string text (strAngle);
//int fontFace = CV_FONT_HERSHEY_PLAIN;
//Point textOrg(0,50);
//putText(mColor, text, textOrg, fontFace, 1, Scalar(0, 255,0), 1, 8);
//cv::drawContours(mColor, vecContour, -1, cv::Scalar(0, 255,0), CV_FILLED, 8);
//imshow("color", mColor);
//waitKey(2000);


		int  numComp = vecContour.size();
		if(numComp ==1 && m_nLED1 ==-1)
			m_nLED1 = i;
	
		if(numComp ==2 && m_nLED2 ==-1)
			m_nLED2 = i;

		if(numComp ==0 && m_nLED1 != -1 && m_nLED_End==-1)
			m_nLED_End = i-1;

		unsigned found = m_vFilenames[i].find_last_of("/\\");
		string name =  m_vFilenames[i].substr(found+1) ;
		mDest = cv::Scalar(0);
		cv::drawContours(mDest, vecContour, -1, cv::Scalar(255), CV_FILLED, 8);

//m_vecDest.push_back(mColor);

	
	}
	if(m_nLED_End==-1)  m_nLED_End = m_nSlices-1;

	MainFrame:: myMsgOutput("1-LED: %d, 2-LED: %d\n", m_nLED1+1, m_nLED2+1); 

//saveResult("LED", m_vecDest);

	if (m_nLED2 < 0) 
		m_bLED_OK = false;
	else 
		m_bLED_OK = true;
		
	return m_bLED_OK;
}

Point CRat::aspectRatio(vector<cv::Point> &con, double &ratio, double &angle )
{
	cv::RotatedRect box = cv::minAreaRect(con);	
	cv::Point2f vtx[4], pt1, pt2;
	double slope;

	box.points(vtx);
	float w, h;
	pt1 = vtx[0]- vtx[1];
	h = cv::norm(pt1);
	
	pt2 = vtx[1]- vtx[2];
	w = cv::norm(pt2);
	if(w>h) {
		ratio = h/w;
		if(pt2.x !=0) slope = (double)pt2.y/pt2.x;
		else
			slope = INT_MAX;
	}else{ 
		ratio = w/h;
		if(pt1.x !=0) slope = (double)pt1.y/pt1.x;
		else
			slope = INT_MAX;
	}

	angle = atan(slope) * 180. / CV_PI; ; //box.angle;

	return box.center;
}

void CRat::cropImage(bool& bCutTop)
{
	vector <Mat> vecData;
	vector <string> vFilenames;
	int w = m_vecMat[0].cols;
	int h = m_vecMat[0].rows;
	
	if(h <=m_nCageLine) return;
	if(m_nCageLine >0) {
		for(int i=0; i<m_nSlices; i++)  {
			Mat mSrc;
			if(m_nCageLine > h/2)
				mSrc = m_vecMat[i].rowRange(0, m_nCageLine);
			else {
				mSrc = m_vecMat[i].rowRange(m_nCageLine, h);
                bCutTop = true;
            }
				
			Mat mData;
			mSrc.copyTo(mData);

			vecData.push_back(mData);
			vFilenames.push_back(m_vFilenames[i]);
		
			if(i>=m_nLED1 && i<m_nLED_End) 
				cv::circle(mData, Point(15, 15), 7, cv::Scalar(255), -1);
			if(i==m_nLED2) 
				cv::circle(mData, Point(mData.cols-15, 15), 7, cv::Scalar(255), -1);
		}
		m_vecMat.clear();
		int sz= vecData.size();
		m_vecMat.resize(sz);
		m_vFilenames.resize(sz);
			
		std::copy(vecData.begin(), vecData.end(), m_vecMat.begin());
		std::copy(vFilenames.begin(), vFilenames.end(), m_vFilenames.begin());
	}

	m_szImg = Size(m_vecMat[0].cols, m_vecMat[0].rows );
	m_nSlices = m_vecMat.size();

//	saveResult("cage", m_vecMat);
	
	return;
}
void CRat::DC_removal(int nFirstLED, vector <float>& vecSignal)
{
	int n = vecSignal.size();
	
	if(n<=nFirstLED )  {
		wxLogMessage("n <= nFirstLED");
		return;
	}
	
	float mean = 0;
	
	for(int i=0; i<nFirstLED; i++) 	mean += vecSignal[i];
	mean /= nFirstLED;
	for(int i=0; i<n; i++)  vecSignal[i] -= mean;	
}
void CRat::Notch_removal(vector <float>& vecSignal, int refFrame)
{
    vecSignal[refFrame] = (vecSignal[refFrame-1] + vecSignal[refFrame+1]) /2.0;
}

bool CRat::process(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR, Point& ptRed, Point& ptCyan)
{
	m_ptEyeL = ptEyeL;
	m_ptEyeR = ptEyeR;
	
	m_ptEarL = ptEarL;
	m_ptEarR = ptEarR;
    
	m_ptRed = ptRed;
	m_ptCyan = ptCyan;
	
	m_vecEyeL.clear();
	m_vecEyeR.clear();

	Point ptEyeC;
	ptEyeC.x= (ptEyeL.x+ptEyeR.x)/2;
	ptEyeC.y= (ptEyeL.y+ptEyeR.y)/2;
	
	
	/////////////////////////////////////////////////////////////optical flow
	MyConfigData  configData;
	MainFrame::m_pThis->getConfigData(configData);
	
	bool bNewOpFlowV1;
	int  newReferFrame;
	int  newFrameSteps;
	int frameStep = configData.m_frameStep;	
	double threshold = configData.m_threshold;
	bool bLED = configData.m_bLED;
    bool bRefLine = configData.m_bRefLine;
	bool bPinna = configData.m_bPinna;
	bool bVerLine = configData.m_bVerLine;
	
	bool bEyeMove = configData.m_bEyeMove;
	bool bGrayDiff = configData.m_bGrayDiff;
	bool bAbdo = configData.m_bAbdo;
    bool bEar = configData.m_bEar;
	bool bOpticalPDF = configData.m_bOpticalPDF;
	bool bOpFlowV1 = configData.m_bOpFlowV1;
	bool bAccumulate = configData.m_bAccumulate;
	bool bSaveFile = configData.m_bSaveFile;
	
	double verLine = configData.m_verLine;
	double ymin = configData.m_ymin;
	double ymax = configData.m_ymax;
	m_ROIsz = configData.m_szROI;
    long referFrame = configData.m_referFrame;
	double gainEye = configData.m_gainEye;
	double gainPDF = configData.m_gainPDF;
	
	DlgOpticalInput dlg(frameStep, threshold, MainFrame::m_pThis);
	dlg.setVerticalLine(bLED, bRefLine, bPinna, bVerLine, verLine);
	dlg.setSeriesLine(bEyeMove, bEar, bGrayDiff, bAbdo);
	dlg.setOptions(bOpticalPDF, bOpFlowV1, bAccumulate, bSaveFile);
	dlg.setYRange(ymin, ymax, m_ROIsz, referFrame);
	dlg.setGain(gainEye, gainPDF);
	
	if(dlg.ShowModal() !=  wxID_OK) return false;
	newFrameSteps = dlg.getFrameSteps();
	threshold = dlg.getThreshold();
	dlg.getVerticalLine(bLED, bRefLine, bPinna, bVerLine, verLine);
	dlg.getSeriesLine(bEyeMove, bEar, bGrayDiff, bAbdo);
	dlg.getOptions(bOpticalPDF, bNewOpFlowV1, bAccumulate, bSaveFile);
	dlg.getYRange(ymin, ymax, m_ROIsz, referFrame);
	dlg.getGain(gainEye, gainPDF);
	dlg.Destroy();
	
	configData.m_frameStep = newFrameSteps;	
	configData.m_threshold = threshold;
	configData.m_bLED = bLED;
    configData.m_bRefLine = bRefLine;
	configData.m_bPinna = bPinna;
	configData.m_bVerLine = bVerLine;
	
	configData.m_bEyeMove = bEyeMove;
	configData.m_bGrayDiff = bGrayDiff;
    configData.m_bEar = bEar;
	configData.m_bAbdo = bAbdo;
	configData.m_bOpticalPDF = bOpticalPDF;
	configData.m_bOpFlowV1 = bNewOpFlowV1;
	configData.m_bAccumulate = bAccumulate;
	configData.m_bSaveFile = bSaveFile;
	
	configData.m_verLine = verLine;
	configData.m_ymin = ymin;
	configData.m_ymax = ymax;
	configData.m_szROI = m_ROIsz;
    configData.m_referFrame = referFrame;
	configData.m_gainEye = gainEye;
	configData.m_gainPDF = gainPDF;
	
    m_bShowEye = bEyeMove;
    m_bShowEar = bEar;
    m_bShowAbdo = bAbdo;
   
	MainFrame::m_pThis->setConfigData(configData);
	m_offsetEar = Point(m_ROIsz/2, m_ROIsz/2);
	
    if(referFrame<=0)
        newReferFrame = findReferenceFrame(ptEarL);
    else
        newReferFrame = referFrame;
        
	MainFrame::myMsgOutput("Reference frame %d\n", newReferFrame);
	if(newReferFrame <0) {
		wxLogMessage("cannot find reference frame");
		return false;
	}
	
	if(bEyeMove) {
		findEyeCenter(ptEyeL, m_vecEyeL, m_vecEyeLMove, newReferFrame);
		findEyeCenter(ptEyeR, m_vecEyeR, m_vecEyeRMove, newReferFrame);
	}else {
		m_vecEyeR.resize(m_nSlices);
		m_vecEyeL.resize(m_nSlices);
		for(int i=0; i<m_nSlices;i++) {
			m_vecEyeR[i] = ptEyeR;
			m_vecEyeL[i] = ptEyeL;
		}
	}
      
//////////////////////////////////////////////////////////////////////	
	clock_t start, finish;
	double  duration;
	start = clock();
	
	wxBeginBusyCursor();
	
	vector <float>  vecEyeFlowPdfL;
	vector <float>  vecEyeFlowPdfR;
	vector <float>  vecEyeFlowPdfRegresL;
	vector <float>  vecEyeFlowPdfRegresR;
	vector <float>  vecEyeFlowPdfSubRegresL;
	vector <float>  vecEyeFlowPdfSubRegresR;
	
	
	vector <float>  vecLEarFlowPdf;
	vector <float>  vecREarFlowPdf;
	vector <float>  vecLEarFlowPdfRegres;
	vector <float>  vecREarFlowPdfRegres;
	vector <float>  vecLEarFlowPdfSubRegres;
	vector <float>  vecREarFlowPdfSubRegres;
	
	vector <float>  vecRedFlowPdf;
	vector <float>  vecCyanFlowPdf;
	vector <float>  vecRedFlowPdfRegres;
	vector <float>  vecCyanFlowPdfRegres;
	vector <float>  vecRedFlowPdfSubRegres;
	vector <float>  vecCyanFlowPdfSubRegres;
	
	
	int szVecFlow = m_vecFlow.size();
    if(bOpticalPDF ) {
        if(newFrameSteps != frameStep || szVecFlow ==0 || m_referFrame != newReferFrame || bNewOpFlowV1 != bOpFlowV1) {
            if(bOpFlowV1)
				opticalFlow_v1(newFrameSteps, newReferFrame);
			else
				opticalFlow_v2(newReferFrame);
        }
    }
	bOpFlowV1 = bNewOpFlowV1;
	frameStep = newFrameSteps;
	m_referFrame = newReferFrame;
    
	MainFrame:: myMsgOutput("y range [%d, %d], szROI %d, bSave %d\n", ymin, ymax, m_ROIsz, bSaveFile);	
	MainFrame:: myMsgOutput("PDF threshold %f, frame steps %d, opticalFlow v1 %d, eyeGain %.2f, bAccumulate %d\n", 
			threshold, frameStep, bOpFlowV1, gainEye, bAccumulate);
	
	vector <float>  vecRedGrayDiff;
	vector <float>  vecCyanGrayDiff;	
	
	vector <float>  vecLEarGrayDiff;
	vector <float>  vecREarGrayDiff;
	vector<float> smoothL;	
	vector<float> smoothR;
	int maxPointL, maxPointR;
	maxPointL = maxPointR = -1;
	if(bGrayDiff || bPinna) {
		if(m_bShowEar) {
			graylevelDiff(m_referFrame, ptEarL, ptEarR, vecLEarGrayDiff, vecREarGrayDiff);
			smoothData(vecLEarGrayDiff, smoothL, 2);	
			smoothData(vecREarGrayDiff, smoothR, 2);	
			
			maxPointL = findMaxMotionPoint(smoothL);
			maxPointR = findMaxMotionPoint(smoothR);	

			DC_removal(m_nLED1, vecLEarGrayDiff);
			DC_removal(m_nLED1, vecREarGrayDiff);
			
			Notch_removal(vecLEarGrayDiff, m_referFrame);
			Notch_removal(vecREarGrayDiff, m_referFrame);
		}
		if(m_bShowAbdo) {
			graylevelDiff(m_referFrame, ptRed, ptCyan, vecRedGrayDiff, vecCyanGrayDiff);
			DC_removal(m_nLED1, vecRedGrayDiff);
			DC_removal(m_nLED1, vecCyanGrayDiff);
			Notch_removal(vecCyanGrayDiff, m_referFrame);
			Notch_removal(vecRedGrayDiff, m_referFrame);			
		}
	}
	

	vector <Point> vpDrawPtRect;
    
	if(bOpticalPDF) {
		int sz = m_vecFlow.size();

        if(bSaveFile) {
            m_vmOpPDFMap.resize(sz);
            for(int i=0; i<sz; i++)
                m_vmOpPDFMap[i] = Mat::zeros(m_vecFlow[0].size(), CV_8UC1);
        }
		if(m_bShowEar) {
			opticalMovement(ptEarL, vecLEarFlowPdf, m_vmDistEarL, threshold, bOpFlowV1);
			opticalMovement(ptEarR, vecREarFlowPdf, m_vmDistEarR, threshold, bOpFlowV1);
			if(frameStep > 0 && bAccumulate) {			
				for(int i=1; i<sz; i++) {
					vecLEarFlowPdf[i] += vecLEarFlowPdf[i-1];
					vecREarFlowPdf[i] += vecREarFlowPdf[i-1];
				}
			}
			DC_removal(m_nLED1, vecLEarFlowPdf);
			DC_removal(m_nLED1, vecREarFlowPdf);
			
			Notch_removal(vecLEarFlowPdf, m_referFrame);
			Notch_removal(vecREarFlowPdf, m_referFrame);
			
			if(frameStep > 0 && bAccumulate) {
				linearRegression(vecLEarFlowPdf, vecLEarFlowPdfRegres, vecLEarFlowPdfSubRegres);
				linearRegression(vecREarFlowPdf, vecREarFlowPdfRegres, vecREarFlowPdfSubRegres);
			}
			
			if(bSaveFile) {
                opticalAssignThresholdMap(m_vmDistEarL, threshold, ptEarL);
                opticalAssignThresholdMap(m_vmDistEarR, threshold, ptEarR);
                vpDrawPtRect.push_back(ptEarL);
                vpDrawPtRect.push_back(ptEarR);
            }
		}
		
		if(m_bShowAbdo) {
			opticalMovement(ptRed, vecRedFlowPdf, m_vmDistRed, threshold, bOpFlowV1);
			opticalMovement(ptCyan, vecCyanFlowPdf, m_vmDistCyan, threshold, bOpFlowV1);
			if(frameStep > 0 && bAccumulate) {			
				for(int i=1; i<sz; i++) {
					vecRedFlowPdf[i] += vecRedFlowPdf[i-1];
					vecCyanFlowPdf[i] += vecCyanFlowPdf[i-1];
				}
			}
			DC_removal(m_nLED1, vecRedFlowPdf);
			DC_removal(m_nLED1, vecCyanFlowPdf);
			
			Notch_removal(vecRedFlowPdf, m_referFrame);
			Notch_removal(vecCyanFlowPdf, m_referFrame);
			
			if(frameStep > 0 && bAccumulate) {
				linearRegression(vecRedFlowPdf, vecRedFlowPdfRegres, vecRedFlowPdfSubRegres);
				linearRegression(vecCyanFlowPdf, vecCyanFlowPdfRegres, vecCyanFlowPdfSubRegres);
				
				int sz1 = vecRedFlowPdfSubRegres.size();
				for(int i=0; i<sz1; i++) {
					vecRedFlowPdfSubRegres[i] = vecRedFlowPdfSubRegres[i] /gainPDF;
					vecCyanFlowPdfSubRegres[i] = vecCyanFlowPdfSubRegres[i]/gainPDF;
				}		
			}
			if(bSaveFile) {
                opticalAssignThresholdMap(m_vmDistRed, threshold, ptRed);
                opticalAssignThresholdMap(m_vmDistCyan, threshold, ptCyan);
                vpDrawPtRect.push_back(ptRed);
                vpDrawPtRect.push_back(ptCyan);
            }            
		}
		
		if(m_bShowEye) {
			opticalMovement(ptEyeL, vecEyeFlowPdfL, m_vmDistEyeL, threshold, bOpFlowV1);
			opticalMovement(ptEyeR, vecEyeFlowPdfR, m_vmDistEyeR, threshold, bOpFlowV1);
			if(gainEye!=1) {
				for(int i=0; i<sz; i++) {
					vecEyeFlowPdfL[i] = vecEyeFlowPdfL[i]*gainEye;
					vecEyeFlowPdfR[i] = vecEyeFlowPdfR[i]*gainEye;
				}
			}
			if(frameStep > 0 && bAccumulate) {			
				for(int i=1; i<sz; i++) {
					vecEyeFlowPdfL[i] += vecEyeFlowPdfL[i-1];
					vecEyeFlowPdfR[i] += vecEyeFlowPdfR[i-1];
				}
				for(int i=1; i<sz; i++) {
					vecEyeFlowPdfL[i] /= frameStep;
					vecEyeFlowPdfR[i] /= frameStep;
				}				
			}
			
			//Notch_removal(vecEyeFlowPdfL, m_referFrame);
            //Notch_removal(vecEyeFlowPdfR, m_referFrame);
			if(frameStep > 0 && bAccumulate) {
				linearRegression(vecEyeFlowPdfL, vecEyeFlowPdfRegresL, vecEyeFlowPdfSubRegresL);
				linearRegression(vecEyeFlowPdfR, vecEyeFlowPdfRegresR, vecEyeFlowPdfSubRegresR);
				
				int sz1 = vecEyeFlowPdfSubRegresL.size();
				for(int i=0; i<sz1; i++) {
					vecEyeFlowPdfSubRegresL[i] = vecEyeFlowPdfSubRegresL[i] /gainEye;
					vecEyeFlowPdfSubRegresR[i] = vecEyeFlowPdfSubRegresR[i]/gainEye;
				}		
			}
			
            if(bSaveFile) {
                opticalAssignThresholdMap(m_vmDistEyeL, threshold, ptEyeL);
                opticalAssignThresholdMap(m_vmDistEyeR, threshold, ptEyeR);
                vpDrawPtRect.push_back(ptEyeL);
                vpDrawPtRect.push_back(ptEyeR);                
            } 
		}
        //////////////////////////////////////save file

        
        if(bSaveFile) {
            opticalDrawFlowmapWithPDF(vpDrawPtRect, frameStep);
            opticalSavePlot("scatter", "dots", threshold);
			opticalSavePlot("pdf", "lines", threshold);
        }
	}
	 
	wxEndBusyCursor(); 
	
///////////G N U P L O T//////////////////////////////////////////////////////////////////////////////
	wxFileName fileName = m_strSrcPath;
	const char* title =fileName.GetName();
	_gnuplotInit(gPlotL, title, ymin, ymax);
	_gnuplotInit(gPlotR, title, ymin, ymax);
	gPlotL.set_legend("left");
	gPlotR.set_legend("left");	
	gPlotL.cmd("set termoption noenhanced");
	gPlotR.cmd("set termoption noenhanced");
	
	if(bLED && m_nLED1>0 && m_nLED2 >0) {
		_gnuplotLED(gPlotL, m_nLED1, m_nLED2);
		_gnuplotLED(gPlotR, m_nLED1, m_nLED2);
	}
	if(bRefLine) {
		_gnuplotVerticalLine(gPlotL, m_referFrame);
		_gnuplotVerticalLine(gPlotR, m_referFrame);        
    }
	if(bPinna && maxPointL>=0 && maxPointR>=0) {
		if(vecLEarGrayDiff[maxPointL]> vecREarGrayDiff[maxPointR]) {
			MainFrame:: myMsgOutput("max motion: left ear %d\n", maxPointL);
			_gnuplotVerticalLine(gPlotL, maxPointL);
			_gnuplotVerticalLine(gPlotR, maxPointL);
			saveEarROI(m_referFrame, maxPointL, ptEarL);
		}else {
			MainFrame:: myMsgOutput("max motion: right ear %d\n", maxPointR);
			_gnuplotVerticalLine(gPlotL, maxPointR);
			_gnuplotVerticalLine(gPlotR, maxPointL);
			saveEarROI(m_referFrame, maxPointR, ptEarR);
		}	
	}
	
	if(bVerLine && verLine >0) {
		_gnuplotVerticalLine(gPlotL, verLine);
		_gnuplotVerticalLine(gPlotR, verLine);		
	}
	
	if(bEyeMove) {
		_gnuplotLine(gPlotL, "LEyePosMove", m_vecEyeLMove, "#008B0000", ".");
		_gnuplotLine(gPlotR, "REyePosMove", m_vecEyeRMove, "#008B0000", ".");
	
	}
	if(bGrayDiff) {
		if(m_bShowEar) {
			_gnuplotLine(gPlotL, "LEarGraylevelDiff", vecLEarGrayDiff, "#000000ff", ".");
			_gnuplotLine(gPlotR, "REarGraylevelDiff", vecREarGrayDiff, "#000000ff", ".");
		}
		if(m_bShowAbdo) {
            //_OutputVec(vecRedGrayDiff, "_redgray.csv");
			_gnuplotLine(gPlotL, "APB-Red GraylevelDiff", vecRedGrayDiff, "#00008000", ".");
			_gnuplotLine(gPlotR, "APB-Cyan GraylevelDiff", vecCyanGrayDiff, "#00008000", ".");
		}
	}
	if(bOpticalPDF) {
		if(m_bShowEar) {
			_gnuplotLine(gPlotL, "LEarPDF", vecLEarFlowPdf, "#000000ff");
			_gnuplotLine(gPlotR, "REarPDF", vecREarFlowPdf, "#000000ff");
			
			if(frameStep > 0 && bAccumulate ) {
				_gnuplotLine(gPlotL, "LEarPDFRegress", vecLEarFlowPdfRegres, "#000080FF", "-");
				_gnuplotLine(gPlotR, "REarPDFRegress", vecREarFlowPdfRegres, "#000080FF", "-");
				
				_gnuplotLine(gPlotL, "LEarPDF-Regress", vecLEarFlowPdfSubRegres, "#00FF0000");
				_gnuplotLine(gPlotR, "REarPDF-Regress", vecREarFlowPdfSubRegres, "#00FF0000");
			}
		}
		if(m_bShowAbdo) {
            //_OutputVec(vecRedFlowPdf, "_redw.csv");
			_gnuplotLine(gPlotL, "APB-Red FlowPDF", vecRedFlowPdf, "#00008000");
			_gnuplotLine(gPlotR, "APB-Cyan FlowPDF", vecCyanFlowPdf, "#00008000");
			
			if(frameStep > 0 && bAccumulate ) {
				_gnuplotLine(gPlotL, "APB-Red PDFRegress", vecRedFlowPdfRegres, "#00F000FF", "-");
				_gnuplotLine(gPlotR, "APB-Cyan PDFRegress", vecCyanFlowPdfRegres, "#00F000FF", "-");
				
				_gnuplotLine(gPlotL, "APB-Red PDF-Regress", vecRedFlowPdfSubRegres, "#00FF8000");
				_gnuplotLine(gPlotR, "APB-Cyan PDF-Regress", vecCyanFlowPdfSubRegres, "#00FF8000");
			}	
		}
		if(m_bShowEye) {
			_gnuplotLine(gPlotL, "HeadMoveL", vecEyeFlowPdfL, "#008B0000");
			_gnuplotLine(gPlotR, "HeadMoveR", vecEyeFlowPdfR, "#008B0000");				
			if(frameStep > 0 && bAccumulate ) {
				_gnuplotLine(gPlotL, "EyeL PDFRegress", vecEyeFlowPdfRegresL, "#00B8860B", "-");
				_gnuplotLine(gPlotR, "EyeR PDFRegress", vecEyeFlowPdfRegresR, "#00B8860B", "-");
				
				_gnuplotLine(gPlotL, "EyeL PDF-Regress", vecEyeFlowPdfSubRegresL, "#00FF00FF");
				_gnuplotLine(gPlotR, "EyeR PDF-Regress", vecEyeFlowPdfSubRegresR, "#00FF00FF");
			}	
		}		
	}
	
	drawOnDestImage(bSaveFile);
	
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	int minutes = duration / 60;
	int second = duration - minutes * 60;
	MainFrame::myMsgOutput("OnRatProcessEar: computation time: %02dm:%02ds\n", minutes, second);
	
	
	return true;
}
void CRat::drawOnDestImage(bool bSaveFile)
{
	////////////////////// save result to dest
	m_vecDest.resize(m_nSlices);
	Point ptL1 (m_ptEarL-m_offsetEar);
	Point ptL2 (m_ptEarL+m_offsetEar);
	Point ptR1 (m_ptEarR-m_offsetEar);
	Point ptR2 (m_ptEarR+m_offsetEar);
	
	Point ptD1 (m_ptRed-m_offsetEar);
	Point ptD2 (m_ptRed+m_offsetEar);
	Point ptC1 (m_ptCyan-m_offsetEar);
	Point ptC2 (m_ptCyan+m_offsetEar);
	
	Point ptEyeL1 (m_ptEyeL-m_offsetEar);
	Point ptEyeL2 (m_ptEyeL+m_offsetEar);
	Point ptEyeR1 (m_ptEyeR-m_offsetEar);
	Point ptEyeR2 (m_ptEyeR+m_offsetEar);
	
	for (int i = 0; i < m_nSlices; i++)
		cvtColor(m_vecMat[i], m_vecDest[i], CV_GRAY2BGR);

	for (int i = 0; i < m_nSlices; i++)
	{
		Mat mDestColor;
		mDestColor = m_vecDest[i];
		// original ears
        if(m_bShowEar) {
            rectangle(mDestColor, Rect(ptL1, ptL2), Scalar(255,0, 0));
            rectangle(mDestColor, Rect(ptR1, ptR2), Scalar(255,0, 0));
        }
        if(m_bShowAbdo) {
            rectangle(mDestColor, Rect(ptD1, ptD2), Scalar(0,128,0));
            rectangle(mDestColor, Rect(ptC1, ptC2), Scalar(0,128,0));
        }
		if(m_bShowEye) {
            rectangle(mDestColor, Rect(ptEyeL1, ptEyeL2), Scalar(255, 0,255));
            rectangle(mDestColor, Rect(ptEyeR1, ptEyeR2), Scalar(255, 0,255));
        }
		// original eyes
		circle(mDestColor, Point(m_ptEyeL.x, m_ptEyeL.y), 2, Scalar(0, 0, 255), -1);	
		circle(mDestColor, Point(m_ptEyeR.x, m_ptEyeR.y), 2, Scalar(0, 0, 255), -1);	
	}

	if(bSaveFile)
		saveResult("dest", m_vecDest);	
}
void CRat::opticalMovement(Point pt, vector <float>& vecPdfMove, vector <Mat>& vecmDist, float threshold, bool bOpFlowV1)
{
	cv::Mat	 cvMat;
	cv::Mat	 cvMatGray;

	if(m_vecFlow.size() ==0) return;

	double sigma = -1;
	int ksize = 9;
	Mat mGaus ; 
	createGaussianMask(sigma, ksize, mGaus);
	
	int sz = m_vecFlow.size();
	vecmDist.resize(sz);
	vecPdfMove.resize(sz);
	
	for(int i=0; i<sz; i++) {
		Mat& mFlow = m_vecFlow[i];
		vecmDist[i] = Mat::zeros(PDF_SIZE, PDF_SIZE, CV_32FC1);
		opticalBuildPDF(mFlow, mGaus, vecmDist[i], pt);
		if(bOpFlowV1)
			vecPdfMove[i] = optical_compute_movement_v1(mFlow, vecmDist[i], pt, threshold);	
		else
			vecPdfMove[i] = optical_compute_movement_v2(mFlow, vecmDist[i], pt, threshold);	
	}	
}
void CRat::linearRegression(vector <float>& vecSignal, vector <float>& vecOut, vector <float>& vecSubOut)
{
	int i, pivot, dataLen, len, c;
	double *seg, *x;
	double c0, c1, cov00, cov01, cov11, sumsq, mse;
	
	len = m_nLED1;
	seg = new double[len]; 
	x = new double[len]; 
	
	for(i=0; i<len; i++) {
		seg[i] = vecSignal[i];
		x[i] = i;
	}
			
	gsl_fit_linear(x, 1, seg, 1, len, &c0, &c1, &cov00, &cov01, &cov11, &sumsq);
	mse = sumsq/len;	
	delete [] seg;
	delete [] x;
	
	int sz = vecSignal.size();
	vecOut.resize(sz);
	vecSubOut.resize(sz);
	
	for(int i=0; i<sz; i++) {
		vecOut[i] = c0 + c1*i;
		vecSubOut[i] = vecSignal[i] - vecOut[i]; 
	}
	
}
void  CRat::findNewEarCenter(vector <Point>& vecEye, Point ptEar0, vector <Point>& vecEar, int referFrame)
{
	vecEar.clear();	
	vecEar.resize(m_nSlices);	
	
	Point ptReferEye = vecEye[referFrame];
	for (int i = 0; i < m_nSlices; i++)	{
		Point ptOffset = vecEye[i] - ptReferEye;
		vecEar[i] = ptEar0 + ptOffset;	
	}
}
void CRat::findEyeCenter(Point& ptEye0, vector <Point>& vecEye, vector <float>&  vecEyeMove, int referFrame)
{
	
//	vecEye.clear();	
	vecEye.resize(m_nSlices);

//	vecEyeMove.clear();	
	vecEyeMove.resize(m_nSlices);	
	
	Point	offset(20, 20);
	Point 	ptEye = ptEye0;
	double sigma = 4;	
	
	for (int i = 0; i < m_nSlices; i++)	{	
		Point pt1 (ptEye-offset);
		Point pt2 (ptEye+offset);
	
		Mat mROI(m_vecMat[i], Rect(pt1, pt2));	
		Mat mROI32F, mSmooth;
	
		mROI.convertTo(mROI32F, CV_32F);
		cv::GaussianBlur(mROI32F, mSmooth, Size(), sigma, sigma);
		//_OutputMat(mSmooth, "_eyesm3.dat", false);

		Mat mROIsm(mSmooth, Rect(Point(10, 10), Point(30, 30)));
		double min;
		Point minLoc;
		try {
		minMaxLoc(mROIsm, &min, NULL, &minLoc);
		
		}catch(cv::Exception& e){
			wxString str;
			str.Printf("Bad allocation: %s", e.what() );
			wxLogMessage( str);
			continue;
		}
		minLoc = minLoc + Point(10, 10) + pt1;
	//MainFrame:: myMsgOutput("eye center [%d, %d]= %f, ori eye [%d, %d]\n", minLoc.x, minLoc.y, min, ptEye.x, ptEye.y);	
	
		ptEye = minLoc;
		
		vecEye[i] = ptEye;
		
	}
	
	Point eyeRef = vecEye[referFrame];
	
	for (int i = 0; i < m_nSlices; i++) {
		vecEyeMove[i] = sqrt(	(vecEye[i].x-eyeRef.x)*(vecEye[i].x-eyeRef.x) + 
								(vecEye[i].y-eyeRef.y)*(vecEye[i].y-eyeRef.y));
		
	}	
	
}
void CRat::saveEyeTrajectory()
{
	Point ptL = m_vecEyeL[0];
	Point ptR = m_vecEyeR[0];
	Point ptC((ptL.x + ptR.x)/2, (ptL.y + ptR.y)/2);
	
	Point	offsetWH(100, 100); 

	int wImg = m_vecMat[0].cols;
	int hImg = m_vecMat[0].rows;
	
	Point pt1 (ptC-offsetWH);
	Point pt2 (ptC+offsetWH);
/*	
	vector <Mat> vecTraj;
	for(int i=0; i<m_nSlices; i++) {
		Mat mTraj(200, 200, CV_8UC3, Scalar(0, 0, 0));
		
		for(int j=0; j<i; j++) {
			Point ptL = m_vecEyeL[j] - pt1;
			Point ptR = m_vecEyeR[j] - pt1;	
			circle(mTraj, Point(ptL.x, ptL.y), 1, Scalar(0, 255, 255), -1);	
			circle(mTraj, Point(ptR.x, ptR.y), 1, Scalar(0, 255, 255), -1);				
		}
		
		Point ptL = m_vecEyeL[i] - pt1;
		Point ptR = m_vecEyeR[i] - pt1;	
		circle(mTraj, Point(ptL.x, ptL.y), 1, Scalar(0, 0, 255), -1);	
		circle(mTraj, Point(ptR.x, ptR.y), 1, Scalar(0, 0, 255), -1);	
		
		vecTraj.push_back(mTraj);
	}	
	saveResult("eyeT", vecTraj);
*/
	int i;
	Mat mTraj(200, 200, CV_8UC3, Scalar(0, 0, 0));
	for(i=0; i<m_nSlices; i++) {
		ptL = m_vecEyeL[i] - pt1;
		ptR = m_vecEyeR[i] - pt1;	
		circle(mTraj, Point(ptL.x, ptL.y), 0, Scalar(0, 255, 255), -1);	
		circle(mTraj, Point(ptR.x, ptR.y), 0, Scalar(0, 255, 255), -1);				
	}	
	ptL = m_vecEyeL[i-1] - pt1;
	ptR = m_vecEyeR[i-1] - pt1;	
	circle(mTraj, Point(ptL.x, ptL.y), 1, Scalar(0, 0, 255), -1);	
	circle(mTraj, Point(ptR.x, ptR.y), 1, Scalar(0, 0, 255), -1);

	wxFileName bmpName(m_strSrcPath, "_eyeTraj.bmp");		
	imwrite(bmpName.GetFullPath().ToStdString(), mTraj);	

	imshow("eyeTrajectory", mTraj);
}
void CRat::saveEarROI(int stable, int motion, Point& pt)
{
	int wImg = m_vecMat[0].cols;
	int hImg = m_vecMat[0].rows;
	
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= wImg) pt2.x = wImg-1;
	if(pt2.y >= hImg) pt2.y = hImg-1;
	
	Mat mROI0(m_vecMat[stable], Rect(pt1, pt2));	
	Mat mROI1(m_vecMat[motion], Rect(pt1, pt2));
	
	Mat mEdge0, mEdge1, mEdge, mEdge8U, mEdge08U, mEdge18U;
	cv::Mat dx;
	cv::Mat dy;
	double  min, max, a;
	int kSize = 5;

	cv::Sobel(mROI0, dx, CV_32F, 1, 0, kSize);
	cv::Sobel(mROI0, dy, CV_32F, 0, 1, kSize);
	mEdge0 = abs(dx) + abs(dy);

	cv::Sobel(mROI1, dx, CV_32F, 1, 0, kSize);
	cv::Sobel(mROI1, dy, CV_32F, 0, 1, kSize);
	mEdge1 = abs(dx) + abs(dy);	
	
	mEdge = mEdge0 + mEdge1;
	
	cv::minMaxLoc(mEdge, &min, &max);
	a = 255./(max - min);
	mEdge.convertTo(mEdge8U, CV_8UC1, a, -min*a );
	
	cv::minMaxLoc(mEdge0, &min, &max);
	a = 255./(max - min);	
	mEdge0.convertTo(mEdge08U, CV_8UC1, a, -min*a );
	
	cv::minMaxLoc(mEdge1, &min, &max);
	a = 255./(max - min);		
	mEdge1.convertTo(mEdge18U, CV_8UC1, a, -min*a );
	
	Mat mColor (mROI0.size(), CV_8UC3, Scalar(0,0,0)) ;
	vector<cv::Mat> planes(3);
	cv::split(mColor, planes);
	mEdge08U.copyTo(planes[1]);
	mEdge18U.copyTo(planes[2]);
	merge(planes, mColor);	
	
//	wxFileName bmpName1(m_strSrcPath, "_stable.bmp");
//	wxFileName bmpName2(m_strSrcPath, "_motion.bmp");
	wxFileName bmpName3(m_strSrcPath, "_edge.bmp");
		
//	imwrite(bmpName1.GetFullPath().ToStdString(), mROI0);
//	imwrite(bmpName2.GetFullPath().ToStdString(), mROI1);
	imwrite(bmpName3.GetFullPath().ToStdString(), mColor);
	
//	imshow("_stable", mROI0);
//	imshow("_motion", mROI1); 
	imshow("_ColorEdge", mColor);
}
int CRat::findMaxMotionPoint(vector<float>& inData)
{
	int maxPoint = -1;
	
	for(int i=m_nLED2+3; i<m_nSlices; i++) {
		if( inData[i-2] <=inData[i-1] && 
			inData[i-1] <=inData[i] &&
			inData[i] >= inData[i+1] &&
			inData[i+1] >= inData[i+2]) 
		{
			maxPoint = i;			
			break;	
		}		
	}	
	return maxPoint;
}

int CRat::findReferenceFrame(Point& pt)
{
	int start, end, lenSeg;
	start = 0;
    if(m_nLED2>0)
        end = m_nLED2 + 10;
    else
        end = m_nSlices/2;
        
	lenSeg = 10;
	
	vector <double>  vSeries;
	
    vSeries.resize(end);
    
	Mat mSrc1, mSrc2, mDiff;
    m_vecMat[end/2].convertTo(mSrc2, CV_16S);
    double mu;
	for(int i=0; i<end; i++) {
		m_vecMat[i].convertTo(mSrc1, CV_16S);        
        mDiff = mSrc1 - mSrc2;
        mu = errorSum(mDiff, pt);
        vSeries[i] = mu;
    }
    
  	int  idxMin= -1;  
    double min = vSeries[0];
    for(int i=0; i<end; i++) {
        if(vSeries[i] < min) {
            min = vSeries[i];
            idxMin = i;
        }
	}
	return idxMin;
}

double CRat::errorSum(Mat &mDiff, Point ptEarL)
{
	int wImg = m_vecMat[0].cols;
	int hImg = m_vecMat[0].rows;
	
	Point pt1 (ptEarL-m_offsetEar);
	Point pt2 (ptEarL+m_offsetEar);
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x <0) pt2.x = 0;
	if(pt2.y <0) pt2.y = 0;
	
	if(pt1.x >= wImg) pt1.x = wImg-1;
	if(pt1.y >= hImg) pt1.y = hImg-1;	
	if(pt2.x >= wImg) pt2.x = wImg-1;
	if(pt2.y >= hImg) pt2.y = hImg-1;
	
	Mat mROI(mDiff, Rect(pt1, pt2));
	
	Scalar sSum = cv::mean(mROI);
	
	return sSum[0];
}

void CRat::smoothData(vector<float>& inData, vector<float>& outData, int bw)
{
	int  n = inData.size();	

	double* pData = new double[n];
	double  *out = new double[n];

	for(int i=0; i<n; i++)  pData[i] = inData[n];
	
	CKDE::MSKernel kde_kernel = CKDE::Gaussian;
	
	CKDE kde1x(pData, out, n);
	kde1x.KernelDensityEstimation(kde_kernel, bw);

	outData.resize(n);
	for(int i=0; i<n; i++)  outData[i] = out[n];
	//outData.assign (out,out+n);	
	
	delete [] pData;
	delete [] out;
}

void CRat::graylevelDiff_Eye(int refer, Point ptEar, vector <Point>& vecEye, vector <float>& vecEarGrayDiff)
{
	int wImg = m_vecMat[0].cols;
	int hImg = m_vecMat[0].rows;
	
	Point pt1 (ptEar-m_offsetEar);
	Point pt2 (ptEar+m_offsetEar);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x <0) pt2.x = 0;
	if(pt2.y <0) pt2.y = 0;
	
	if(pt1.x >= wImg) pt1.x = wImg-1;
	if(pt1.y >= hImg) pt1.y = hImg-1;	
	if(pt2.x >= wImg) pt2.x = wImg-1;
	if(pt2.y >= hImg) pt2.y = hImg-1;
	
	
	if(pt1.x <0 || pt1.y <0 || pt2.x >= wImg || pt2.y >= hImg) {
		wxLogMessage("ear region outside");
		return;
	}
    
	Mat mSrc1;
    m_vecMat[refer].convertTo(mSrc1, CV_16S);		
	Mat mROIRefer(mSrc1, Rect(pt1, pt2));

	Point ptReferEye = vecEye[refer];
	
	vecEarGrayDiff.resize(m_nSlices);	
	for(int i=0; i<m_nSlices; i++) {
		Mat mSrc2, mDiff;
		m_vecMat[i].convertTo(mSrc2, CV_16S);;
		
		Point ptOffset = vecEye[i] - ptReferEye;
		Point ptNewEar = ptEar + ptOffset;
		
		Point pt1 (ptNewEar-m_offsetEar);
		Point pt2 (ptNewEar+m_offsetEar);
	
		if(pt1.x <0 || pt1.y <0 || pt2.x >= wImg || pt2.y >= hImg) {
			wxLogMessage("ear region outside");
			return;
		}
		Mat mROI(mSrc2, Rect(pt1, pt2));
		cv::absdiff(mROIRefer, mROI, mDiff);
		Scalar sSum = cv::mean(mDiff);		
		vecEarGrayDiff[i] = sSum[0];
	}
}

void CRat::imageDiff(vector<Mat>& vecMatDiff, vector <float>& vecAdjDiff, int nFrameSteps)
{
	vecMatDiff.resize(m_nSlices - nFrameSteps);
	vecAdjDiff.resize(m_nSlices - nFrameSteps);
	int imgSz = m_vecMat[0].rows * m_vecMat[0].cols;
	
	for(int i=0; i<m_nSlices - nFrameSteps; i++) {
		Mat mSrc1, mSrc2, mDiff;
		if(nFrameSteps==0)
			m_vecMat[m_referFrame].convertTo(mSrc1, CV_16S);
		else
			m_vecMat[i].convertTo(mSrc1, CV_16S);;
			
		m_vecMat[i+nFrameSteps].convertTo(mSrc2, CV_16S);;
		cv::absdiff(mSrc1, mSrc2, mDiff);
		//cv::threshold(mDiff, mDiff,0, 255, cv::THRESH_BINARY); 
		
		vecMatDiff[i] = mDiff;
		
		Scalar sSum = cv::sum(mDiff);
		vecAdjDiff[i] = sSum[0]/(imgSz);
	}
}
void CRat::pointGraylevel(Point ptAbdoRed, Point ptAbdoCyan, vector <float>& vecRedPoint, vector <float>& vecCyanPoint)
{
	vecRedPoint.resize(m_nSlices);
	vecCyanPoint.resize(m_nSlices);

	for(int i=0; i<m_nSlices; i++) {
		Mat mSrc = m_vecMat[i];
		vecRedPoint[i] = mSrc.at<uchar>(ptAbdoRed.y, ptAbdoRed.x);
		vecCyanPoint[i] = mSrc.at<uchar>(ptAbdoCyan.y, ptAbdoCyan.x);
	}
	
}
void CRat::graylevelDiff(int refer, Point& ptEarL, Point& ptEarR, vector <float>& vLEarGray,  vector <float>& vREarGray)
{
	vLEarGray.resize(m_nSlices);
	vREarGray.resize(m_nSlices);
	
	Mat mSrc1, mSrc2, mDiff;
	
	m_vecMat[refer].convertTo(mSrc1, CV_16S);;
	
	for(int i=0; i<m_nSlices; i++) {
		double errSumL, errSumR;
		m_vecMat[i].convertTo(mSrc2, CV_16S);
		cv::absdiff(mSrc1, mSrc2, mDiff);
		
		errSumL = errorSum(mDiff, ptEarL);
		errSumR = errorSum(mDiff, ptEarR);
		vLEarGray[i] = errSumL;
		vREarGray[i] = errSumR;
	}
}
void CRat::opticalFlow_v2(int referFrame)
{
	// optical flow
	double pyr_scale = 0.5;
	int nIter = 3;
	int levels= 1;
	int nWinSize = 3;// 3;
	int poly_n = 7; // 7
	double dblSigma = 1.5;
    
    int step = 5;

	m_vecFlow.resize(m_nSlices);
	Mat mSrc1, mSrc2;

	m_vecFlow[referFrame] = Mat::zeros(m_vecMat[0].size(),CV_32FC2);
	for(int i=referFrame; i>=0; i-=step) {
		mSrc1 = m_vecMat[i];
		for(int k=1; k<=step && i-k>=0; k++) {
			mSrc2 = m_vecMat[i-k];
			Mat& mFlow = m_vecFlow[i-k];
			calcOpticalFlowFarneback(mSrc1, mSrc2, mFlow, pyr_scale, levels, nWinSize, nIter, poly_n, dblSigma, 0);
			if(i<referFrame)
				m_vecFlow[i-k] += m_vecFlow[i];
		}
	}
    
	for(int i=referFrame; i<m_nSlices; i+=step) {
		mSrc1 = m_vecMat[i];
		for(int k=1; k<=step && i+k<m_nSlices; k++) {
			mSrc2 = m_vecMat[i+k];
			Mat& mFlow = m_vecFlow[i+k];
			calcOpticalFlowFarneback(mSrc1, mSrc2, mFlow, pyr_scale, levels, nWinSize, nIter, poly_n, dblSigma, 0);
			if(i>referFrame)
				m_vecFlow[i+k] += m_vecFlow[i];
		}
	}

}
void CRat::opticalFlow_v1(int nFrameSteps, int refFrame)
{
	// optical flow
	double pyr_scale = 0.5;
	int nIter = 3;
	int levels= 1;
	int nWinSize = 3;// 3;
	int poly_n = 7; // 7
	double dblSigma = 1.5;

	m_vecFlow.resize(m_nSlices  - nFrameSteps);
	
//	clock_t start, finish;
//	double  duration;
//	start = clock();
	
//#pragma omp parallel for 
	for(int i=0; i<m_nSlices - nFrameSteps; i++) {
		int referFrame;
		if(nFrameSteps==0)  referFrame= refFrame;
		else referFrame = i;
		
		Mat mSrc1, mSrc2;
		mSrc1 = m_vecMat[referFrame];
		mSrc2 = m_vecMat[i+nFrameSteps];

		Mat& mFlow = m_vecFlow[i];

        calcOpticalFlowFarneback(mSrc1, mSrc2, mFlow, pyr_scale, levels, nWinSize, nIter, poly_n, dblSigma, 0);
	}
	/*
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	int minutes = duration / 60;
	int second = duration - minutes * 60;
	MainFrame:: myMsgOutput("Opticalflow computation time: %02dm:%02ds\n", minutes, second);
*/
}
bool CRat::opticalLoadPDFfile(const char* filename, Mat &mPdf)
{
    float  probability;
    FILE* fpPdf = fopen(filename, "r");
    if(fpPdf ==NULL) {
        //wxString str;
        //str << filename << ": read error";
        //wxLogMessage(str);
        return false;
    }
    for(int i=0; i<PDF_SIZE; i++)
        for(int c=0; c<PDF_SIZE; c++) {
            fscanf(fpPdf, "%f", &probability);
            mPdf.at<float>(i, c) = probability;
        }
    fclose(fpPdf);  

    return true;
}

void CRat::opticalAssignThresholdMap(vector<Mat>& vmPDF, float th, Point pt)
{
    Point p1 (pt-m_offsetEar);
    Point p2 (pt+m_offsetEar);
	
	if(p1.x <0) p1.x = 0;
	if(p1.y <0) p1.y = 0;
	if(p2.x >= m_vecFlow[0].cols) p2.x = m_vecFlow[0].cols-1;
	if(p2.y >= m_vecFlow[0].rows) p2.y = m_vecFlow[0].rows-1;
	
    int sz = m_vecFlow.size();

    for(int i=0; i<sz; i++) {
        Mat& mFlow = m_vecFlow[i]; 
        Mat& mPdf = vmPDF[i];
        Mat& mThMap = m_vmOpPDFMap[i];
        
        Mat mROI(mFlow, Rect(p1, p2));
		int count = 0;
        for(int y = 0; y < mROI.rows; y ++) {
            for(int x = 0; x < mROI.cols; x ++)
            {
                Point2f fxy = mROI.at<Point2f>(y, x);
                Point2f fxy1 = fxy + Point2f(PDF_SIZE/2, PDF_SIZE/2);
                
                if(fxy1.x >= PDF_SIZE || fxy1.x <0)  {
//                    wxString str;
//                    str.Printf("[%f, %f]", fxy1.x, fxy1.y);
//                    wxLogMessage(str);
                    continue;
                }
                if(fxy1.y >= PDF_SIZE || fxy1.y <0)  continue;
                
                float probability = mPdf.at<float>(fxy1.y+0.5, fxy1.x+0.5);
                if(probability >= th) 	{
                    mThMap.at<uchar>(y+p1.y,x+p1.x) = 1;
					count ++;
				}
            } 
		}
    }
}

void CRat::opticalDrawFlowmapWithPDF(vector<Point>& vpDrawPtRect, int nFrameSteps)
{
    int sz = m_vecFlow.size();
    
	vector <Mat> vecFlowmap(sz);

	for(int i=0; i<sz; i++) {
        Mat& mThMap = m_vmOpPDFMap[i];
        Mat& mFlow = m_vecFlow[i]; 
        
		Mat mSrc2 = m_vecMat[i+nFrameSteps];				
		Mat& mFlowmapColor = vecFlowmap[i];
		
        cvtColor(mSrc2, mFlowmapColor, CV_GRAY2BGR);
		drawOptFlowMapWithPDF(mFlowmapColor, mFlow, 4, mThMap);

        for(int k=0; k<vpDrawPtRect.size(); k++) {
            Point pt = vpDrawPtRect[k];
            cv::rectangle(mFlowmapColor,pt-m_offsetEar,pt+m_offsetEar, cv::Scalar(0, 255, 255)); 
            cv::circle(mFlowmapColor, pt, 2, Scalar(255, 255, 0), -1);	       
        }
	}

	saveResult("flow", vecFlowmap);	
}


void CRat::drawOptFlowMapWithPDF(Mat& cflowmap, const Mat& flow,  int step, Mat &mPdfMap)
{
    const Scalar& colorIn = cv::Scalar(255, 0, 0);
    const Scalar& colorOut = cv::Scalar(0, 0, 255);
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            if( mPdfMap.at<uchar>(y, x) >0 )
                line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), colorIn);
            else
                line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), colorOut);
            //circle(cflowmap, Point(x,y), 1, color, -1);
        }
}
void CRat::opticalDrawFlowmap(Point pt1, Point pt2, int nFrameSteps, char type)
{
	vector <Mat> vecFlowmap;
	vecFlowmap.clear();
	vecFlowmap.resize(m_nSlices  - nFrameSteps);

	Point ptEyeC;
	if(type =='E') {
		ptEyeC.x= (m_vecEyeL[m_referFrame].x+m_vecEyeR[m_referFrame].x)/2;
		ptEyeC.y= (m_vecEyeL[m_referFrame].y+m_vecEyeR[m_referFrame].y)/2;
	}
	for(int i=0; i<m_nSlices - nFrameSteps; i++) {
		Mat mSrc2;
		mSrc2 = m_vecMat[i+nFrameSteps];
				
		Mat& mFlow = m_vecFlow[i];
		Mat& mFlowmapColor = vecFlowmap[i];
		
        cvtColor(mSrc2, mFlowmapColor, CV_GRAY2BGR);
		drawOptFlowMap(mFlowmapColor, mFlow, 4, CV_RGB(0, 128, 0));

		cv::rectangle(mFlowmapColor,pt1-m_offsetEar, pt1+m_offsetEar, cv::Scalar(0, 0, 255));
		cv::rectangle(mFlowmapColor,pt2-m_offsetEar, pt2+m_offsetEar, cv::Scalar(0, 0, 255));
		if(type =='E') {
			// new positions of eyes 
			circle(mFlowmapColor, Point(m_vecEyeL[i].x, m_vecEyeL[i].y), 3, Scalar(0, 0, 255), -1);	
			circle(mFlowmapColor, Point(m_vecEyeR[i].x, m_vecEyeR[i].y), 3, Scalar(0, 0, 255), -1);
			// reference eyes
			circle(mFlowmapColor, Point(m_vecEyeL[m_referFrame].x, m_vecEyeL[m_referFrame].y), 2, Scalar(0, 255, 255), -1);	
			circle(mFlowmapColor, Point(m_vecEyeR[m_referFrame].x, m_vecEyeR[m_referFrame].y), 2, Scalar(0, 255, 255), -1);	
	   
			cv::rectangle(mFlowmapColor,ptEyeC-m_offsetEar,ptEyeC+m_offsetEar, cv::Scalar(255, 0, 255));
		}
	}

	saveResult("flow", vecFlowmap);	
}

bool CRat::prepareGnuPlot(Gnuplot& plotSave, int numPlots, char* subpath)
{
    wxString str;
    
    if(numPlots!= 2 && numPlots!= 4 && numPlots!= 6) {
        str.Printf("prepareSaveScatterPlot: numPlots error, %d", numPlots);
        wxLogMessage(str);
		return false;   
    }
    
	wxString outpath; 
#if defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__) 
	outpath = m_strSrcPath + "/" + subpath;
#else
	outpath = m_strSrcPath + "\\" + subpath;
#endif	

	if(m_vecFlow.size() ==0) return false;

	if(wxDirExists(outpath)==false) {
		if(wxMkdir(outpath)==false) {
			str.Printf("Create output directory error, %s", outpath);
			wxLogMessage(str);
			return false;
		}
	}
	
#if defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__) 
    if(numPlots==2) 
        plotSave.cmd("set terminal png size 800, 400");
    else if(numPlots==4) 
        plotSave.cmd("set terminal png size 800, 800");
    else if(numPlots==6) 
        plotSave.cmd("set terminal png size 800, 1200");
#else
    if(numPlots==2) 
        plotSave.cmd("set terminal pngcairo size 800, 400");
    else if(numPlots==4) 
        plotSave.cmd("set terminal pngcairo size 800, 800");
    else if(numPlots==6) 
        plotSave.cmd("set terminal pngcairo size 800, 1200");
#endif	

    int range = PDF_SIZE/2;
	plotSave.cmd("set grid");
	plotSave.cmd("unset key");	
	plotSave.set_xrange(-range, range);
	plotSave.set_yrange(-range, range);
	plotSave.cmd("set size 1,1");
	plotSave.cmd("set origin 0,0");
	plotSave.cmd("set termoption noenhanced");
	
	if(strcmp("pdf", subpath)==0 ){
		plotSave.set_zrange(0,0.15);	
		plotSave.cmd("set vi 60,30");
		plotSave.set_hidden3d();
		plotSave.cmd("set style data lines");
	}
	
	wxFileName fileName = wxString(m_vFilenames[0]);
	int dirCount = fileName.GetDirCount();
	wxArrayString dirs = fileName.GetDirs();
	wxString dirName = dirs[dirCount-1];  

    return true;
}

void CRat::opticalSavePlot(char* subpath, char* type, float threshold)
{
    int numScatterPlot = 0;
    Gnuplot plotSave(type);
    if(m_bShowEar)  numScatterPlot+=2;
    if(m_bShowAbdo)  numScatterPlot+=2;
    if(m_bShowEye)  numScatterPlot+=2;
    bool bRet = prepareGnuPlot(plotSave, numScatterPlot, subpath);   
    
    if(bRet ==false) return;
    
	wxFileName fileName = wxString(m_vFilenames[0]);
	int dirCount = fileName.GetDirCount();
	wxArrayString dirs = fileName.GetDirs();
	wxString dirName = dirs[dirCount-1];
    
    int sz = m_vecFlow.size();
    for(int i=0; i<sz; i++) {
        // assign save filename
        wxFileName fileName = wxString(m_vFilenames[i]);
        fileName.AppendDir(subpath);
        wxFileName saveName(fileName.GetPath(), fileName.GetName());
        
        /////////////////////////////////////////////// plot command
        std::ostringstream cmdstr0;
        cmdstr0 << "set output '" << saveName.GetFullPath().ToAscii() << ".png'";
        plotSave.cmd(cmdstr0.str());

        std::ostringstream cmdstr1;
        cmdstr1 << "set multiplot title '" << dirName.ToAscii() << "/" << fileName.GetName().ToAscii()  ;
        cmdstr1 << ", th= " << threshold << "'";
        plotSave.cmd(cmdstr1.str());
      
        float szX, szY, oriX, oriY;
        szX = 0.5;
        if(numScatterPlot ==2) {
            szY = 1;
            if(m_bShowEar) {
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEarL, m_vmDistEarL,
                        "_EarL", "Left Ear", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEarR, m_vmDistEarR,
                        "_EarR", "Right Ear", threshold, szX, szY, oriX, oriY); 
            }
            if(m_bShowAbdo) {
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptRed, m_vmDistRed,
                        "_Red", "Red", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptCyan, m_vmDistCyan,
                        "_Cyan", "Cyan", threshold, szX, szY, oriX, oriY); 
            }  
            if(m_bShowEye) {
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEyeL, m_vmDistEyeL,
                        "_EyeL", "Left Eye", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEyeR, m_vmDistEyeR,
                        "_EyeR", "Right Eye", threshold, szX, szY, oriX, oriY); 
            }                   
        }else if(numScatterPlot ==4) {
            szY = 0.5;
            if(m_bShowEar && m_bShowAbdo) {
                oriX = 0; oriY = 0.5;
                plotOneSpot(type, plotSave, i, saveName, m_ptEarL, m_vmDistEarL,
                        "_EarL", "Left Ear", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0.5;
                plotOneSpot(type, plotSave, i, saveName, m_ptEarR, m_vmDistEarR,
                        "_EarR", "Right Ear", threshold, szX, szY, oriX, oriY); 
                        
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptRed, m_vmDistRed,
                        "_Red", "Red", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptCyan, m_vmDistCyan,
                        "_Cyan", "Cyan", threshold, szX, szY, oriX, oriY); 
            }
            if(m_bShowAbdo && m_bShowEye) {
                oriX = 0; oriY = 0.5;
                plotOneSpot(type, plotSave, i, saveName, m_ptRed, m_vmDistRed,
                        "_Red", "Red", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0.5;
                plotOneSpot(type, plotSave, i, saveName, m_ptCyan, m_vmDistCyan,
                        "_Cyan", "Cyan", threshold, szX, szY, oriX, oriY); 
                        
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEyeL, m_vmDistEyeL,
                        "_EyeL", "Left Eye", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEyeR, m_vmDistEyeR,
                        "_EyeR", "Right Eye", threshold, szX, szY, oriX, oriY);                         
            }  
            if(m_bShowEar && m_bShowEye) {
                oriX = 0; oriY = 0.5;
                plotOneSpot(type, plotSave, i, saveName, m_ptEarL, m_vmDistEarL,
                        "_EarL", "Left Ear", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0.5;
                plotOneSpot(type, plotSave, i, saveName, m_ptEarR, m_vmDistEarR,
                        "_EarR", "Right Ear", threshold, szX, szY, oriX, oriY); 
                
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEyeL, m_vmDistEyeL,
                        "_EyeL", "Left Eye", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_ptEyeR, m_vmDistEyeR,
                        "_EyeR", "Right Eye", threshold, szX, szY, oriX, oriY); 
            }             
        }else if(numScatterPlot ==6) {
            szY = 0.33;
            oriX = 0; oriY = 0.66;
            plotOneSpot(type, plotSave, i, saveName, m_ptEarL, m_vmDistEarL,
                    "_EarL", "Left Ear", threshold, szX, szY, oriX, oriY); 
            oriX = 0.5; oriY = 0.66;
            plotOneSpot(type, plotSave, i, saveName, m_ptEarR, m_vmDistEarR,
                    "_EarR", "Right Ear", threshold, szX, szY, oriX, oriY); 
            
            oriX = 0; oriY = 0.33;
            plotOneSpot(type, plotSave, i, saveName, m_ptRed, m_vmDistRed,
                    "_Red", "Red", threshold, szX, szY, oriX, oriY); 
            oriX = 0.5; oriY = 0.33;
            plotOneSpot(type, plotSave, i, saveName, m_ptCyan, m_vmDistCyan,
                    "_Cyan", "Cyan", threshold, szX, szY, oriX, oriY); 
            
            oriX = 0; oriY = 0;
            plotOneSpot(type, plotSave, i, saveName, m_ptEyeL, m_vmDistEyeL,
                    "_EyeL", "Left Eye", threshold, szX, szY, oriX, oriY); 
            oriX = 0.5; oriY = 0;
            plotOneSpot(type, plotSave, i, saveName, m_ptEyeR, m_vmDistEyeR,
                    "_EyeR", "Right Eye", threshold, szX, szY, oriX, oriY); 
            
        }

        plotSave.cmd("unset multiplot");
        
    }
    plotSave.cmd("reset");    
}
void CRat::plotOneSpot(char* type, Gnuplot& plotSave, int i, wxFileName& saveName, Point pt, vector<Mat>& vmPDF,
								char* extName1, char* title1, float threshold,
                                float sizeX, float sizeY, float oriX, float oriY)
{
    Mat& mFlow = m_vecFlow[i];	
    Mat& mPdf = vmPDF[i];
   
    char str[100];
    sprintf(str, "set size %f, %f", sizeX, sizeY);
    plotSave.cmd(str);
    
    sprintf(str, "set origin %f, %f", oriX, oriY);
    plotSave.cmd(str);

    std::ostringstream cmdstr2;
    cmdstr2 << "set title '" << title1 << "'" ;
    plotSave.cmd(cmdstr2.str());

    wxString strOutName = saveName.GetFullPath() + extName1;
	if(strcmp(type, "dots")==0)
		plotDotScatter(plotSave, mFlow, pt, strOutName, mPdf, threshold);
	else
		plotDistribution(plotSave, mPdf, strOutName);
}

void CRat::plotDistribution(Gnuplot& plot, Mat& mDist, wxString& strOutName)
{
	wxString fNameBin = strOutName + ".bin";
	_OutputMatGnuplotBinData(mDist, fNameBin.ToAscii());
	
	std::ostringstream cmdstr;
    cmdstr << "splot '" << fNameBin.ToAscii() << "' binary matrix using 1:2:3";
    plot.cmd(cmdstr.str());	
}

void CRat::plotDotScatter(Gnuplot& plotSavePGN, Mat& mFlow, Point pt, wxString& strOutName, Mat& mPdf, float threshold)
{
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= mFlow.cols) pt2.x = mFlow.cols-1;
	if(pt2.y >= mFlow.rows) pt2.y = mFlow.rows-1;
	
	Mat mROI(mFlow, Rect(pt1, pt2));
	
	vector<Point2f>  vDataUp;
	vector<Point2f>  vDataBelow;
	
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
			Point2f fxy = mROI.at<Point2f>(y, x);
			Point2f fxy1 = fxy + Point2f(PDF_SIZE/2, PDF_SIZE/2);
			
			if(fxy1.x >= PDF_SIZE || fxy1.x <0) {
				vDataBelow.push_back(fxy);
				continue;
			}
			if(fxy1.y >= PDF_SIZE || fxy1.y <0) {
				vDataBelow.push_back(fxy);
				continue;
			}
			
			float probability = mPdf.at<float>(fxy1.y+0.5, fxy1.x+0.5);
			if(probability >= threshold) 	vDataUp.push_back(fxy);
			else vDataBelow.push_back(fxy);
			
		}
		
	
	// save csv and png for dot density
	wxString  fName = strOutName + "_up.csv";
	_OutputVecPoints(vDataUp, fName.ToAscii(), true);
	
	std::ostringstream cmdstr;
    cmdstr << "plot '" << fName.ToAscii() << "' with dots linecolor '#0022FF'";
    plotSavePGN.cmd(cmdstr.str());	
	
	fName = strOutName + "_below.csv";
	_OutputVecPoints(vDataBelow, fName.ToAscii(), true);
	
	//_OutputMatPoint2f(mROI, fName.ToAscii());

	std::ostringstream cmdstr1;
    cmdstr1 << "plot '" << fName.ToAscii() << "' with dots linecolor '#FF0022'";
    plotSavePGN.cmd(cmdstr1.str());	
}


float CRat::optical_compute_movement_v1(Mat& mFlow, Mat& mDistEar, Point pt, float threshold)
{
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= mFlow.cols) pt2.x = mFlow.cols-1;
	if(pt2.y >= mFlow.rows) pt2.y = mFlow.rows-1;
	
	Mat mROI(mFlow, Rect(pt1, pt2));
	float movement = 0;
	float alpha = 1;
    float beta = 2;
	float count = 0;

    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
			Point2f fxy = mROI.at<Point2f>(y, x);
			float mv  = cv::norm(fxy);	
			
			fxy += Point2f(PDF_SIZE/2, PDF_SIZE/2);
			
			if(fxy.x >= PDF_SIZE || fxy.x <0) continue;
			if(fxy.y >= PDF_SIZE || fxy.y <0) continue;
			
			float Pr = mDistEar.at<float>(fxy.y+0.5, fxy.x+0.5);

			if(Pr >= threshold) {
				float w =  1./(1.+exp(-alpha*(mv-beta)));
				movement += mv*w;
				count+=w; 
			}
		}
	if(count!=0)
		movement = (movement /count);
	  	
	return (movement);
}

float CRat::optical_compute_movement_v2(Mat& mFlow, Mat& mDistEar, Point pt, float threshold)
{
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= mFlow.cols) pt2.x = mFlow.cols-1;
	if(pt2.y >= mFlow.rows) pt2.y = mFlow.rows-1;
	
	Mat mROI(mFlow, Rect(pt1, pt2));
	float movement = 0;
	double alpha = 5;
	float count = 0;
	Point2f fxysum(0,0);
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
			Point2f fxy = mROI.at<Point2f>(y, x);
			Point2f fxy0 = fxy;
			//float mv  = cv::norm(fxy);	
			
			fxy += Point2f(PDF_SIZE/2, PDF_SIZE/2);
			
			if(fxy.x >= PDF_SIZE || fxy.x <0) continue;
			if(fxy.y >= PDF_SIZE || fxy.y <0) continue;
			
			float Pr = mDistEar.at<float>(fxy.y+0.5, fxy.x+0.5);

			if(Pr >= threshold) {
				fxysum += fxy0;
				//float w =  1./(1.+exp(-1*(mv-6)));
				//movement += w*mv;
				//count+=w; 
				count++; 
			}
		}
	
	movement  = cv::norm(fxysum);
	if(count!=0)
		movement = (movement /count);
	  	
	return (movement);
}

void CRat::opticalBuildPDF(Mat& mFlow, Mat& mGaus, Mat& mDist, Point pt)
{
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= mFlow.cols) pt2.x = mFlow.cols-1;
	if(pt2.y >= mFlow.rows) pt2.y = mFlow.rows-1;
	
	Mat mROI(mFlow, Rect(pt1, pt2));
	
	int ksize = mGaus.rows;
	int border = ksize /2;
	int  pdfCenter = PDF_SIZE /2;
	mDist = Scalar(0);
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
            const Point2f& fxy = mROI.at<Point2f>(y, x);
            if(fxy.x > pdfCenter-border-1 || fxy.x <-pdfCenter+border+1) continue;
			if(fxy.y > pdfCenter-border-1 || fxy.y <-pdfCenter+border+1) continue;

			Point pt1 (fxy.x+0.5-border+pdfCenter, fxy.y+0.5-border+pdfCenter);
			Point pt2 (fxy.x+0.5+border+pdfCenter+1, fxy.y+0.5+border+pdfCenter+1);

			Mat mask(mDist, Rect(pt1, pt2));
			mask += mGaus;
        }
		
	mDist /= (mROI.rows*mROI.cols);	
}

void CRat::createGaussianMask(double& sigma, int& ksize, Mat& mKernel)
{
	if(sigma <0) 	sigma = 0.3*((ksize-1)*0.5 - 1) + 0.8;
	if(ksize <0) 	ksize = ((sigma - 0.8)/0.3 +1)*2 +1;
	
	int W = ksize;
	double mean = W/2;
	double sigma2_2PI = 2 * M_PI * sigma * sigma;
	mKernel.create(W, W, CV_32FC1);
	
	float sum = 0.0; // For accumulating the kernel values	
	for (int x = 0; x < W; ++x) 
		for (int y = 0; y < W; ++y) {
			mKernel.at<float>(y, x) = exp( -0.5 * (pow((x-mean)/sigma, 2.0) + pow((y-mean)/sigma,2.0)) )/sigma2_2PI;

			// Accumulate the kernel values
			sum += mKernel.at<float>(y, x);
		}

	// Normalize the kernel
	mKernel /= sum;	
}
void CRat::drawOptFlowMap(Mat& cflowmap, const Mat& flow,  int step, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
            //circle(cflowmap, Point(x,y), 1, color, -1);
        }
}

float CRat::findAvgMotion(Mat& mFlowROI)
{
	float sum = 0;
	int w = mFlowROI.cols;
	int h = mFlowROI.rows;
	int sz = 0;
    for(int y = 0; y <h; y++){
        for(int x = 0; x <w; x ++)
        {
            Point2f fxy = mFlowROI.at<Point2f>(y, x);
			float mv  = cv::norm(fxy);
			//if (mv >= 1) {
				sum += mv;
				sz++;
			//}
        }
	}

	if (sz == 0) return 0;
	return sum/ sz;
}


void CRat::saveEarImage()
{
//	int  n = m_vecLEarMotion.size();
	double maxVal = 0;
	int maxIdx = 0;
	bool bLeftBig = true;
//	for(int i=0; i<n; i++) {
//		if(m_vecLEarMotion[i] > maxVal) {
//			maxVal = m_vecLEarMotion[i];
//			maxIdx = i;
//		}
//	}
	
//	n = m_vecREarMotion.size();
//	for(int i=0; i<n; i++) {
//		if(m_vecREarMotion[i] > maxVal) {
//			maxVal = m_vecREarMotion[i];
//			maxIdx = i;
//			bLeftBig = false;
//		}
//	}	

//////////////////////////////////////////	
/*
	int w = m_vecMat[0].cols;
	int h = m_vecMat[0].rows;
	
	TwoPts ptEar = m_vecEarPair[maxIdx];
	Point pt1, pt2;
	if(bLeftBig){
		pt1 = Point(ptEar.ptL-m_offsetEar);
		pt2 = Point(ptEar.ptL+m_offsetEar);
		if(pt1.x <0) pt1.x = 0;
		if(pt1.y <0) pt1.y = 0;
		if(pt2.x >= w) pt2.x = w-1;
		if(pt2.y >= h) pt2.y = h-1;	

	}else{
		pt1 = Point(ptEar.ptL-m_offsetEar);
		pt2 = Point(ptEar.ptL+m_offsetEar);
		if(pt1.x <0) pt1.x = 0;
		if(pt1.y <0) pt1.y = 0;
		if(pt2.x >= w) pt2.x = w-1;
		if(pt2.y >= h) pt2.y = h-1;	
	
	}
	
	//////////////////////////////////////////////////////
	
	Mat mROI(m_vecDest[maxIdx], Rect(pt1, pt2));
	

	wxFileName fileName = wxString(m_vFilenames[maxIdx]);
	wxString  fName = fileName.GetName();
	wxFileName saveName(m_strSrcPath, "_"+fName+ "_ear.bmp");
	imwrite(saveName.GetFullPath().ToStdString(), mROI);
	
	wxString str;
	str.Printf("save name: %s\n", saveName.GetFullPath());
	MainFrame:: myMsgOutput(str);
*/
}

double CRat::lineSquareError(vector<cv::Point>& contour, cv::Point& pt)
{
	int sz = contour.size();
	int len = 150;
	double y[150], x[150];
	double a1, b1, cov00, cov01, cov11, sumsq, mseLeft, mseRight;
	
	int minDist = 999999;
	int tip =0;
	for(int i=0; i<sz; i++) {
		int dist = abs(pt.x-contour[i].x) + abs(pt.y-contour[i].y);
		if(dist < minDist) {
			minDist = dist;
			tip = i;
		}
	}


	for(int k=0; k<len; k++) {
		x[k] = contour[(tip-k+sz)%sz].x;
		y[k] = contour[(tip-k+sz)%sz].y;
	}
	gsl_fit_linear(x, 1, y, 1, len, &a1, &b1, &cov00, &cov01, &cov11, &sumsq);
	mseLeft = sumsq/len;

	for(int k=0; k<len; k++) {
		x[k] = contour[(tip+k+sz)%sz].x;
		y[k] = contour[(tip+k+sz)%sz].y;
	}
	gsl_fit_linear(x, 1, y, 1, len, &a1, &b1, &cov00, &cov01, &cov11, &sumsq);
	mseRight = sumsq/len;

	if(mseRight < mseLeft)
		return mseRight;
	else 
		return mseLeft;
}
void CRat::itkLaplacianOfGaussian(Mat& mSrc, Mat& mDest, double sigma)
{

	typedef itk::OpenCVImageBridge BridgeType;
	charImageType::Pointer itkImage = BridgeType::CVMatToITKImage<charImageType>(mSrc);
	// cast to float
	typedef itk::CastImageFilter<charImageType, floatImageType > CastFilterType;

	CastFilterType::Pointer castFilter = CastFilterType::New();
	castFilter->SetInput(itkImage );

	typedef itk::LaplacianRecursiveGaussianImageFilter<
                        floatImageType, floatImageType >  FilterType;
	FilterType::Pointer laplacian = FilterType::New();
	laplacian->SetNormalizeAcrossScale( false );
	laplacian->SetSigma( sigma );
	laplacian->SetInput( castFilter->GetOutput() );
	laplacian->Update();

	cv::Mat resultImage =BridgeType::ITKImageToCVMat< floatImageType >(laplacian->GetOutput());
//	resultImage.copyTo(mDest);

	//double  min, max;
	//cv::minMaxLoc(mDest, &min, &max);
	//gpMsgbar->ShowMessage("LaplacianOfGaussian, [%f, %f]\n", min, max);
	cv::threshold(resultImage, mDest, 0, 255, THRESH_TOZERO);

}


void CRat::ContourSmooth(vector<cv::Point> & contour, int sigma)
{
	typedef  struct _pointArray{
		float sx, sy;
		cv::Point p;
		bool bRedundant;
	}PointArray;


	int i, j, k, m;
	
	m = contour.size();
	Point2f center = Point2f(0., 0.);
	for(k=0; k<m; k++) {
		center.x += contour[k].x;
		center.y += contour[k].y;
	}
	center.x /= m;
	center.y /= m;


	PointArray * pArray = new PointArray [m];
    for(i=0; i<m; i++) {
        pArray[i].p.x = contour[i].x - center.x;
		pArray[i].p.y = contour[i].y - center.y;
    }

	// generate gaussian mask
	float *mask, sum, coef1, coef2, pi2;    
    int n1, n2, maskWidth;


    n2 = 5 * sigma;
 
	pi2 = 3.1415926 * 2;
    mask = new float[n2+1];
    coef1 = 1.0 / (sqrt(pi2) * sigma);
    coef2 = -0.2 / (sigma * sigma);
    sum = coef1;
    mask[0] = coef1;
    for(n1=1; (sum<=0.9999)&&(n1<=n2); n1++)
    {
       mask[n1] = coef1 * exp(coef2 * n1 * n1);
       sum += 2 * mask[n1];
    }
    maskWidth = n1; 
 
	//   smooth contour
	for(i=0; i<m; i++)
	{
		pArray[i].sx = pArray[i].p.x * mask[0];
		pArray[i].sy = pArray[i].p.y * mask[0];
		for(j=1; j<maskWidth; j++) {
			pArray[i].sx += (pArray[(i+j)%m].p.x + pArray[(i-j+m)%m].p.x) * mask[j];
			pArray[i].sy += (pArray[(i+j)%m].p.y + pArray[(i-j+m)%m].p.y) * mask[j];
		}
	}
	for(i=0; i<m; i++) {
		pArray[i].p.x = (int)floor(pArray[i].sx + 0.5 + center.x);
		pArray[i].p.y = (int)floor(pArray[i].sy + 0.5 + center.y);
	}


	// mark redundant points
	for(i=0; i<m; i++) {
      pArray[i].bRedundant = FALSE; 
      for(j=i+1; j<m; j++) {
          if(pArray[i].p == pArray[j].p ) 
            pArray[j].bRedundant = TRUE;        
         else {
            i = j-1;
            break;
         }
      }
    }
    k = 0;
    for(i=0; i<m; i++) {
        if(!pArray[i].bRedundant) {
            pArray[k].p = pArray[i].p; 
            k++;
        }
    }
    contour.clear();
 
    m = k;
 
    for(i=0; i<m; i++) {
        BresLine(pArray[i].p.x, pArray[i].p.y, pArray[(i+1)%m].p.x, pArray[(i+1)%m].p.y, contour);
    }
    delete [] pArray;
 
    m = contour.size(); 
 
    // eliminate Redundant points
    pArray = new PointArray[m];
    for(i=0; i<m; i++) {
        pArray[i].p = contour[i];
        pArray[i].bRedundant = FALSE;
    }
 
    for(i=0; i<m-1; i++) {
        if(pArray[i].p == pArray[i+1].p)
            pArray[i+1].bRedundant = TRUE;
    }
 
    contour.clear();
    for(i=0; i<m; i++) {
        if(!pArray[i].bRedundant) {
			contour.push_back(pArray[i].p); 
        }
    }
 
   delete [] pArray;
   delete [] mask;
 
}
 
 
 
void CRat::BresLine(int Ax, int Ay, int Bx, int By, vector<cv::Point> & contour)
{
    //------------------------------------------------------------------------
    // INITIALIZE THE COMPONENTS OF THE ALGORITHM THAT ARE NOT AFFECTED BY THE
    // SLOPE OR DIRECTION OF THE LINE
    //------------------------------------------------------------------------
    int dX = abs(Bx-Ax);    // store the change in X and Y of the line endpoints
    int dY = abs(By-Ay);
    
    //------------------------------------------------------------------------
    // DETERMINE "DIRECTIONS" TO INCREMENT X AND Y (REGARDLESS OF DECISION)
    //------------------------------------------------------------------------
    int Xincr, Yincr;
    if (Ax > Bx) { Xincr=-1; } else { Xincr=1; }    // which direction in X?
    if (Ay > By) { Yincr=-1; } else { Yincr=1; }    // which direction in Y?
    
    //------------------------------------------------------------------------
    // DETERMINE INDEPENDENT VARIABLE (ONE THAT ALWAYS INCREMENTS BY 1 (OR -1) )
    // AND INITIATE APPROPRIATE LINE DRAWING ROUTINE (BASED ON FIRST OCTANT
    // ALWAYS). THE X AND Y'S MAY BE FLIPPED IF Y IS THE INDEPENDENT VARIABLE.
    //------------------------------------------------------------------------
    if (dX >= dY)    // if X is the independent variable
    {           
        int dPr     = dY<<1;           // amount to increment decision if right is chosen (always)
        int dPru     = dPr - (dX<<1);   // amount to increment decision if up is chosen
        int P         = dPr - dX;  // decision variable start value
 
        for (; dX>=0; dX--)            // process each point in the line one at a time (just use dX)
        {
			contour.push_back( Point(Ax, Ay)); // plot the pixel
//            fprintf(m_pFile, "%3d, %3d\n", Ax, Ay);
 
            if (P > 0)               // is the pixel going right AND up?
            { 
                Ax+=Xincr;           // increment independent variable
                Ay+=Yincr;         // increment dependent variable
                P+=dPru;           // increment decision (for up)
            }
            else                     // is the pixel just going right?
            {
                Ax+=Xincr;         // increment independent variable
                P+=dPr;            // increment decision (for right)
            }
        }        
    }
    else              // if Y is the independent variable
    {
        int dPr     = dX<<1;           // amount to increment decision if right is chosen (always)
        int dPru     = dPr - (dY<<1);   // amount to increment decision if up is chosen
        int P         = dPr - dY;  // decision variable start value
 
        for (; dY>=0; dY--)            // process each point in the line one at a time (just use dY)
        {
            contour.push_back( Point(Ax, Ay)); // plot the pixel
//            fprintf(m_pFile, "%3d, %3d\n", Ax, Ay);
 
            if (P > 0)               // is the pixel going up AND right?
            { 
                Ax+=Xincr;         // increment dependent variable
                Ay+=Yincr;         // increment independent variable
                P+=dPru;           // increment decision (for up)
            }
            else                     // is the pixel just going up?
            {
                Ay+=Yincr;         // increment independent variable
                P+=dPr;            // increment decision (for right)
            }
        }        
    }        
}
