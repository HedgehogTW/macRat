
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

using namespace cv;

Gnuplot gPlotL("lines");
Gnuplot gPlotR("lines");

CRat::CRat()
{
	m_offsetEar = Point(EAR_RECT, EAR_RECT);
	m_offsetEye = Point(10, 10);

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
	m_bCropped = false;
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

	MainFrame:: myMsgOutput("1-LED frames [%d..%d], ----> 2-LED frame %d\n", 
		m_nLED1+1, m_nLED_End+1, m_nLED2+1); 

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

bool CRat::prepareData()
{
	vector <Mat> vecData;
	vector <string> vFilenames;
	int w = m_vecMat[0].cols;
	int h = m_vecMat[0].rows;
	
	if(h <=m_nCageLine) return m_bCropped;
	if(m_nCageLine >0) {
		for(int i=0; i<m_nSlices; i++)  {
			Mat mSrc;
			if(m_nCageLine > h/2)
				mSrc = m_vecMat[i].rowRange(0, m_nCageLine);
			else
				mSrc = m_vecMat[i].rowRange(m_nCageLine, h);
				
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
		
		m_bCropped = true;
	}

	m_szImg = Size(m_vecMat[0].cols, m_vecMat[0].rows );
	m_nSlices = m_vecMat.size();

	saveResult("cage", m_vecMat);
	
	return m_bCropped;
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
	for(int i=0; i<nFirstLED; i++)  vecSignal[i] -= mean;	
}

bool CRat::processAbdomen(Point ptAbdoBorder, Point ptAbdoIn)
{
	m_ptAbdoBo = ptAbdoBorder;
	m_ptAbdoIn = ptAbdoIn;
	
	m_referFrame = 60;//findReferenceFrame(ptEar);
	
	MainFrame:: myMsgOutput("Reference frame %d\n", m_referFrame);
	if(m_referFrame <0) {
		wxLogMessage("cannot find reference frame");
		return false;
	}	
	
	vector <float>  vecAbdoBoGrayDiff;
	vector <float>  vecAbdoInGrayDiff;
	graylevelDiff(m_referFrame, ptAbdoBorder, ptAbdoIn, vecAbdoBoGrayDiff, vecAbdoInGrayDiff);
	
	DC_removal(m_nLED1, vecAbdoBoGrayDiff);
	DC_removal(m_nLED1, vecAbdoInGrayDiff);
	
	/////////////////////////////////////////////////////////////optical flow
	int  newFrameSteps;
	static int frameStep = 2;	
	static double threshold = 0.001;
	static bool bLED = false;
	static bool bPinna = false;
	static bool bVerLine = false;
	
	static bool bEyeMove = false;
	static bool bGrayDiff = false;
	static bool bOptical = true;
	static bool bOpticalPDF = true;
	static bool bAccumulate = true;
	
	static double verLine = 190;
	
	static int  ymin, ymax;
	if(bAccumulate ==false) {
		ymin = -2;
		ymax = 2;	
	}else if(frameStep==0) {
		ymin = -2;
		ymax = 25;
	}else {
		ymin = -20;
		ymax = 120;
	}
	
	DlgOpticalInput dlg(frameStep, threshold, MainFrame::m_pThis);
	dlg.setVerticalLine(bLED, bPinna, bVerLine, verLine);
	dlg.setSeriesLine(bEyeMove, bGrayDiff, bOptical, bOpticalPDF, bAccumulate);
	dlg.setYRange(ymin, ymax);
	
	if(dlg.ShowModal() !=  wxID_OK) return false;
	newFrameSteps = dlg.getFrameSteps();
	threshold = dlg.getThreshold();
	dlg.getVerticalLine(bLED, bPinna, bVerLine, verLine);
	dlg.getSeriesLine(bEyeMove, bGrayDiff, bOptical, bOpticalPDF, bAccumulate);
	dlg.getYRange(ymin, ymax);
	dlg.Destroy();
	
	clock_t start, finish;
	double  duration;
	start = clock();
	
	wxBeginBusyCursor();
	vector <float>  vecLEarFlow;  // border
	vector <float>  vecREarFlow;  // in
	vector <float>  vecLEarFlowPdf;
	vector <float>  vecREarFlowPdf;
	vector <float>  vecLEarFlowPdfRegres;
	vector <float>  vecREarFlowPdfRegres;
	vector <float>  vecLEarFlowPdfSubRegres;
	vector <float>  vecREarFlowPdfSubRegres;
	
	int szVecFlow = m_vecFlow.size();
	if(newFrameSteps != frameStep || szVecFlow ==0 ) {
		opticalFlow(newFrameSteps);
	}
	frameStep = newFrameSteps;
	
	MainFrame:: myMsgOutput("PDF threshold %f, frame steps %d, bAccumulate %d, y range [%d, %d]\n", 
			threshold, frameStep, bAccumulate, ymin, ymax);
	
	opticalDrawFlowmap(ptAbdoBorder, ptAbdoIn, frameStep, 'A');
	
	if(bOpticalPDF) {
		opticalFlowDistribution(m_vecFlow, "pdf", vecLEarFlowPdf, vecREarFlowPdf, ptAbdoBorder, ptAbdoIn, 
									"_Border", "_In", "Border", "Inside", threshold);
		opticalSaveScatterPlot(m_vecFlow, "scatter", ptAbdoBorder, ptAbdoIn, "_Border", "_In", "Border", "Inside", threshold, "pdf");
		
		int sz = vecLEarFlowPdf.size();
		if(frameStep > 0 && bAccumulate) {
			for(int i=+1; i<sz; i++) {
				vecLEarFlowPdf[i] += vecLEarFlowPdf[i-1];
				vecREarFlowPdf[i] += vecREarFlowPdf[i-1];
			}
		}
		DC_removal(m_nLED1, vecLEarFlowPdf);
		DC_removal(m_nLED1, vecREarFlowPdf);				
		
		if(bAccumulate) {
			linearRegression(vecLEarFlowPdf, vecLEarFlowPdfRegres, vecLEarFlowPdfSubRegres);
			linearRegression(vecREarFlowPdf, vecREarFlowPdfRegres, vecREarFlowPdfSubRegres);
		}
	}
	 
	if(bOptical) {
		opticalFlowAnalysis(m_vecFlow, ptAbdoBorder, vecLEarFlow);
		opticalFlowAnalysis(m_vecFlow, ptAbdoIn, vecREarFlow);	
		
		int sz = vecLEarFlow.size();
		if(frameStep > 0 && bAccumulate) {	
			for(int i=+1; i<sz; i++) {
				vecLEarFlow[i] += vecLEarFlow[i-1];
				vecREarFlow[i] += vecREarFlow[i-1];
			}
		}
		DC_removal(m_nLED1, vecLEarFlow);
		DC_removal(m_nLED1, vecREarFlow);	
	}
	wxEndBusyCursor(); 
	
///////////G N U P L O T//////////////////////////////////////////////////////////////////////////////


	wxFileName fileName = m_strSrcPath;
	const char* title =fileName.GetName();
	_gnuplotInit(gPlotL, title, ymin, ymax);
	_gnuplotInit(gPlotR, title, ymin, ymax);
	gPlotL.set_legend("left");
	gPlotR.set_legend("left");	
/*	
	if(bLED) {
		_gnuplotLED(gPlotL, m_nLED1, m_nLED2);
		_gnuplotLED(gPlotR, m_nLED1, m_nLED2);
	}

	if(bPinna) {
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
*/	
	if(bVerLine && verLine >0) {
		_gnuplotVerticalLine(gPlotL, verLine);
		_gnuplotVerticalLine(gPlotR, verLine);		
	}
/*	
	if(bEyeMove) {
		_gnuplotLine(gPlotL, "LEyeMove", m_vecEyeLMove, "#008B0000", ".");
		_gnuplotLine(gPlotR, "REyeMove", m_vecEyeRMove, "#008B0000", ".");
	}
	 */ 
	if(bGrayDiff) {
		_gnuplotLine(gPlotL, "Border GraylevelDiff", vecAbdoBoGrayDiff, "#00008000");
		_gnuplotLine(gPlotR, "Inside GraylevelDiff", vecAbdoInGrayDiff, "#00008000");
	}
	if(bOptical) {
		_gnuplotLine(gPlotL, "Border Flow", vecLEarFlow, "#00FF4500");
		_gnuplotLine(gPlotR, "Inside Flow", vecREarFlow, "#00FF4500");
	}
	if(bOpticalPDF) {
		_gnuplotLine(gPlotL, "Border FlowPDF", vecLEarFlowPdf, "#000000FF");
		_gnuplotLine(gPlotR, "Inside FlowPDF", vecREarFlowPdf, "#000000FF");
		
		if(bAccumulate ) {
			_gnuplotLine(gPlotL, "Border PDFRegress", vecLEarFlowPdfRegres, "#000000FF", "-");
			_gnuplotLine(gPlotR, "Inside PDFRegress", vecREarFlowPdfRegres, "#000000FF", "-");
			
			_gnuplotLine(gPlotL, "Border PDF-Regress", vecLEarFlowPdfSubRegres, "#00FF0000");
			_gnuplotLine(gPlotR, "Inside PDF-Regress", vecREarFlowPdfSubRegres, "#00FF0000");
		}
	}
	
	//gPlotL.cmd("load '../_splot_move.gpt'");
	////////////////////// save result to dest
	m_vecDest.resize(m_nSlices);
	Point ptL1 (ptAbdoBorder-m_offsetEar);
	Point ptL2 (ptAbdoBorder+m_offsetEar);
	Point ptR1 (ptAbdoIn-m_offsetEar);
	Point ptR2 (ptAbdoIn+m_offsetEar);
	
	
	for (int i = 0; i < m_nSlices; i++)
		cvtColor(m_vecMat[i], m_vecDest[i], CV_GRAY2BGR);

	for (int i = 0; i < m_nSlices; i++)
	{
		Mat mDestColor;
		mDestColor = m_vecDest[i];
		// original ears
		rectangle(mDestColor, Rect(ptL1, ptL2), Scalar(255,0,0));
		rectangle(mDestColor, Rect(ptR1, ptR2), Scalar(255,0,0));
	}
	saveResult("dest", m_vecDest);

	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	int minutes = duration / 60;
	int second = duration - minutes * 60;
	MainFrame::myMsgOutput("processAbdomen: computation time: %02dm:%02ds\n", minutes, second);
	
	return true;
}

bool CRat::processEar(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR)
{
	m_ptEyeL = ptEyeL;
	m_ptEyeR = ptEyeR;
	m_ptEarL = ptEarL;
	m_ptEarR = ptEarR;
	
	m_referFrame = findReferenceFrame(ptEarL);
	
	MainFrame:: myMsgOutput("Reference frame %d\n", m_referFrame);
	if(m_referFrame <0) {
		wxLogMessage("cannot find reference frame");
		return false;
	}
	findEyeCenter(ptEyeL, m_vecEyeL, m_vecEyeLMove);
	findEyeCenter(ptEyeR, m_vecEyeR, m_vecEyeRMove);
	
	vector <Point>  vecEarL;
	vector <Point>  vecEarR;
	findNewEarCenter(m_vecEyeL, ptEarL, vecEarL);
	findNewEarCenter(m_vecEyeR, ptEarR, vecEarR);
	
//	saveEyeTrajectory();
//	_OutputVecPoints(m_vecEyeL, "_eyeLeft.txt");
	
	vector <float>  vecLEarGrayDiff;
	vector <float>  vecREarGrayDiff;
	graylevelDiff(m_referFrame, ptEarL, ptEarR, vecLEarGrayDiff, vecREarGrayDiff);

	vector<float> smoothL;
	smoothData(vecLEarGrayDiff, smoothL, 2);	
	
	vector<float> smoothR;
	smoothData(vecREarGrayDiff, smoothR, 2);	
		
	int maxPointL = findMaxMotionPoint(smoothL);
	int maxPointR = findMaxMotionPoint(smoothR);	

	DC_removal(m_nLED1, vecLEarGrayDiff);
	DC_removal(m_nLED1, vecREarGrayDiff);
	
	/////////////////////////////////////////////////////////////optical flow
	int  newFrameSteps;
	static int frameStep = 2;	
	static double threshold = 0.001;
	static bool bLED = false;
	static bool bPinna = false;
	static bool bVerLine = false;
	
	static bool bEyeMove = false;
	static bool bGrayDiff = false;
	static bool bOptical = true;
	static bool bOpticalPDF = true;
	static bool bAccumulate = true;
	
	static double verLine = 190;
	
	static int  ymin, ymax;
	if(bAccumulate ==false) {
		ymin = -2;
		ymax = 2;	
	}else if(frameStep==0) {
		ymin = -2;
		ymax = 25;
	}else {
		ymin = -20;
		ymax = 120;
	}
	
	DlgOpticalInput dlg(frameStep, threshold, MainFrame::m_pThis);
	dlg.setVerticalLine(bLED, bPinna, bVerLine, verLine);
	dlg.setSeriesLine(bEyeMove, bGrayDiff, bOptical, bOpticalPDF, bAccumulate);
	dlg.setYRange(ymin, ymax);
	
	if(dlg.ShowModal() !=  wxID_OK) return false;
	newFrameSteps = dlg.getFrameSteps();
	threshold = dlg.getThreshold();
	dlg.getVerticalLine(bLED, bPinna, bVerLine, verLine);
	dlg.getSeriesLine(bEyeMove, bGrayDiff, bOptical, bOpticalPDF, bAccumulate);
	dlg.getYRange(ymin, ymax);
	dlg.Destroy();
	
	clock_t start, finish;
	double  duration;
	start = clock();
	
	wxBeginBusyCursor();
	vector <float>  vecLEarFlow;
	vector <float>  vecREarFlow;
	
	vector <float>  vecLEarFlowPdf;
	vector <float>  vecREarFlowPdf;
	vector <float>  vecLEarFlowPdfRegres;
	vector <float>  vecREarFlowPdfRegres;
	vector <float>  vecLEarFlowPdfSubRegres;
	vector <float>  vecREarFlowPdfSubRegres;
	
	int szVecFlow = m_vecFlow.size();
	if(newFrameSteps != frameStep || szVecFlow ==0 ) {
		opticalFlow(newFrameSteps);
	}
	frameStep = newFrameSteps;
	
	MainFrame:: myMsgOutput("PDF threshold %f, frame steps %d, bAccumulate %d, y range [%d, %d]\n", 
			threshold, frameStep, bAccumulate, ymin, ymax);
	
	opticalDrawFlowmap(m_ptEarL, m_ptEarR, frameStep, 'E');
	
	if(bOpticalPDF) {
		opticalFlowDistribution(m_vecFlow, "pdf", vecLEarFlowPdf, vecREarFlowPdf, m_ptEarL, m_ptEarR, 
									"_EarL", "_EarR", "Left Ear", "Right Ear", threshold);
		opticalSaveScatterPlot(m_vecFlow, "scatter", m_ptEarL, m_ptEarR, "_EarL", "_EarR", "Left Ear", "Right Ear", threshold, "pdf");
		
		int sz = vecLEarFlowPdf.size();
		if(frameStep > 0 && bAccumulate) {
			for(int i=+1; i<sz; i++) {
				vecLEarFlowPdf[i] += vecLEarFlowPdf[i-1];
				vecREarFlowPdf[i] += vecREarFlowPdf[i-1];
			}
		}
		DC_removal(m_nLED1, vecLEarFlowPdf);
		DC_removal(m_nLED1, vecREarFlowPdf);				
		
		if(bAccumulate) {
			linearRegression(vecLEarFlowPdf, vecLEarFlowPdfRegres, vecLEarFlowPdfSubRegres);
			linearRegression(vecREarFlowPdf, vecREarFlowPdfRegres, vecREarFlowPdfSubRegres);
		}
	}
	 
	if(bOptical) {
		opticalFlowAnalysis(m_vecFlow, ptEarL, vecLEarFlow);
		opticalFlowAnalysis(m_vecFlow, ptEarR, vecREarFlow);	
		
		int sz = vecLEarFlow.size();
		if(frameStep > 0 && bAccumulate) {	
			for(int i=+1; i<sz; i++) {
				vecLEarFlow[i] += vecLEarFlow[i-1];
				vecREarFlow[i] += vecREarFlow[i-1];
			}
		}
		DC_removal(m_nLED1, vecLEarFlow);
		DC_removal(m_nLED1, vecREarFlow);	
	}
	wxEndBusyCursor(); 
	
///////////G N U P L O T//////////////////////////////////////////////////////////////////////////////


	wxFileName fileName = m_strSrcPath;
	const char* title =fileName.GetName();
	_gnuplotInit(gPlotL, title, ymin, ymax);
	_gnuplotInit(gPlotR, title, ymin, ymax);
	gPlotL.set_legend("left");
	gPlotR.set_legend("left");		
	if(bLED) {
		_gnuplotLED(gPlotL, m_nLED1, m_nLED2);
		_gnuplotLED(gPlotR, m_nLED1, m_nLED2);
	}

	if(bPinna) {
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
		_gnuplotLine(gPlotL, "LEyeMove", m_vecEyeLMove, "#008B0000", ".");
		_gnuplotLine(gPlotR, "REyeMove", m_vecEyeRMove, "#008B0000", ".");
	}
	if(bGrayDiff) {
		_gnuplotLine(gPlotL, "LEarGraylevelDiff", vecLEarGrayDiff, "#00008000");
		_gnuplotLine(gPlotR, "REarGraylevelDiff", vecREarGrayDiff, "#00008000");
	}
	if(bOptical) {
		_gnuplotLine(gPlotL, "LEarFlow", vecLEarFlow, "#00FF4500");
		_gnuplotLine(gPlotR, "REarFlow", vecREarFlow, "#00FF4500");
	}
	if(bOpticalPDF) {
		_gnuplotLine(gPlotL, "LEarPDF", vecLEarFlowPdf, "#000000FF");
		_gnuplotLine(gPlotR, "REarPDF", vecREarFlowPdf, "#000000FF");
		
		if(bAccumulate ) {
			_gnuplotLine(gPlotL, "LEarPDFRegress", vecLEarFlowPdfRegres, "#000000FF", "-");
			_gnuplotLine(gPlotR, "REarPDFRegress", vecREarFlowPdfRegres, "#000000FF", "-");
			
			_gnuplotLine(gPlotL, "LEarPDF-Regress", vecLEarFlowPdfSubRegres, "#00FF0000");
			_gnuplotLine(gPlotR, "REarPDF-Regress", vecREarFlowPdfSubRegres, "#00FF0000");
		}
	}
	
	//gPlotL.cmd("load '../_splot_move.gpt'");
	////////////////////// save result to dest
	m_vecDest.resize(m_nSlices);
	Point ptL1 (ptEarL-m_offsetEar);
	Point ptL2 (ptEarL+m_offsetEar);
	Point ptR1 (ptEarR-m_offsetEar);
	Point ptR2 (ptEarR+m_offsetEar);
	
	
	for (int i = 0; i < m_nSlices; i++)
		cvtColor(m_vecMat[i], m_vecDest[i], CV_GRAY2BGR);

	for (int i = 0; i < m_nSlices; i++)
	{
		Mat mDestColor;
		mDestColor = m_vecDest[i];
		// original ears
		rectangle(mDestColor, Rect(ptL1, ptL2), Scalar(255,0,0));
		rectangle(mDestColor, Rect(ptR1, ptR2), Scalar(255,0,0));
		
		// new positions of eyes 
		circle(mDestColor, Point(m_vecEyeL[i].x, m_vecEyeL[i].y), 3, Scalar(0, 0, 255), -1);	
		circle(mDestColor, Point(m_vecEyeR[i].x, m_vecEyeR[i].y), 3, Scalar(0, 0, 255), -1);	
		
		// original eyes
		circle(mDestColor, Point(ptEyeL.x, ptEyeL.y), 2, Scalar(0, 255, 255), -1);	
		circle(mDestColor, Point(ptEyeR.x, ptEyeR.y), 2, Scalar(0, 255, 255), -1);	
		
		// new positions of ears 
		Point ptL1n (vecEarL[i]-m_offsetEar);
		Point ptL2n (vecEarL[i]+m_offsetEar);
		Point ptR1n (vecEarR[i]-m_offsetEar);
		Point ptR2n (vecEarR[i]+m_offsetEar);	
		rectangle(mDestColor, Rect(ptL1n, ptL2n), Scalar(0,255,0));
		rectangle(mDestColor, Rect(ptR1n, ptR2n), Scalar(0,255,0));
	}
	saveResult("dest", m_vecDest);

	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	int minutes = duration / 60;
	int second = duration - minutes * 60;
	MainFrame::myMsgOutput("OnRatProcessEar: computation time: %02dm:%02ds\n", minutes, second);
	
	return true;
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
void  CRat::findNewEarCenter(vector <Point>& vecEye, Point ptEar0, vector <Point>& vecEar)
{
	vecEar.clear();	
	vecEar.resize(m_nSlices);	
	
	Point ptReferEye = vecEye[m_referFrame];
	for (int i = 0; i < m_nSlices; i++)	{
		Point ptOffset = vecEye[i] - ptReferEye;
		vecEar[i] = ptEar0 + ptOffset;	
	}
}
void CRat::findEyeCenter(Point& ptEye0, vector <Point>& vecEye, vector <float>&  vecEyeMove)
{
	
	vecEye.clear();	
	vecEye.resize(m_nSlices);

	vecEyeMove.clear();	
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
		minMaxLoc(mROIsm, &min, NULL, &minLoc);
		minLoc = minLoc + Point(10, 10) + pt1;
	//MainFrame:: myMsgOutput("eye center [%d, %d]= %f, ori eye [%d, %d]\n", minLoc.x, minLoc.y, min, ptEye.x, ptEye.y);	
	
		ptEye = minLoc;
		
		vecEye[i] = ptEye;
		
	}
	
	Point eyeRef = vecEye[m_referFrame];
	
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
	end = m_nLED2 + 10;
	lenSeg = 10;
	
	vector <double>  vSeries;
	
	Mat mSrc1, mSrc2, mDiff;

	for(int i=0; i<end; i++) {
		mSrc1 = m_vecMat[i]; //17
		double errSumL, errSumR;
		mSrc2 = m_vecMat[i+2];
		cv::absdiff(mSrc1, mSrc2, mDiff);
		
		errSumL = errorSum(mDiff, pt);
//		errSumR = errorSum(mDiff, ptEarR);
		vSeries.push_back(errSumL);
//		m_vecREarGrayDiff.push_back(errSumR);
	}
//	_gnuplotLine("LeftEar", vSeries);	
	
	double *seg, *x;
	double a1, b1, cov00, cov01, cov11, sumsq;

	seg = new double[lenSeg]; 
	x = new double[lenSeg]; 

	double min = 9999;
	int  idxMin= -1;
//FILE* fp= fopen("mse.csv", "w");
	for(int k=start ; k<= end-lenSeg+1; k++) {
		for(int i=0; i<lenSeg; i++) {
			seg[i] = vSeries[k+i];
			x[i] = i;
		}
		
		gsl_fit_linear(x, 1, seg, 1, lenSeg, &a1, &b1, &cov00, &cov01, &cov11, &sumsq);
		if(sumsq < min){
			min = sumsq;
			idxMin = k;
		}
//		fprintf(fp, "%d, %.3f\n", k, sumsq);
	}
//fclose(fp);

	delete [] seg;
	delete [] x;	
	
	return idxMin +2;
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
	
	Mat mSrc1 = m_vecMat[refer]; //17

	Point pt1 (ptEar-m_offsetEar);
	Point pt2 (ptEar+m_offsetEar);
	
	if(pt1.x <0 || pt1.y <0 || pt2.x >= wImg || pt2.y >= hImg) {
		wxLogMessage("ear region outside");
		return;
	}
		
	Mat mROIRefer(mSrc1, Rect(pt1, pt2));
	
	Point ptReferEye = vecEye[refer];
	
	vecEarGrayDiff.resize(m_nSlices);	
	for(int i=0; i<m_nSlices; i++) {
		Mat mSrc2, mDiff;
		mSrc2 = m_vecMat[i];
		
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

void CRat::graylevelDiff(int refer, Point& ptEarL, Point& ptEarR, vector <float>& vLEarGray,  vector <float>& vREarGray)
{
	vLEarGray.resize(m_nSlices);
	vREarGray.resize(m_nSlices);
	
	Mat mSrc1, mSrc2, mDiff;
	
	mSrc1 = m_vecMat[refer]; //17
	
	for(int i=0; i<m_nSlices; i++) {
		double errSumL, errSumR;
		mSrc2 = m_vecMat[i];
		cv::absdiff(mSrc1, mSrc2, mDiff);
		
		errSumL = errorSum(mDiff, ptEarL);
		errSumR = errorSum(mDiff, ptEarR);
		vLEarGray[i] = errSumL;
		vREarGray[i] = errSumR;
	}
}

void CRat::opticalFlow(int nFrameSteps)
{
	// optical flow
	double pyr_scale = 0.5;
	int nIter = 3;
	int levels= 1;
	int nWinSize = 3;
	int poly_n = 7;
	double dblSigma = 1.5;

//	gpMainFrame->CreateProgressBar(0, m_nSlices, 1);
//	CProgressCtrl *pb =  gpMainFrame->GetProgressBarCtrl();

	m_vecFlow.resize(m_nSlices  - nFrameSteps);
	
	clock_t start, finish;
	double  duration;
	start = clock();
	
//	Point ptEyeC;
//	ptEyeC.x= (m_vecEyeL[m_referFrame].x+m_vecEyeR[m_referFrame].x)/2;
//	ptEyeC.y= (m_vecEyeL[m_referFrame].y+m_vecEyeR[m_referFrame].y)/2;
	
//#pragma omp parallel for 
	for(int i=0; i<m_nSlices - nFrameSteps; i++) {
		int referFrame;
		if(nFrameSteps==0)  referFrame= m_referFrame;
		else referFrame = i;
		
		Mat mSrc1, mSrc2;
		mSrc1 = m_vecMat[referFrame];
		mSrc2 = m_vecMat[i+nFrameSteps];

		Mat& mFlow = m_vecFlow[i];

        calcOpticalFlowFarneback(mSrc1, mSrc2, mFlow, pyr_scale, levels, nWinSize, nIter, poly_n, dblSigma, 0);
	}
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	int minutes = duration / 60;
	int second = duration - minutes * 60;
	MainFrame:: myMsgOutput("Opticalflow computation time: %02dm:%02ds\n", minutes, second);
	MainFrame:: myMsgOutput("Opticalflow done, m_vecFlow size %d------\n", m_vecFlow.size());
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
void CRat::opticalSaveScatterPlot(vector<Mat>& vecFlow, char* subpath, Point pt1, Point pt2,
									char* extName1, char* extName2, char* title1, char* title2, float threshold, char* pdfPath)
{
	//char subpath[] = "movement";
	cv::Mat	 cvMat;
	cv::Mat	 cvMatGray;
	wxString outpath; 

#if defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__) 
	outpath = m_strSrcPath + "/" + subpath;
#else
	outpath = m_strSrcPath + "\\" + subpath;
#endif	

	if(vecFlow.size() ==0) return;

	if(wxDirExists(outpath)==false) {
		if(wxMkdir(outpath)==false) {
			wxString str;
			str.Printf("Create output directory error, %s", outpath);
			wxLogMessage(str);
			return;
		}
	}
	
	bool  bHasEyeData;
	if(m_vecEyeL.size() >0)   bHasEyeData = true;
	else bHasEyeData = false;
	
	Gnuplot plotSavePGN("dots");
#if defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__) 
	if(bHasEyeData)
		plotSavePGN.cmd("set terminal png size 800, 800");
	else
		plotSavePGN.cmd("set terminal png size 800, 400");
#else
	if(bHasEyeData)
		plotSavePGN.cmd("set terminal pngcairo size 800, 800");
	else
		plotSavePGN.cmd("set terminal pngcairo size 800, 400");
#endif	
	plotSavePGN.cmd("set grid");
	plotSavePGN.cmd("unset key");	
	plotSavePGN.set_xrange(-20,20);
	plotSavePGN.set_yrange(-20,20);
	plotSavePGN.cmd("set size 1,1");
	plotSavePGN.cmd("set origin 0,0");
	plotSavePGN.cmd("set termoption noenhanced");
	
	wxFileName fileName = wxString(m_vFilenames[0]);
	int dirCount = fileName.GetDirCount();
	wxArrayString dirs = fileName.GetDirs();
	wxString dirName = dirs[dirCount-1];

	Point ptEyeC;
	if(m_vecEyeL.size() >0){
		ptEyeC.x= (m_vecEyeL[m_referFrame].x+m_vecEyeR[m_referFrame].x)/2;
		ptEyeC.y= (m_vecEyeL[m_referFrame].y+m_vecEyeR[m_referFrame].y)/2;
	}
	Mat mPdf(EAR_RECT, EAR_RECT, CV_32FC1);	
	FILE* fpPdf;
	float  probability;
	int sz = vecFlow.size();
	for(int i=0; i<sz; i++) {
		Mat& mFlow = vecFlow[i];	

		
		// assign save filename
		wxFileName fileName = wxString(m_vFilenames[i]);
		fileName.AppendDir(subpath);
		wxFileName saveName(fileName.GetPath(), fileName.GetName());
		
		// assign pdf filename
		fileName = wxString(m_vFilenames[i]);
		fileName.AppendDir(pdfPath);
		//wxString savePath = fileName.GetPath();
		wxFileName pdfName(fileName.GetPath(), fileName.GetName());
		
		//MainFrame:: myMsgOutput(pdfFilename);
		
		/////////////////////////////////////////////// plot command
		std::ostringstream cmdstr0;
		cmdstr0 << "set output '" << saveName.GetFullPath().ToAscii() << ".png'";
		plotSavePGN.cmd(cmdstr0.str());
	
		std::ostringstream cmdstr1;
		cmdstr1 << "set multiplot title '" << dirName.ToAscii() << "/" << fileName.GetName().ToAscii()  ;
		cmdstr1 << ", th= " << threshold << "'";
		// analyze blocks of ear and eye
		plotSavePGN.cmd(cmdstr1.str());
		if(bHasEyeData){
			plotSavePGN.cmd("set size 0.5, 0.5");
			plotSavePGN.cmd("set origin 0.0, 0.5");
		}else {
			plotSavePGN.cmd("set size 0.5, 1");
			plotSavePGN.cmd("set origin 0.0, 0");		
		}	
		std::ostringstream cmdstr2;
		cmdstr2 << "set title '" << title1 << "'" ;
		plotSavePGN.cmd(cmdstr2.str());
	
	
		////////////////////////////////////////// read pdf matrix
		wxString pdfFilename = pdfName.GetFullPath()+ extName1 + ".txt";
		fpPdf = fopen(pdfFilename.ToAscii(), "r");
		if(fpPdf ==NULL) {
			wxString str = pdfFilename + ": read error";
			wxLogMessage(str);
			continue;
		}
		for(int i=0; i<EAR_RECT; i++)
			for(int c=0; c<EAR_RECT; c++) {
				fscanf(fpPdf, "%f", &probability);
				mPdf.at<float>(i, c) = probability;
			}
		fclose(fpPdf);
	
		wxString strOutName;
		strOutName = saveName.GetFullPath() + extName1;
		saveDotDensity(plotSavePGN, mFlow, pt1, strOutName, mPdf, threshold);
		//////////////////////////////////////////		
		if(bHasEyeData){
			plotSavePGN.cmd("set size 0.5, 0.5");
			plotSavePGN.cmd("set origin 0.5, 0.5");
		}else {
			plotSavePGN.cmd("set size 0.5, 1");
			plotSavePGN.cmd("set origin 0.5, 0");		
		}	
		
		std::ostringstream cmdstr3;
		cmdstr3 << "set title '" << title2 << "'" ;
		plotSavePGN.cmd(cmdstr3.str());
		
		////////////////////////////////////////// read pdf matrix
		pdfFilename = pdfName.GetFullPath()+ extName2 + ".txt";
		fpPdf = fopen(pdfFilename.ToAscii(), "r");
		if(fpPdf ==NULL) {
			wxString str = pdfFilename + ": read error";
			wxLogMessage(str);
			continue;
		}
		for(int i=0; i<EAR_RECT; i++)
			for(int c=0; c<EAR_RECT; c++) {
				fscanf(fpPdf, "%f", &probability);
				mPdf.at<float>(i, c) = probability;
			}
		fclose(fpPdf);

		strOutName = saveName.GetFullPath() + extName2;
		saveDotDensity(plotSavePGN, mFlow, pt2, strOutName, mPdf, threshold);
			
		//////////////////////////////////////////	
		if(m_vecEyeL.size() >0){	
			plotSavePGN.cmd("set size 0.5,0.5");
			plotSavePGN.cmd("set origin 0.0,0.0");
			plotSavePGN.cmd("set title 'Eyes'");
		
		////////////////////////////////////////// read pdf matrix

			pdfFilename = pdfName.GetFullPath()+ "_Eye.txt";
			fpPdf = fopen(pdfFilename.ToAscii(), "r");
			if(fpPdf ==NULL) {
				wxString str = pdfFilename + ": read error";
				wxLogMessage(str);
				continue;
			}
			for(int i=0; i<EAR_RECT; i++)
				for(int c=0; c<EAR_RECT; c++) {
					fscanf(fpPdf, "%f", &probability);
					mPdf.at<float>(i, c) = probability;
				}
			fclose(fpPdf);

			strOutName = saveName.GetFullPath() + "_Eye";
			saveDotDensity(plotSavePGN, mFlow, ptEyeC, strOutName, mPdf, threshold);
		}
		plotSavePGN.cmd("unset multiplot");
	}	
	plotSavePGN.cmd("reset");
}
void CRat::saveDotDensity(Gnuplot& plotSavePGN, Mat& mFlow, Point pt, wxString& strOutName, Mat& mPdf, float threshold)
{
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	Mat mROI(mFlow, Rect(pt1, pt2));
	
	vector<Point2f>  vDataUp;
	vector<Point2f>  vDataBelow;
	
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
			Point2f fxy = mROI.at<Point2f>(y, x);
			Point2f fxy1 = fxy + Point2f(20, 20);
			
			if(fxy1.x >= 40 || fxy1.x <0) {
				vDataBelow.push_back(fxy);
				continue;
			}
			if(fxy1.y >= 40 || fxy1.y <0) {
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

//FILE *fp = fopen("_move.csv", "w");
void CRat::opticalFlowDistribution(vector<Mat>& vecFlow, char* subpath, vector <float>& vecPdf1, vector <float>& vecPdf2,
									Point pt1, Point pt2, char* extName1, char* extName2, char* title1, char* title2, float threshold)
{
//	char subpath[] = "distribution";
	cv::Mat	 cvMat;
	cv::Mat	 cvMatGray;
	wxString outpath; 

#if defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__) 
	outpath = m_strSrcPath + "/" + subpath;
#else
	outpath = m_strSrcPath + "\\" + subpath;
#endif	

	if(vecFlow.size() ==0) return;

	if(wxDirExists(outpath)==false) {
		if(wxMkdir(outpath)==false) {
			wxString str;
			str.Printf("Create output directory error, %s", outpath);
			wxLogMessage(str);
			return;
		}
	}
	
	bool  bHasEyeData;
	if(m_vecEyeL.size() >0)   bHasEyeData = true;
	else bHasEyeData = false;
	
	double sigma = -1;
	int ksize = 9;
	Mat mGaus ; 
	createGaussianMask(sigma, ksize, mGaus);
	MainFrame:: myMsgOutput("createGaussianMask: sigma %.2f, ksize %d\n", sigma, ksize);
	
	Gnuplot plot("lines");
#if defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__) 
	if(bHasEyeData)
		plot.cmd("set terminal png size 800, 800");
	else
		plot.cmd("set terminal png size 800, 400");
#else
	if(bHasEyeData)
		plot.cmd("set terminal pngcairo size 800, 800");
	else
		plot.cmd("set terminal pngcairo size 800, 400");
#endif		
	plot.set_xrange(-20,20);
	plot.set_yrange(-20,20);
	plot.set_zrange(0,0.15);	
	plot.set_grid();
	plot.cmd("unset key");
	plot.cmd("set vi 60,30");
	plot.set_hidden3d();
	plot.cmd("set style data lines");
	plot.cmd("set size 1,1");
	plot.cmd("set origin 0,0");
	plot.cmd("set termoption noenhanced");
	
	wxFileName fileName = wxString(m_vFilenames[0]);
	int dirCount = fileName.GetDirCount();
	wxArrayString dirs = fileName.GetDirs();
	wxString dirName = dirs[dirCount-1];
	
	Point ptEyeC;
	if(bHasEyeData){
		ptEyeC.x= (m_vecEyeL[m_referFrame].x+m_vecEyeR[m_referFrame].x)/2;
		ptEyeC.y= (m_vecEyeL[m_referFrame].y+m_vecEyeR[m_referFrame].y)/2;
	}
	
	Mat mDistEarL(40, 40, CV_32FC1, Scalar(0));
	Mat mDistEarR(40, 40, CV_32FC1, Scalar(0));
	Mat mDistEye(40, 40, CV_32FC1, Scalar(0));
	int sz = vecFlow.size();
	
//	vecPdf1.clear();
//	vecPdf2.clear();
	
	vecPdf1.resize(sz);
	vecPdf2.resize(sz);
	
//	FILE* fp = fopen("_threshold.csv", "w");	
	for(int i=0; i<sz; i++) {
		Mat& mFlow = vecFlow[i];
		
		wxFileName fileName = wxString(m_vFilenames[i]);
		fileName.AppendDir(subpath);
		wxFileName savePath = fileName.GetPath();
		wxFileName saveName(savePath.GetFullPath(), fileName.GetName());
		
		std::ostringstream cmdstr0;
		cmdstr0 << "set output '" << saveName.GetFullPath().ToAscii() << ".png'";
		plot.cmd(cmdstr0.str());
	
		std::ostringstream cmdstr1;
		cmdstr1 << "set multiplot title '" << dirName.ToAscii() << "/" << fileName.GetName().ToAscii()  ;
		cmdstr1 << ", th=" << threshold << "'";
		
		/////////////////////////////////////// plot earL 
		plot.cmd(cmdstr1.str());
		if(bHasEyeData){
			plot.cmd("set size 0.5, 0.5");
			plot.cmd("set origin 0.0, 0.5");
		}else {
			plot.cmd("set size 0.5, 1");
			plot.cmd("set origin 0.0, 0");		
		}
		std::ostringstream cmdstr2;
		cmdstr2 << "set title '" << title1 << "'" ;
		plot.cmd(cmdstr2.str());	

		wxString strOutName;
		strOutName = saveName.GetFullPath() + extName1;
		opticalBuildPDF(plot, mFlow, mGaus, mDistEarL, pt1, strOutName);
		
		/////////////////////////////////////// plot earR 
		if(bHasEyeData){
			plot.cmd("set size 0.5, 0.5");
			plot.cmd("set origin 0.5, 0.5");
		}else {
			plot.cmd("set size 0.5, 1");
			plot.cmd("set origin 0.5, 0");		
		}	

		std::ostringstream cmdstr3;
		cmdstr3 << "set title '" << title2 << "'" ;
		plot.cmd(cmdstr3.str());
		
		strOutName = saveName.GetFullPath() + extName2;
		opticalBuildPDF(plot, mFlow, mGaus, mDistEarR, pt2, strOutName);
			
		/////////////////////////////////////// plot eye 
		if(bHasEyeData){
			plot.cmd("set size 0.5,0.5");
			plot.cmd("set origin 0.0,0.0");
			plot.cmd("set title 'Eyes'");
			
			strOutName = saveName.GetFullPath() + "_Eye";
			opticalBuildPDF(plot, mFlow, mGaus, mDistEye, ptEyeC, strOutName);
		}
		plot.cmd("unset multiplot");
//	fprintf(fp, "%d\n", i);	
		/////////////////////////////////////// compute movement
		float moveEarL = optical_compute_movement(mFlow, mDistEarL, pt1, threshold);
		float moveEarR = optical_compute_movement(mFlow, mDistEarR, pt2, threshold);
		
		vecPdf1[i] = moveEarL;
		vecPdf2[i] = moveEarR;
		
//		fprintf(fp, "%f, %f\n", thLEar, thREar);

	}	
//	fclose(fp);
	plot.cmd("reset");

}


float CRat::optical_compute_movement(Mat& mFlow, Mat& mDistEar, Point pt, float threshold)
{
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	Mat mROI(mFlow, Rect(pt1, pt2));
	float movement = 0;
	double alpha = 5;
	float count = 0;
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
			Point2f fxy = mROI.at<Point2f>(y, x);
			float mv  = cv::norm(fxy);	
			
			fxy += Point2f(20, 20);
			
			if(fxy.x >= 40 || fxy.x <0) continue;
			if(fxy.y >= 40 || fxy.y <0) continue;
			
			float Pear = mDistEar.at<float>(fxy.y+0.5, fxy.x+0.5);
//			float Peye = mDistEye.at<float>(fxy.y+0.5, fxy.x+0.5);
			
			//movement += mv * Pear;//;// /(1.+exp(-Peye*alpha));
			if(Pear >= threshold) {
				movement += mv;
				count++;  
				//count+=Pear;
			}
		}
		
	if(count!=0)
		movement = (movement /count);
	  	
	return (movement);
}

float CRat::opticalBuildPDF(Gnuplot& plot, Mat& mFlow, Mat& mGaus, Mat& mDist, Point pt, wxString& strOutName)
{
	Point pt1 (pt-m_offsetEar);
	Point pt2 (pt+m_offsetEar);
	Mat mROI(mFlow, Rect(pt1, pt2));
	
	int ksize = mGaus.rows;
	int border = ksize /2;
	
	mDist = Scalar(0);
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
            const Point2f& fxy = mROI.at<Point2f>(y, x);
            if(fxy.x > 20-border-1 || fxy.x <-20+border+1) continue;
			if(fxy.y > 20-border-1 || fxy.y <-20+border+1) continue;

			Point pt1 (fxy.x+0.5-border+20, fxy.y+0.5-border+20);
			Point pt2 (fxy.x+0.5+border+20+1, fxy.y+0.5+border+20+1);

			Mat mask(mDist, Rect(pt1, pt2));
			mask += mGaus;
        }
		
	mDist /= (mROI.rows*mROI.cols);	
	wxString fName = strOutName + ".txt";
	_OutputMat(mDist, fName.ToAscii(), false);
	wxString fNameBin = strOutName + ".bin";
	_OutputMatGnuplotBinData(mDist, fNameBin.ToAscii());
	
	std::ostringstream cmdstr;
    cmdstr << "splot '" << fNameBin.ToAscii() << "' binary matrix using 1:2:3";
    plot.cmd(cmdstr.str());		

	float threshold = 0.001;
	return threshold; 
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
void CRat::opticalFlowAnalysis(vector<Mat>& vecFlow, Point ptEar, vector <float>& vecEarFlow)
{
	// compute ear motion
	int w = vecFlow[0].cols;
	int h = vecFlow[0].rows;
	
	Point pt1 (ptEar-m_offsetEar);
	Point pt2 (ptEar+m_offsetEar);
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	
//	Point ptReferEye = vecEye[m_referFrame];
	int sz = vecFlow.size();	
//	vecEarFlow.clear();
	vecEarFlow.resize(sz);
	for(int i=0; i<sz; i++) {
		float motion;
		Mat mROIEar(vecFlow[i], Rect(pt1, pt2));
		/*
		if(bOffset) {
			//Point ptEyeOffset = vecEye[i] - ptReferEye;
			motion = findAvgMotion(mROIEar);
			motion -= vecEyeMove[i]*0.2;
		}else */
			motion = findAvgMotion(mROIEar);
			
		vecEarFlow[i] = motion;
	}
}
float CRat::findAvgMotion(Mat& mFlowROI, cv::Point ptEyeOffset)
{
	float sum = 0;
	int w = mFlowROI.cols;
	int h = mFlowROI.rows;
	int sz = 0;// h*w;
    for(int y = 0; y <h; y++){
        for(int x = 0; x <w; x ++)
        {
            Point2f fxy = mFlowROI.at<Point2f>(y, x);
			//fxy.x -= ptEyeOffset.x;
			//fxy.y -= ptEyeOffset.y;
			
			float mv  = cv::norm(fxy);
			//if (mv >= 1) {
				fxy.x -= ptEyeOffset.x;
				fxy.y -= ptEyeOffset.y;
				mv  = cv::norm(fxy);
				sum += mv;
				sz++;
			//}
        }
	}

	if (sz == 0) return 0;
	return sum/ sz;
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
