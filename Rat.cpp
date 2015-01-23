#include <wx/dir.h>
#include <wx/utils.h> 

#include "MainFrame.h"
#include "Rat.h"
#include "LineDetector.h"

#include "KDE.h"

#include "itkOpenCVImageBridge.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"

//#include "MyContour.h"
#include "MyUtil.h"
#include <gsl/gsl_fit.h>

#include "BendPoint.h"

using namespace cv;

CRat::CRat()
{
	m_offsetEar = Point(EAR_RECT, EAR_RECT);
	m_offsetEye = Point(10, 10);

	m_nCageLineX = -1;
	m_nCageLineY = -1;

	m_mean = m_stddev = 0;
	m_nProcessFrame = -1;
	m_classifier = 0;
	
	m_bFirstEyeIsLeft = true;
	m_bFirstEarIsLeft = true;
}


CRat::~CRat(void)
{
}
void CRat::clearData()
{
	m_vecMat.clear();
	m_vecDest.clear();	

	m_vecEyePair.clear();
	m_vecEarPair.clear();

	m_vecLEarGrayDiff.clear();
	m_vecREarGrayDiff.clear();
	m_vecLEyeGrayDiff.clear();
	m_vecREyeGrayDiff.clear();

	m_vFilenames.clear();
	m_nSlices = 0;

	m_idxLightBegin = -1;
	m_idxTwoLight = -1;
	m_idxLightEnd = -1;
	m_nCageLineX = -1;
	m_nCageLineY = -1;
}

int CRat::readData(wxString inputPath)
{
	m_strSrcPath = inputPath;	
	
	wxString fileSpec = _T("*.bmp");
	wxArrayString  	files;
	wxDir::GetAllFiles(inputPath, &files,  fileSpec, wxDIR_FILES  );
	
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

	for(int i=0; i<vecM.size(); i++) {
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

bool CRat::verticalLine()
{
	Mat mSrc = m_vecMat[0].rowRange(0, m_nCageLineY);
	int w = mSrc.cols;
	int h = mSrc.rows;
	
	cv::Mat mDy, mDyAbs;
	cv::Sobel(mSrc, mDy, CV_32F, 1, 0, 3);
	mDyAbs = abs(mDy);


	m_nCageLineX = -1;

	int kerSize = 9;
	int angleScan = 5;
	int threshold = 1.2;
	double  stddevTh = 0.2;
	bool bRet ;
	vector<Vec2s> vecY;
	CLineDetector  detectLine(mDyAbs, kerSize);
	bRet = detectLine.detectVertical(vecY, angleScan, threshold, stddevTh);
	if(bRet==false)  return false;

	if(vecY[0].val[0] < w/2) {
		if(vecY[0].val[0] < vecY[0].val[1])
			m_nCageLineX = vecY[0].val[0];
		else
			m_nCageLineX = vecY[0].val[1];
	}else {
		if(vecY[0].val[0] < vecY[0].val[1])
			m_nCageLineX = vecY[0].val[1];
		else
			m_nCageLineX = vecY[0].val[0];
	}
	if(m_nCageLineX <30 ||  m_nCageLineX > w-30)  m_nCageLineX = -1;
//Mat color_dst;
//cvtColor( m_vecMat[0], color_dst, CV_GRAY2BGR );
//for( size_t i = 0; i < vecY.size(); i++ )
//{
//	line( color_dst, Point(vecY[i].val[1],0), Point(vecY[i].val[0], m_vecMat[0].rows), Scalar(0,0,255), 3, 8 );
//}
//imshow("ver", color_dst);

	MainFrame:: myMsgOutput("cage ver. line X %d\n", m_nCageLineX);
	return true;
}
bool CRat::horizontalLine()
{
	Mat mBin, mEdge, mIn;
	int height = m_vecMat[0].rows;

	// ¨ú¤U¥b³¡¼v¹³§PÂ_cage Ãä½t
	mIn = m_vecMat[0].rowRange(height/2, height);

	cv::Mat mDx, mDxAbs;
	cv::Sobel(mIn, mDx, CV_32F, 0, 1, 3);
	mDxAbs = abs(mDx);

	int kerSize = 9;
	int angleScan = 5;
	double threshold = 1.2;//2;
	double  stddevTh = 0.2;//0.2;
	bool bRet ;
	vector<Vec2s> vecY;
	CLineDetector  detectLine(mDxAbs, kerSize);
	bRet = detectLine.detectHorizontal(vecY, angleScan, threshold, stddevTh);
	if(bRet==false)  return false;

	double grayStddev = 43;
//FILE* fp = fopen("_graylevel1.csv", "w");
//Mat color_dst;
//cvtColor( m_vecMat[0], color_dst, CV_GRAY2BGR );
for( size_t i = 0; i < vecY.size(); i++ )
{
	cv::Point pt1, pt2;
	pt1 = Point(0, vecY[i].val[0]+height/2);
	pt2 = Point(m_vecMat[0].cols-1, vecY[i].val[1]+height/2);
	vector<cv::Point> linePts;
	vector<int> lineGrays;
	BresLine(pt1.x, pt1.y, pt2.x, pt2.y, linePts);
    for(int k=0; k<linePts.size(); k++) {
		int gray = m_vecMat[0].at<uchar>(linePts[k].y, linePts[k].x);
		lineGrays.push_back(gray);
		//fprintf(fp, "%d, ", gray);
	} 
	//fprintf(fp, "\n");
	cv::Scalar stddev, mean;
	cv::meanStdDev(lineGrays, mean, stddev);
//	TRACE("line %d: mean %.3f std %.3f", i, mean[0], stddev[0]);
	//line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
	if(stddev[0] > grayStddev) {
//		TRACE(" X\n");
		vecY.erase (vecY.begin()+i);
		i--;
		continue;
	}//else TRACE("\n");


}
//fclose(fp);
//imshow("edge-hor", color_dst);
	
	if(vecY.size() <=0) {
		MainFrame:: myMsgOutput("detect h-line error\n");
		return false;
	}
	
	if(vecY[0].val[0] > vecY[0].val[1])
		m_nCageLineY = vecY[0].val[0] +height/2;
	else
		m_nCageLineY = vecY[0].val[1] +height/2;

	MainFrame:: myMsgOutput("cage line %d\n", m_nCageLineY);
	return true;
}

bool CRat::detectTwoLight()
{
	double minArea = 330;
	double maxArea = 1600;
	double minRatio = 0.45; //0.65;
	double maxCompact = 14;

	int i;
	m_idxLightBegin = -1;
	m_idxTwoLight = -1;
	m_idxLightEnd = -1;

//	gpMainFrame->CreateProgressBar(0, m_nSlices, 1);
//	CProgressCtrl *pb =  gpMainFrame->GetProgressBarCtrl();

	//m_vecDest.clear();

	for(i=0; i<m_nSlices; i++) {
//i=2;
//Mat mColor;
		Mat mIn, mDest;
		mIn = m_vecMat[i].rowRange(m_nCageLineY, m_vecMat[i].rows);
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
//cv::drawContours(mColor, vecContour, 0, cv::Scalar(0, 255,0), 2, 8);
//imshow("color", mColor);
//waitKey(2000);


		int  numComp = vecContour.size();
		if(numComp ==1 && m_idxLightBegin ==-1)
			m_idxLightBegin = i;
	
		if(numComp ==2 && m_idxTwoLight ==-1)
			m_idxTwoLight = i;

		if(numComp ==0 && m_idxLightBegin != -1 && m_idxLightEnd==-1)
			m_idxLightEnd = i-1;

		unsigned found = m_vFilenames[i].find_last_of("/\\");
		string name =  m_vFilenames[i].substr(found+1) ;
		
//		gpOutput->ShowMessage("%s: CC= %d", name.c_str(), numComp);
		mDest = cv::Scalar(0);
		cv::drawContours(mDest, vecContour, -1, cv::Scalar(255), CV_FILLED, 8);

		//m_vecDest.push_back(mDest);

//		pb->StepIt();
//break;		
	}
	if(m_idxLightEnd==-1)  m_idxLightEnd = m_nSlices-1;

	MainFrame:: myMsgOutput("one-lantern frames [%d..%d], ----> two-lantern frame %d\n", 
		m_idxLightBegin+1, m_idxLightEnd+1, m_idxTwoLight+1); 
//	gpMainFrame->DestroyProgressBar();


	if (m_idxTwoLight < 0) 
		return false;
	else 
		return true;
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

void CRat::prepareData()
{

	vector <Mat> vecData;
	vector <string> vFilenames;
	int w = m_vecMat[0].cols;

	for(int i=0; i<m_nSlices; i++)  {
		Mat mSrc = m_vecMat[i].rowRange(0, m_nCageLineY);
		Mat mData, mV;
		if(m_nCageLineX > 0) {
			if(m_nCageLineX > w/2) {
				mV = mSrc.colRange(0, m_nCageLineX);
			}else {
				mV = mSrc.colRange(m_nCageLineX, w);
			}
			mV.copyTo(mData);
		}else
			mSrc.copyTo(mData);

		vecData.push_back(mData);
		vFilenames.push_back(m_vFilenames[i]);

		if(i>=m_idxLightBegin && i<m_idxLightEnd) 
			cv::circle(mData, Point(15, 15), 7, cv::Scalar(255), -1);
		if(i==m_idxTwoLight) 
			cv::circle(mData, Point(mData.cols-15, 15), 7, cv::Scalar(255), -1);
	}

	m_vecMat.clear();
	m_vecDest.clear();
	m_vFilenames.clear();

	int sz= vecData.size();
	m_vecMat.resize(sz);
	m_vecDest.resize(sz);
	m_vFilenames.resize(sz);

	std::copy(vecData.begin(), vecData.end(), m_vecMat.begin());
	std::copy(vecData.begin(), vecData.end(), m_vecDest.begin());
	std::copy(vFilenames.begin(), vFilenames.end(), m_vFilenames.begin());

	m_szImg = Size(m_vecMat[0].cols, m_vecMat[0].rows );
	m_nSlices = m_vecMat.size();

	saveResult("cage", m_vecMat);
}

void CRat::process1(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR)
{
	recognizeLeftRight(ptEyeL, ptEyeR, ptEarL, ptEarR);	
	m_ptEyeL = ptEyeL;
	m_ptEyeR = ptEyeR;
	m_ptEarL = ptEarL;
	m_ptEarR = ptEarR;
	
	// save eye ear marks 
	wxFileName fileName = m_strSrcPath;
	wxFileName dataName(m_strSrcPath, "_eye_earMarks.txt");
	FILE* fp = fopen(dataName.GetFullPath(), "w");
	if(fp!=NULL) {
		fprintf(fp, "%d %d %d %d\n", ptEyeL.x, ptEyeL.y, ptEyeR.x, ptEyeR.y );
		fprintf(fp, "%d %d %d %d\n", ptEarL.x, ptEarL.y, ptEarR.x, ptEarR.y );
		fclose(fp);
	}
	
	int stableFrame = findStablePoint(ptEarL);
	MainFrame:: myMsgOutput("stable frame %d\n", stableFrame);
	if(stableFrame <0) {
		wxLogMessage("cannot find stable frame");
		return;
	}

	graylevelDiff(stableFrame, ptEyeL, ptEyeR, ptEarL, ptEarR);
	
	vector<double> smoothL;
	smoothData(m_vecLEarGrayDiff, smoothL, 2);	
	
	vector<double> smoothR;
	smoothData(m_vecREarGrayDiff, smoothR, 2);	
		
	
	int maxPointL = findMaxMotionPoint(smoothL);
	int maxPointR = findMaxMotionPoint(smoothR);
	
	const char* title =fileName.GetName();
	_gnuplotLantern(title, m_idxLightBegin, m_idxTwoLight);
	_gnuplotLine("LeftEar", m_vecLEarGrayDiff);
	_gnuplotLine("RightEar", m_vecREarGrayDiff);	
	//_gnuplotLine("LeftEar_Smooth", smoothL);
	//_gnuplotLine("LeftEar_Smooth", smoothR);
	

	if(m_vecLEarGrayDiff[maxPointL]> m_vecREarGrayDiff[maxPointR]) {
		MainFrame:: myMsgOutput("max motion: left ear %d\n", maxPointL);
		_gnuplotLine("maxMotion", maxPointL);
		saveEarROI(stableFrame, maxPointL, ptEarL);
	}else {
		MainFrame:: myMsgOutput("max motion: right ear %d\n", maxPointR);
		_gnuplotLine("maxMotion", maxPointR);
		saveEarROI(stableFrame, maxPointR, ptEarR);
	}
	
	
	
/*		
	///////////////////////////////////////////find bending points
	int k = 3;
	int angleTh = 170;
	
	int  npts = m_idxTwoLight+k*2;
	vector<cv::Point> contour(npts);
	for(int i=0; i<npts; i++) {
		cv::Point p(i, smooth[i]*100+0.5);
		contour[i] = p;
		MainFrame:: myMsgOutput("[%d, %d] ", p.x, p.y);
	}
	
	

	CBendPoint  bendpt;
	vector<float> curvature;
	int szBending = bendpt.bendingPointsByCurvature(contour, k, angleTh, curvature);
	MainFrame:: myMsgOutput("szBending %d\n", szBending);
	
	vector<double> peakX;
	vector<double> peakY;
	for(int i=0; i<szBending; i++) {	
		cv::Point pt = contour[bendpt.m_bendingIdx[i]];
		peakX.push_back(pt.x);
		peakY.push_back((pt.y/100.0));	
	
		MainFrame:: myMsgOutput("[%d, %d]", pt.x, pt.y);
	
	}
	_gnuplotPoint("point", peakX, peakY);
	
	///////////////////////////////////////////find bending points
	int nSize = smooth.size();
	int segLen = 5;
	double threshold = 0.15;
	int initX = 0;
	int lastX = m_idxTwoLight;
	
	double * pOutput = smooth.data();
	
	CBendPoint  bendpt(pOutput, nSize, segLen);
	bendpt.findAllBendPoints(initX, lastX, threshold);	
	
	int  numPeaks = bendpt.m_vPeak.size();
	int  numNotches = bendpt.m_vNotch.size();	
	
	vector<double> peakX;
	vector<double> peakY;
	for(int i=0; i<numPeaks; i++) {
		peakX.push_back(bendpt.m_vPeak[i].idx);
		peakY.push_back(smooth[bendpt.m_vPeak[i].idx]);		
	}
	_gnuplotPoint("point", peakX, peakY);
	
	vector<double> notchX;
	vector<double> notchY;
	for(int i=0; i<numNotches; i++) {
		notchX.push_back(bendpt.m_vNotch[i].idx);
		notchY.push_back(smooth[bendpt.m_vNotch[i].idx]);		
	}
	_gnuplotPoint("point", notchX, notchY);	
	 */ 
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
	
	
	wxFileName bmpName1(m_strSrcPath, "_stable.bmp");
	wxFileName bmpName2(m_strSrcPath, "_motion.bmp");
	wxFileName bmpName3(m_strSrcPath, "_edge.bmp");
		
	imwrite(bmpName1.GetFullPath().ToStdString(), mROI0);
	imwrite(bmpName2.GetFullPath().ToStdString(), mROI1);
	imwrite(bmpName3.GetFullPath().ToStdString(), mColor);
	
	imshow("_stable", mROI0);
	imshow("_motion", mROI1); 
	imshow("_ColorEdge", mColor);
}
int CRat::findMaxMotionPoint(vector<double>& inData)
{
	int maxPoint = -1;
	
	for(int i=m_idxTwoLight+3; i<m_nSlices; i++) {
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

int CRat::findStablePoint(Point& pt)
{
	int start, end, lenSeg;
	start = 0;
	end = m_idxTwoLight + 10;
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
	_gnuplotLine("LeftEar", vSeries);	
	
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

void CRat::recognizeLeftRight(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR)
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

double CRat::errorSum(Mat &mDiff, Point ptEarL)
{
	int wImg = m_vecMat[0].cols;
	int hImg = m_vecMat[0].rows;
	
	Point pt1 (ptEarL-m_offsetEar);
	Point pt2 (ptEarL+m_offsetEar);
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= wImg) pt2.x = wImg-1;
	if(pt2.y >= hImg) pt2.y = hImg-1;
	
	Mat mROI(mDiff, Rect(pt1, pt2));
	
	Scalar sSum = cv::mean(mROI);
	
	return sSum[0];
}

void CRat::smoothData(vector<double>& inData, vector<double>& outData, int bw)
{
	int  n = inData.size();	

	double* pData = inData.data();

	double  *out = new double[n];

	CKDE::MSKernel kde_kernel = CKDE::Gaussian;
	
	CKDE kde1x(pData, out, n);
	kde1x.KernelDensityEstimation(kde_kernel, bw);

	outData.assign (out,out+n);	
}

void CRat::graylevelDiff(int stable, Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR)
{
	m_vecLEarGrayDiff.clear();
	m_vecREarGrayDiff.clear();
	m_vecLEyeGrayDiff.clear();
	m_vecREyeGrayDiff.clear();
	
	Mat mSrc1, mSrc2, mDiff;
	
	mSrc1 = m_vecMat[stable]; //17
	
	for(int i=0; i<m_nSlices; i++) {
		double errSumL, errSumR;
		mSrc2 = m_vecMat[i];
		cv::absdiff(mSrc1, mSrc2, mDiff);
		
		errSumL = errorSum(mDiff, ptEarL);
		errSumR = errorSum(mDiff, ptEarR);
		m_vecLEarGrayDiff.push_back(errSumL);
		m_vecREarGrayDiff.push_back(errSumR);
		
		errSumL = errorSum(mDiff, ptEyeL);
		errSumR = errorSum(mDiff, ptEyeR);		
		m_vecLEyeGrayDiff.push_back(errSumL);
		m_vecREyeGrayDiff.push_back(errSumR);
	}
}


float CRat::findSumMotion(Mat& mFlowROI, cv::Point& ptDiff)
{
	float sum = 0;
	int w = mFlowROI.cols;
	int h = mFlowROI.rows;
	int sz = 0;// h*w;
    for(int y = 0; y <h; y++){
        for(int x = 0; x <w; x ++)
        {
            Point2f fxy = mFlowROI.at<Point2f>(y, x);
			//fxy.x -= ptDiff.x;
			//fxy.y -= ptDiff.y;
			
			float mv  = cv::norm(fxy);
			if (mv >= 1) {
				sum += mv;
				sz++;
			}
        }
	}
//	gpOutput->ShowMessage("sz %d  %d \n", h*w, sz);
	if (sz == 0) return 0;
	return sum/ sz;
}

float CRat::findMaxMotion(Mat& mFlowROI, cv::Point& ptDiff)
{
	float mv, max = 0;
	int w = mFlowROI.cols;
	int h = mFlowROI.rows;
    for(int y = 0; y <h; y++){
        for(int x = 0; x <w; x ++)
        {
            Point2f fxy = mFlowROI.at<Point2f>(y, x);
			//fxy.x -= ptDiff.x;
			//fxy.y -= ptDiff.y;
			
			mv  = cv::norm(fxy);

			if(mv > max) max = mv;
        }
	}
	return mv;
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

}

void CRat::locateEarPosition(Point& ptEarL, Point& ptEarR)
{
	m_vecEarPair.clear();
	m_vecEarPair.resize(m_nSlices);

	TwoPts eye = m_vecEyePair[0];
	Point pt = ptEarL - eye.ptL;
	double dist1 = norm(pt); // L2 norm
	pt = ptEarL - eye.ptR;
	double dist2 = norm(pt); // L2 norm

	if (dist1 > dist2) {
		Point tmp;
		tmp = ptEarL;
		ptEarL = ptEarR;
		ptEarR = tmp;
	}

	TwoPts  earPair;
	earPair.ptL = ptEarL;
	earPair.ptR = ptEarR;
	m_vecEarPair[0] = earPair;
	TwoPts eye0 = m_vecEyePair[0];
	for (int i = 1; i < m_nSlices; i++) {
		TwoPts  earPair;
		earPair.ptL = ptEarL;
		earPair.ptR = ptEarR;

		TwoPts eye1 = m_vecEyePair[i];
		Point dispEyeL = eye1.ptL - eye0.ptL;
		Point dispEyeR = eye1.ptR - eye0.ptR;

		earPair.ptL += dispEyeL;
		earPair.ptR += dispEyeR;

		m_vecEarPair[i] = earPair;
	}
	Point rectSz(EAR_RECT, EAR_RECT);
	for (int i = 0; i < m_nSlices; i++) {
		Mat mDestColor = m_vecDest[i];
		TwoPts  earPair = m_vecEarPair[i];
		rectangle(mDestColor, earPair.ptL - rectSz, earPair.ptL + rectSz, Scalar(0, 255, 0));
		rectangle(mDestColor, earPair.ptR - rectSz, earPair.ptR + rectSz, Scalar(0, 255, 0));
	}
}

void CRat::setROIRect(Rect& rectEye, Point pt1, Point pt2)
{
	int left, right, top, bottom;
	if (pt1.x < pt2.x) {
		left = pt1.x - 20;
		right = pt2.x + 20;
	}
	else {
		left = pt2.x - 20;
		right = pt1.x + 20;
	}
	if (pt1.y < pt2.y) {
		top = pt1.y - 20;
		bottom = pt2.y + 20;
	}
	else {
		top = pt2.y - 20;
		bottom = pt1.y + 20;
	}

	int rows = m_vecMat[0].rows;
	int cols = m_vecMat[0].cols;
	if (top < 0) top = 0;
	if (left < 0) left = 0;
	if (bottom >= rows) bottom = rows - 1;
	if (right >= cols) right = cols - 1;
	
	rectEye.x = left;
	rectEye.y = top;
	rectEye.width = right - left +1;
	rectEye.height = bottom - top+1;
	

}
void CRat::findMouseEyes(int numFrame, Point ptEyeL, Point ptEyeR)
{
	m_nProcessFrame = numFrame;

	m_vecDest.clear();
	m_vecEyePair.clear();


//	gpMainFrame->CreateProgressBar(0, m_nSlices, 1);
//	CProgressCtrl *pb = gpMainFrame->GetProgressBarCtrl();

	MainFrame:: myMsgOutput("Process frame ...%d\n", numFrame);

	int is, it;
	clock_t start, finish;
	double  duration;
	start = clock();

	for (int i = 0; i < m_nSlices; i++)
	{
		Mat mDestColor;
		cvtColor(m_vecMat[i], mDestColor, CV_GRAY2BGR);
		m_vecDest.push_back(mDestColor);
	}
	m_vecEyePair.resize(m_nSlices);
	bool bShow = false;
	if (numFrame < 0) {
		is = 0;
		it = m_nSlices;
		bShow = false;
	}
	else {
		is = numFrame;
		it = numFrame + 1;
		bShow = true;
	}


	int foundEye = 0;
	//#pragma omp parallel for 
	for (int i = is; i < it; i++)
	{
		Rect rectEye;
		setROIRect(rectEye, ptEyeL, ptEyeR);

		int w = rectEye.width;// - rectEye.left + 1;
		int h = rectEye.height;// - rectEye.top + 1;
		Point ofs(rectEye.x, rectEye.y);
		Rect  r = rectEye; //(rectEye.left, rectEye.top, w, h);

		ptEyeL -= ofs;
		ptEyeR -= ofs;

		Mat mDestColor;
		TwoPts  eyePair;

		Mat mROI(m_vecMat[i], r);
		mDestColor = m_vecDest[i];


		// detect eyes
		vector<vector<cv::Point> > vecContour;
		double angle;
		int idxEye = detectEyes(mROI, vecContour, ptEyeL, ptEyeR, bShow);
		if (idxEye < 0) {
			MainFrame:: myMsgOutput("cannot detect eye pair, frame %d", i + 1);
			foundEye--;
			continue;
			//break;
		}
		foundEye++;

		Mat mROIDest(mDestColor, r);
		cv::drawContours(mROIDest, vecContour, -1, cv::Scalar(0, 255, 0), 1, 8);

		ptEyeL += ofs;
		ptEyeR += ofs;

		eyePair.ptL = ptEyeL;
		eyePair.ptR = ptEyeR;

		m_vecEyePair[i] = eyePair;

		line(mDestColor, eyePair.ptL, eyePair.ptR, Scalar(255, 0,0), 1);
		cv::circle(mDestColor, eyePair.ptL, 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(mDestColor, eyePair.ptR, 3, cv::Scalar(180, 105, 255), -1);

		Point ptEyeCen;
		ptEyeCen.x = (eyePair.ptL.x + eyePair.ptR.x) / 2;
		ptEyeCen.y = (eyePair.ptL.y + eyePair.ptR.y) / 2;
		///line(mDestColor, ptEyeCen, ptCentroid, Scalar(147, 238, 144), 2);
//		pb->StepIt();
	}
	if (it - is == m_nSlices) {
		correctEyeDisp();
//		correctEyePos();
		//locateEarPosition(ptEar1, ptEar2);
	}
//	pb->SetPos(m_nSlices);
//	gpMainFrame->DestroyProgressBar();
	MainFrame:: myMsgOutput("eye detection %d/%d, loss %d\n", foundEye, m_nSlices, m_nSlices - foundEye);
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	int minutes = duration / 60;
	int second = duration - minutes * 60;
	MainFrame:: myMsgOutput("computation time: %02dm:%02ds\n", minutes, second);

//	gpMainFrame->DestroyProgressBar();

	//saveResult("dest", m_vecDest);
	MainFrame:: myMsgOutput("------ find mouse eye done ------\n");

}
int CRat::detectEyes(Mat &mROI, vector<vector<cv::Point> >& vecContour, Point& ptL, Point& ptR, bool bShow)
{
	Mat mLoG, mROIM, mEye, mEyeCopy;

	//#pragma omp critical 
	{
		itkLaplacianOfGaussian(mROI, mLoG, 3);
	}
	double  min, max;
	cv::minMaxLoc(mLoG, &min, &max);
	float z = 255. / (max - min);
	mLoG.convertTo(mEye, CV_8UC1, z, -min*z);
	cv::threshold(mEye, mEye, 0, 255, THRESH_BINARY | THRESH_OTSU);

	//mEye = mEye & mROIMask;
	//imshow("mEye0", mEye);
	mEye.copyTo(mEyeCopy);
//	vector<vector<cv::Point> > vecContour;
	findContours(mEye, vecContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	double minArea = 10; //15;
	double maxArea = 400; //200;
	int	   minGraylevel = 200;

	for (int k = 0; k < vecContour.size(); k++) {
		double area = cv::contourArea(vecContour[k]);
		int numPts = vecContour[k].size();
		if (area >= maxArea || area < minArea) {
			vecContour.erase(vecContour.begin() + k);
			k--;
			continue;
		}
	}
	if (bShow) {
		Mat mEyeClr = mROI.clone(); //cv::Mat::zeros(mEye.size(), CV_8UC3);
		cv::cvtColor(mEyeClr, mEyeClr, CV_GRAY2BGR);
		cv::drawContours(mEyeClr, vecContour, -1, cv::Scalar(0, 255, 255), 1, 8);
		line(mEyeClr, ptL, ptR, Scalar(100, 0, 100), 1);
		imshow("mEye", mEyeClr);
	}

	int idxPt1 = -1;
	double maxInside = -999;
	Point2f fpt1 = ptL;
	for (int k = 0; k < vecContour.size(); k++) {
		double inside = cv::pointPolygonTest(vecContour[k], fpt1, true);
		if (inside > 0) {
			idxPt1 = k;
			break;
		}
		if (inside > maxInside) {
			maxInside = inside;
			idxPt1 = k;
		}
	}
	if (idxPt1 < 0) {
		Mat mEyeClr = mROI.clone(); //cv::Mat::zeros(mEye.size(), CV_8UC3);
		cv::cvtColor(mEyeClr, mEyeClr, CV_GRAY2BGR);
		cv::drawContours(mEyeClr, vecContour, -1, cv::Scalar(0, 255, 255), 1, 8);
		line(mEyeClr, ptL, ptR, Scalar(100, 0, 100), 1);
		imshow("mEye", mEyeClr);
		return -1;
	}
	int idxPt2 = -1;
	maxInside = -999;
	Point2f fpt2 = ptR;
	for (int k = 0; k < vecContour.size(); k++) {
		double inside = cv::pointPolygonTest(vecContour[k], fpt2, true);
		if (inside > 0) {
			idxPt2 = k;
			break;
		}
		if (inside > maxInside) {
			maxInside = inside;
			idxPt2 = k;
		}
	}
	if (idxPt2 < 0)  {
		Mat mEyeClr = mROI.clone(); //cv::Mat::zeros(mEye.size(), CV_8UC3);
		cv::cvtColor(mEyeClr, mEyeClr, CV_GRAY2BGR);
		cv::drawContours(mEyeClr, vecContour, -1, cv::Scalar(0, 255, 255), 1, 8);
		line(mEyeClr, ptL, ptR, Scalar(100, 0, 100), 1);
		imshow("mEye", mEyeClr);
		return -2;
	}

	Scalar m = cv::mean(vecContour[idxPt1]);
	ptL.x = m[0];
	ptL.y = m[1];
	m = cv::mean(vecContour[idxPt2]);
	ptR.x = m[0];
	ptR.y = m[1];


	//double area1 = cv::contourArea(vecContour[idxPt1]);
	//double area2 = cv::contourArea(vecContour[idxPt2]);

//	TRACE("%.3f, (%d %d), %.3f, (%d %d)\n", area1, pt1.x, pt1.y, area2, pt2.x, pt2.y);
//	modifyEyeLocation(mEyeCopy, pt1, pt2, area1, area2);
//	TRACE("(%d %d), (%d %d)\n", pt1.x, pt1.y, pt2.x, pt2.y);
	return  1;

}

void CRat::correctEyeDisp()
{
	Point ptNew1, ptNew2;
	Point ptEye1, ptEye2;
	Point disp1, disp2;
	for (int i = 1; i < m_nSlices; i++) {
		ptEye1 = m_vecEyePair[i-1].ptL;
		ptEye2 = m_vecEyePair[i-1].ptR;

		ptNew1 = m_vecEyePair[i].ptL;
		ptNew2 = m_vecEyePair[i].ptR;

		disp1 = ptNew1 - ptEye1;
		disp2 = ptNew2 - ptEye2;

		double dist1, dist2, minD, maxD;
		dist1 = norm(disp1);
		dist2 = norm(disp2);
		minD = MIN(dist1, dist2);
		maxD = MAX(dist1, dist2);
		//gpOutput->ShowMessage("disp [%d] %.4f  %.4f\n", i + 1, minD, maxD);
		if (minD < 0.01) {
			if (dist1 >= dist2) {
				ptNew1 = ptEye1;
				m_vecEyePair[i].ptL = ptNew1;
			}
			else {
				ptNew2 = ptEye2;
				m_vecEyePair[i].ptR = ptNew2;
			}
		}else if (minD > 3) {
			ptNew1 = ptEye1;
			m_vecEyePair[i].ptL = ptNew1;

			ptNew2 = ptEye2;
			m_vecEyePair[i].ptR = ptNew2;
		}else if (maxD-minD > 2.0) { 		
			if (dist1 > dist2) {
				if (minD < 0.2)
					ptNew1 = ptEye1;
				else
					ptNew1 = ptEye1 + 0.7*disp2 ;
				m_vecEyePair[i].ptL = ptNew1;
			}
			else {
				if (minD < 0.2)
					ptNew2 = ptEye2;
				else
					ptNew2 = ptEye2 + 0.7*disp1 ;
				m_vecEyePair[i].ptR = ptNew2;
			}
		}
		
		Mat mDestColor = m_vecDest[i];
		line(mDestColor, m_vecEyePair[i].ptL, m_vecEyePair[i].ptR, Scalar(0,255,255), 2);

	}
}
void CRat::correctEyePos()
{
	MainFrame:: myMsgOutput("correct Eye Position +++++++++++++++\n");

	double th = 0.02;// 0.008;

	double* histo1X;
	double* histo1Y;
	double* histo2X;
	double* histo2Y;

	int w = m_vecMat[0].cols;
	int h = m_vecMat[0].rows;

	histo1X = new double[w];
	histo1Y = new double[h];
	histo2X = new double[w];
	histo2Y = new double[h];

	memset(histo1X, 0, w*sizeof(double));
	memset(histo2X, 0, w*sizeof(double));
	memset(histo1Y, 0, h*sizeof(double));
	memset(histo2Y, 0, h*sizeof(double));

	for(int i=0; i<m_nSlices; i++) {
		histo1X[m_vecEyePair[i].ptL.x]++;
		histo1Y[m_vecEyePair[i].ptL.y]++;

		histo2X[m_vecEyePair[i].ptR.x]++;
		histo2Y[m_vecEyePair[i].ptR.y]++;
	}
	for(int i=0; i<w; i++) {
		histo1X[i] /= m_nSlices;
		histo2X[i] /= m_nSlices;
	}
	for(int i=0; i<h; i++) {
		histo1Y[i] /= m_nSlices;
		histo2Y[i] /= m_nSlices;
	}

	double  *outkde1X = new double[w];
	double  *outkde1Y = new double[h];
	double  *outkde2X = new double[w];
	double  *outkde2Y = new double[h];

	int bw = 10;
	CKDE::MSKernel kde_kernel = CKDE::Gaussian;
	CKDE kde1x(histo1X, outkde1X, w);
	kde1x.KernelDensityEstimation(kde_kernel, bw);

	CKDE kde1y(histo1Y, outkde1Y, h);
	kde1y.KernelDensityEstimation(kde_kernel, bw);

	CKDE kde2x(histo2X, outkde2X, w);
	kde2x.KernelDensityEstimation(kde_kernel, bw);

	CKDE kde2y(histo2Y, outkde2Y, h);
	kde2y.KernelDensityEstimation(kde_kernel, bw);

	for(int i=0; i<m_nSlices; i++) {
		double px1 = outkde1X[m_vecEyePair[i].ptL.x];
		double py1 = outkde1Y[m_vecEyePair[i].ptL.y];
		double px2 = outkde2X[m_vecEyePair[i].ptR.x];
		double py2 = outkde2Y[m_vecEyePair[i].ptR.y];

		bool bCorrect = false;
		if(px1 < th || py1 < th ){
			MainFrame:: myMsgOutput("outlier eyeL position, slice %d\n", i+1);
			if(i>0) {
				int x1 = m_vecEyePair[i].ptL.x;
				int x2 = m_vecEyePair[i - 1].ptL.x;
				int y1 = m_vecEyePair[i].ptL.y;
				int y2 = m_vecEyePair[i - 1].ptL.y;
				int dist = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
				if (dist >50)
					m_vecEyePair[i].ptL = m_vecEyePair[i-1].ptL;				
			}else {
				int k;
				bool bFound = false;
				for(k=i+1; k<m_nSlices; k++)  {
					double tpx1 = outkde1X[m_vecEyePair[k].ptL.x];
					double tpy1 = outkde1Y[m_vecEyePair[k].ptL.y];
					if(tpx1 > th && tpy1 > th ) {
						bFound = true;
						break;
					}
				}
				if (bFound) {
					int x1 = m_vecEyePair[i].ptL.x;
					int x2 = m_vecEyePair[k].ptL.x;
					int y1 = m_vecEyePair[i].ptL.y;
					int y2 = m_vecEyePair[k].ptL.y;
					int dist = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
					if (dist >50)
						m_vecEyePair[i].ptL = m_vecEyePair[k].ptL;
				}else 
					MainFrame:: myMsgOutput("-------------> cannot correct eye position\n");
			}
			bCorrect = true;
		}

		if(px2 < th || py2 < th ){
			MainFrame:: myMsgOutput("outlier eyeR position, slice %d\n", i+1);
			if(i>0) {
				int x1 = m_vecEyePair[i].ptR.x;
				int x2 = m_vecEyePair[i - 1].ptR.x;
				int y1 = m_vecEyePair[i].ptR.y;
				int y2 = m_vecEyePair[i - 1].ptR.y;
				int dist = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
				if (dist >50)
					m_vecEyePair[i].ptR = m_vecEyePair[i - 1].ptR;
			}else {
				int k;
				bool bFound = false;
				for(k=i+1; k<m_nSlices; k++)  {
					double tpx2 = outkde2X[m_vecEyePair[k].ptR.x];
					double tpy2 = outkde2Y[m_vecEyePair[k].ptR.y];
					if(tpx2 > th && tpy2 > th ) {
						bFound = true;
						break;
					}
				}
				if (bFound) {
					int x1 = m_vecEyePair[i].ptR.x;
					int x2 = m_vecEyePair[k].ptR.x;
					int y1 = m_vecEyePair[i].ptR.y;
					int y2 = m_vecEyePair[k].ptR.y;
					int dist = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
					if (dist >50)
						m_vecEyePair[i].ptR = m_vecEyePair[k].ptR;					
				}
				else
					MainFrame:: myMsgOutput("-------------> cannot correct eye position\n");
			}
			bCorrect = true;
		}

		if(bCorrect) {
			Mat mDestColor = m_vecDest[i]; 
			line(mDestColor, m_vecEyePair[i].ptL, m_vecEyePair[i].ptR, Scalar(0, 0, 255), 2);
		}
	}

	//FILE* fp = fopen("_1x.csv", "w");
	//for(int i=0; i<w; i++)
	//	fprintf(fp, "%f\n", outkde1X[i]);
	//fclose(fp);

	//fp = fopen("_1y.csv", "w");
	//for(int i=0; i<h; i++)
	//	fprintf(fp, "%f\n", outkde1Y[i]);
	//fclose(fp);

	FILE *fpp = fopen("_propX.csv", "w");
	for(int i=0; i<m_nSlices; i++)
		fprintf(fpp, "%f, %f, %f, %f\n", outkde1X[m_vecEyePair[i].ptL.x], outkde1Y[m_vecEyePair[i].ptL.y],
										 outkde2X[m_vecEyePair[i].ptR.x], outkde2Y[m_vecEyePair[i].ptR.y]);
	fclose(fpp);

	delete [] outkde1X;
	delete [] outkde1Y;
	delete [] outkde2X;
	delete [] outkde2Y;

	delete [] histo1X;
	delete [] histo1Y;
	delete [] histo2X;
	delete [] histo2Y;
}


void CRat::modifyEyeLocation(Mat& mEye, Point& ptEye1, Point& ptEye2, double area1, double area2)
{
	double area;
	Mat mDia, mT, mResult;
	int szStruct = 3;
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(szStruct, szStruct));
	//cv::morphologyEx(mEye, mEye, CV_MOP_CLOSE, element);
	cv::dilate(mEye, mDia, element);

	Point ptNew1, ptNew2;
	bool  bChange1 = false;
	bool  bChange2 = false;

	Scalar newVal1(128);
	cv::floodFill(mDia, ptEye1, newVal1);
	mResult = mDia == 128;

	vector<vector<cv::Point> > vecContour;
	findContours(mResult, vecContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	if (vecContour.size() != 0) {
		area = cv::contourArea(vecContour[0]);
		//gpOutput->ShowMessage("area %.3f, area1 %.3f\n", area, area1);
		if (area < 4 * area1 && area > 3 * area1) {
			Scalar m = cv::mean(vecContour[0]);
			ptNew1.x = m[0];
			ptNew1.y = m[1];
			bChange1 = true;
		}
	}

	Scalar newVal2(100);
	cv::floodFill(mDia, ptEye2, newVal2);
	mResult = mDia == 100;

	vecContour.clear();
	findContours(mResult, vecContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	if (vecContour.size() != 0) {
		area = cv::contourArea(vecContour[0]);
		if (area < 4 * area2 && area > 3 * area2) {
			Scalar m = cv::mean(vecContour[0]);
			ptNew2.x = m[0];
			ptNew2.y = m[1];
			bChange2 = true;
		}
	}

	if (bChange1 && bChange2) {
		Point disp1, disp2;
		disp1 = ptNew1 - ptEye1;
		disp2 = ptNew2 - ptEye2;
		double dist1, dist2, minD;
		dist1 = norm(disp1);
		dist2 = norm(disp2);
		minD = MIN(dist1, dist2);

		if (fabs(dist1 - dist2) < 1.5 * minD) {
			ptEye1 = ptNew1;
			ptEye2 = ptNew2;
		}
	}
}

void CRat::findMouseEars(int numFrame, Point ptEarL, Point ptEarR)
{
	m_nProcessFrame = numFrame;

	m_vecEarPair.clear();
	m_vecEarPair.resize(m_nSlices);

	MainFrame:: myMsgOutput("Process frame ...%d\n", numFrame);

	int is, it;
	clock_t start, finish;
	double  duration;
	start = clock();

	bool bShow = false;
	if (numFrame < 0) {
		is = 0;
		it = m_nSlices;
		bShow = false;
	}
	else {
		is = numFrame;
		it = numFrame + 1;
		bShow = true;
	}

	Point ptNewEarL, ptNewEarR;
	ptNewEarL = ptEarL;
	ptNewEarR = ptEarR;
	
	int foundEar = 0;
	//#pragma omp parallel for 
	for (int i = is; i < it; i++)
	{
		// detect ears
		vector<vector<cv::Point> > vecContour;
		int idxEar = detectEars(m_vecMat[i], vecContour, ptNewEarL, ptNewEarR, bShow);
		if (idxEar < 0) {
			MainFrame:: myMsgOutput("cannot detect ear pair, frame %d\n", i + 1);
			foundEar--;
			continue;
			//break;
		}
		foundEar++;

		Mat mDestColor;
		TwoPts  earPair;
		mDestColor = m_vecDest[i];
		cv::drawContours(mDestColor, vecContour, -1, cv::Scalar(0, 255, 0), 1, 8);

		earPair.ptL = ptEarL;
		earPair.ptR = ptEarR;

		m_vecEarPair[i] = earPair;

		Point rectSz(EAR_RECT, EAR_RECT);
		rectangle(mDestColor, earPair.ptL - rectSz, earPair.ptL + rectSz, Scalar(0, 255, 0));
		rectangle(mDestColor, earPair.ptR - rectSz, earPair.ptR + rectSz, Scalar(0, 255, 0));

		cv::circle(mDestColor, earPair.ptL, 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(mDestColor, earPair.ptR, 3, cv::Scalar(0, 0, 255), -1);

//		pb->StepIt();
	}
	//if (it - is == m_nSlices) {
		//correctEyePos();
	//}
//	pb->SetPos(m_nSlices);
//	gpMainFrame->DestroyProgressBar();
	MainFrame:: myMsgOutput("ear detection %d/%d, loss %d\n", foundEar, m_nSlices, m_nSlices - foundEar);
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	int minutes = duration / 60;
	int second = duration - minutes * 60;
	MainFrame:: myMsgOutput("computation time: %02dm:%02ds\n", minutes, second);

//	gpMainFrame->DestroyProgressBar();

	saveResult("dest", m_vecDest);

	MainFrame:: myMsgOutput("------ find mouse ear done ------\n");

}

int CRat::detectEars(Mat &mIn, vector<vector<cv::Point> >& vecContour, Point& ptL, Point& ptR, bool bShow)
{
	Point twoEarCenter;
	twoEarCenter.x = (ptL.x + ptR.x) / 2;
	twoEarCenter.y = (ptL.y + ptR.y) / 2;

	Mat mEar = cv::Mat::zeros(mIn.size(), CV_8UC1);
	if (bShow)
		MainFrame:: myMsgOutput("detectOneEars 1\n");
	int idx1, idx2;
	vector<vector<cv::Point> > vecContour1;
	idx1 = detectOneEars(mIn, vecContour1, ptL, twoEarCenter, mEar);

	if (bShow)
		MainFrame:: myMsgOutput("detectOneEars 2\n");
	vector<vector<cv::Point> > vecContour2;
	idx2 = detectOneEars(mIn, vecContour2, ptR, twoEarCenter, mEar);

	int ret = -1;
	if (idx1 >= 0 && idx2 >= 0)
		ret = 1;

	if (bShow) imshow("ear", mEar);

	findContours(mEar, vecContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	return  ret;

}

int CRat::detectOneEars(Mat &mIn, vector<vector<cv::Point> >& vecContour, Point& pt, Point& twoEarCenter, Mat& mEarOut)
{
	Mat mLoG, mROIM, mEar, mEarCopy;

	Rect  rect;
	int x, y;
	x = MAX(0, pt.x - EAR_RECT);
	y = MAX(0, pt.y - EAR_RECT);
	rect = Rect(x, y, EAR_RECT * 2, EAR_RECT * 2);
	rect &= Rect(0, 0, mIn.cols, mIn.rows);

	Mat mROI(mIn, rect);
	Mat mEarDest(mEarOut, rect);
	itkLaplacianOfGaussian(mROI, mLoG, 3);

	double  min, max;
	cv::minMaxLoc(mLoG, &min, &max);
	float z = 255. / (max - min);
	mLoG.convertTo(mEar, CV_8UC1, z, -min*z);
	cv::threshold(mEar, mEar, 0, 255, THRESH_BINARY | THRESH_OTSU);

	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	//cv::morphologyEx(mEar, mEar, CV_MOP_CLOSE, element);
	cv::dilate(mEar, mEar, element);

	findContours(mEar, vecContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//	cv::drawContours(mEarDest, vecContour, -1, cv::Scalar(255), 1, 8);

	int idx1 = -1;
	double maxArea = 100; //80;
	double minDist = 999;
	double oldDistTh = 30;
	for (int k = 0; k < vecContour.size(); k++) {
		double area = cv::contourArea(vecContour[k]);
		if (area < maxArea) continue;
//		if (!checkEarRegion(mROI, vecContour, k)) continue;
		Scalar m = cv::mean(vecContour[k]);
		Point center, ptDiff;
		center.x = m[0] + x;
		center.y = m[1] + y;

		ptDiff = center - pt;
		double oldDist = cv::norm(ptDiff);
		if (oldDist > oldDistTh) continue;

		ptDiff = center - twoEarCenter; // pt;
		double dist = cv::norm(ptDiff);

		if (dist < minDist) {
			idx1 = k;
			minDist = dist;
		}
	}
	if (idx1 < 0) return -1;

	int idx2 = -1;
	maxArea = 100; //200;
	for (int k = 0; k < vecContour.size(); k++) {
		if (k != idx1) {
			double area = cv::contourArea(vecContour[k]);
			if (area < maxArea) continue;
			if (!checkEarRegion(mROI, vecContour, idx1, k)) continue;

			if (area > maxArea) {
				idx2 = k;
				maxArea = area;
			}
		}
	}

	bool bMerge = false;
	if (idx2 >= 0) {
		bMerge = mergeEarRegions(vecContour, idx1, idx2);

	}


	if (idx1 >= 0) {
		Scalar m = cv::mean(vecContour[idx1]);
		pt.x = m[0] + x;
		pt.y = m[1] + y;
		cv::drawContours(mEarDest, vecContour, idx1, cv::Scalar(255), CV_FILLED, 8);

		if (bMerge) {
			Scalar m = cv::mean(vecContour[idx2]);
			pt.x = (m[0] + x + pt.x) /2;
			pt.y = (m[1] + y + pt.y)/2;
			cv::drawContours(mEarDest, vecContour, idx2, cv::Scalar(255), CV_FILLED, 8);
		}
	}
	return idx1;
}

bool CRat::checkEarRegion(Mat &mROI, vector<vector<cv::Point> >& vecContour, int idx1, int k)
{
	Mat  mRegion, mDilate, mRegionBorderMask;

	// for region idx1
	mRegion = Mat::zeros(mROI.size(), CV_8UC1);
	cv::drawContours(mRegion, vecContour, idx1, cv::Scalar(255), CV_FILLED, 8);

	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	cv::dilate(mRegion, mDilate, element);
	mRegionBorderMask = mDilate - mRegion;
	Scalar border1 = cv::mean(mROI, mRegionBorderMask);

	// for region k
	mRegion = Mat::zeros(mROI.size(), CV_8UC1);
	cv::drawContours(mRegion, vecContour, k, cv::Scalar(255), CV_FILLED, 8);
	cv::dilate(mRegion, mDilate, element);
	mRegionBorderMask = mDilate - mRegion;
	Scalar border2 = cv::mean(mROI, mRegionBorderMask);
	double diff = border1[0] - border2[0];

	//gpOutput->ShowMessage("border1 %.2f, border2 %.2f, diff %.2f\n", border1[0], border2[0], diff);
	//imshow("border", mRegionBorderMask);
	//waitKey(0);

	if (diff > 20)	return false;
	else return true;
}
bool CRat::mergeEarRegions(vector<vector<cv::Point> >& vecContour, int idx1, int idx2)
{
	bool bMerge = false;

	// find nearest dist between two regions
	float nearestTh = 10 * 10;
//	int  idxEar2 = -1;
	vector<cv::Point>& earPts = vecContour[idx1];
	vector<cv::Point>& contourPts = vecContour[idx2];
	float nearestDist = 9999;
	for (int k = 0; k<earPts.size(); k++) {
		Point ptK = earPts[k];
		for (int n = 0; n<contourPts.size(); n++) {
			Point ptN = contourPts[n];
			float dist = (ptK.x - ptN.x)*(ptK.x - ptN.x) + (ptK.y - ptN.y)*(ptK.y - ptN.y);
			if (dist < nearestDist) nearestDist = dist;
		}
	}
	if (nearestDist < nearestTh) {
		nearestTh = nearestDist;
//		idxEar2 = i;
		bMerge = true;
	}

	return bMerge;
} 

void CRat::removeUnlikeBendingPts(vector<int> &vecBendingIdx, vector<float>& curvature, int k)
{
	bool  bFlat;
	int startFlat = -1, endFlat = -1;
	int n = curvature.size();
	for(int i=0; i<n; i++) {
		bFlat = false;
		if(curvature[i] >=179.) {
			bFlat = true;
			for(int j=0; j<10; j++) {
				if(curvature[(i+j)%n] <179) {
					bFlat = false;
					break;
				}
			}
			if(bFlat) {
				startFlat = i;
				break;
			}
		}
	}
	if(startFlat >0) {
		for(int i=startFlat; i<n; i++) {
			if(curvature[i] <179) {
				endFlat = i;
				break;
			}
		}
	}
	if(startFlat <0)  return;
//	int distTh =200;
//	int bb = vecBendingIdx.size();
	//for(int i=0; i<vecBendingIdx.size(); i++) {
	//	int idx = vecBendingIdx[i];
	//	int dist1 = abs(startFlat-idx);
	//	int dist2 = abs(endFlat-idx);
	//	if(dist1 > n/2) dist1 = (dist
	//	if(dist1 <distTh || dist2 <distTh) {
	//		vecBendingIdx.erase (vecBendingIdx.begin()+i);
	//		gpOutput->ShowMessage("remove %d", idx);
	//		i--;
	//	}
	//}
	MainFrame:: myMsgOutput("%d flat %d %d\n",n, startFlat , endFlat);
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
