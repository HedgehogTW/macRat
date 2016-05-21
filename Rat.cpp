
#include <wx/dir.h>
#include <wx/utils.h> 

#include "MainFrame.h"
#include "Rat.h"
#include "KDE.h"

//#include "itkOpenCVImageBridge.h"
//#include "itkCastImageFilter.h"
//#include "itkRescaleIntensityImageFilter.h"
//#include "itkLaplacianRecursiveGaussianImageFilter.h"

//#include "MyContour.h"
#include "MyUtil.h"
#include <gsl/gsl_fit.h>
#include "BendPoint.h"

#define PDF_SIZE    50
#define EYE_OFFSET_RATIO  0.9
#define EYE_OFFSET_BIG_RATIO  1.9

using namespace cv;


Gnuplot gPlotL("lines");
Gnuplot gPlotR("lines");
Gnuplot gPlotP("lines");

CRat::CRat()
{
	
//	m_offsetEar = Point(40, 40);

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
    
//	m_BigRedPdf = -1;
//    m_BigRedGray = -1;    
}

int CRat::readData(wxString inputPath)
{
	clearData();
	
	m_strSrcPath = inputPath;	
	
	wxString fileSpec = _T("*.bmp");
	wxArrayString  	files;
	wxDir::GetAllFiles(inputPath, &files,  fileSpec, wxDIR_FILES );
    for(unsigned int i=0; i<files.size(); i++ ) {
 		wxFileName fileName = files[i];
		wxString  fName = fileName.GetName();
       int pos = fName.Find('_'); 
       if(pos==0 || pos ==wxNOT_FOUND ) {
            files.RemoveAt(i);
            i--;
            continue;
        }
    }
    
    for(unsigned int i=0; i<files.size(); i++ ) {
        wxFileName fileName = files[i];
        wxString  fName = fileName.GetName();
        wxString  fileNum = fName.AfterLast('_');
        if(fileNum.IsEmpty()) continue;
        if(fileNum.Len() >=3) continue;
        long  num;
        if(fileNum.ToLong(&num)) {
            wxString strNum;
            strNum.Printf("%03d", num);
            wxString  fileNameBefore = files[i].BeforeLast('_');   
            wxString  strExt = files[i].AfterLast('.');   
            wxString  newfileName = fileNameBefore +"_"+ strNum +"."+ strExt;
            //MainFrame::myMsgOutput(newfileName+ "\n");
            if(!wxRenameFile(files[i], newfileName)) {
                wxString strMsg = "rename error: " + files[i];
                wxLogMessage(strMsg); 
                break;
            }
       }
    }   
    wxArrayString  newFiles;
	wxDir::GetAllFiles(inputPath, &newFiles, fileSpec, wxDIR_FILES );
	//newFiles.Sort();
    
	cv::Mat	 cvMat;
	m_vecMat.clear();
	m_vFilenames.clear();
	for(unsigned int i=0; i<newFiles.size(); i++ ) {
 		wxFileName fileName = newFiles[i];
		wxString  fName = fileName.GetName();
       int pos = fName.Find('_'); 
       if(pos==0 || pos ==wxNOT_FOUND ) continue;
        
		//MainFrame::myMsgOutput(newFiles[i]+ "\n");
		std::string strStd = newFiles[i].ToStdString();
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

void CRat::cropImage(bool& bCutTop, int nCageLine)
{
	vector <Mat> vecData;
	vector <string> vFilenames;
	int w = m_vecMat[0].cols;
	int h = m_vecMat[0].rows;
	
	m_nCageLine = nCageLine;
	
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
void CRat::DC_removal(int nLED, vector <float>& vecSignal)
{
	int n = vecSignal.size();
	
	if(n<=nLED )  {
		wxLogMessage("n <= nLED");
		return;
	}
	
	float mean = 0;
	
	for(int i=0; i<nLED; i++) 	mean += vecSignal[i];
	mean /= nLED;
	for(int i=0; i<n; i++)  vecSignal[i] -= mean;	
}
float CRat::Notch_removal(vector <float>& vecSignal, int refFrame)
{
    vecSignal[refFrame] = (vecSignal[refFrame-1] + vecSignal[refFrame+1]) /2.0;
	return vecSignal[refFrame];
}
int CRat::isRedSignificant(vector<float>& vecRed, vector<float>& vecCyan)
{
    auto minMaxRed = std::minmax_element (vecRed.begin(), vecRed.end());
    float distRed = *minMaxRed.second - *minMaxRed.first;
    
    auto minMaxCyan = std::minmax_element (vecCyan.begin(), vecCyan.end());
    float distCyan = *minMaxCyan.second - *minMaxCyan.first;   
    
    int bBigRed;
    if(distRed > distCyan ) bBigRed = 1;
    else bBigRed = 0;   

    return bBigRed;
}

cv::Point CRat::findMostSignificantPt(deque<cv::Point>&  dqBellyPts1, int nLed2, long szROIBelly)
{

	int n = nLed2 -5;
	int ref = 50;	
	Point offsetBelly = Point(szROIBelly/2, szROIBelly/2);	
	
	Mat mSrc1, mSrc2, mDiff;
	vector<Mat> vmDiff(n);
	
	m_vecMat[ref].convertTo(mSrc1, CV_16S);;
	
	for(int i=0; i<n; i++) {
		double errSumL, errSumR;
		m_vecMat[i].convertTo(mSrc2, CV_16S);
		cv::absdiff(mSrc1, mSrc2, vmDiff[i]);
	}
	
	
	int h = m_vecMat[0].rows;
	int w = m_vecMat[0].cols;
	int nPts = dqBellyPts1.size();
	vector<float>  bellySeries(n);
	int idxMaxAmp;
	float maxAmp = -999;
	for(int k=0; k<nPts; k++) {
		Point pt1 = Point(dqBellyPts1[k] - offsetBelly);
		Point pt2 = Point(dqBellyPts1[k] + offsetBelly);
		
		if(pt1.x <0) pt1.x = 0;
		if(pt1.y <0) pt1.y = 0;
		if(pt2.x >= w) pt2.x = w-1;
		if(pt2.y >= h) pt2.y = h-1;	
		Rect rect = Rect(pt1, pt2);
		 // = vecBellyGray[k];
		//bellySeries.resize(n);
		for(int i=0; i<n; i++) {
			double errSum = avgROI(vmDiff[i], rect);
			bellySeries[i] = errSum;
		}
		Notch_removal(bellySeries, ref);
		auto minMaxRed = std::minmax_element (bellySeries.begin(), bellySeries.end());
		float amplitude = *minMaxRed.second - *minMaxRed.first;
		if(amplitude > maxAmp) {
			maxAmp = amplitude;
			idxMaxAmp = k;
		}
	}
    
	return dqBellyPts1[idxMaxAmp];
}

void CRat::computeEyeMaskCenter(Point& ptNewMaskCenter, bool bBigHead)
{
	Point ptEyeCenter;
	Point ptEarCenter;
	Point ptEarEyeCen;
	
	ptEyeCenter = (m_ptEyeL + m_ptEyeR)*0.5;
	ptEarCenter = (m_ptEarL + m_ptEarR)*0.5;
	ptEarEyeCen = (ptEyeCenter + ptEarCenter)*0.5;
	
	double m = (double)(ptEarCenter.y - ptEyeCenter.y)/(ptEarCenter.x - ptEyeCenter.x);
	
	cv::Point u, v, pt;
	double angle;
	pt = Point(ptEyeCenter.x, ptEarCenter.y);
	// for ear
	u = ptEyeCenter - ptEarCenter;
	v = pt - ptEarCenter;
	angle = asin(u.cross(v) / (norm(u)*norm(v)));
	
//	MainFrame::myMsgOutput("angle %f\n", angle *180/M_PI);

	double dist = sqrt((ptEarCenter.x - ptEyeCenter.x)*(ptEarCenter.x - ptEyeCenter.x) + 
				(ptEarCenter.y - ptEyeCenter.y)*(ptEarCenter.y - ptEyeCenter.y));
	dist /=4. ;
	double x, y;
	x = fabs(dist*cos(angle));
	y = fabs(dist*sin(angle));
	
	if(ptEyeCenter.x < ptEarCenter.x) {
		if(!bBigHead)
			m_ptHead.x = ptEyeCenter.x-x;
		else
			m_ptHead.x = ptEyeCenter.x+x;
	}else {
		if(!bBigHead)
			m_ptHead.x = ptEyeCenter.x+x;
		else
			m_ptHead.x = ptEyeCenter.x-x;
	}
	if(ptEyeCenter.y < ptEarCenter.y) {
		if(!bBigHead)
			m_ptHead.y = ptEyeCenter.y-y;
		else
			m_ptHead.y = ptEyeCenter.y+y;
	}else {
		if(!bBigHead)
			m_ptHead.y = ptEyeCenter.y+y;
		else
			m_ptHead.y = ptEyeCenter.y-y;
	}	
}
void CRat::recomputeHeadROI()
{
	int h = m_vecMat[0].rows;
	int w = m_vecMat[0].cols;
	
	Point pt1, pt2;	
	pt1 = Point(m_ptHead-m_offsetEye);
	pt2 = Point(m_ptHead+m_offsetEye);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectHead = Rect(pt1, pt2);	
	
	Mat mSrc = Mat::zeros(m_vecMat[0].size(), CV_8UC1);
	Mat mROI(mSrc, m_rectHead);
	mROI = 255;
	
	Mat mROIEarL(mSrc, m_rectEarL);
	mROIEarL = Scalar(0);
	Mat mROIEarR(mSrc, m_rectEarR);
	mROIEarR = Scalar(0);	
	
	Mat mHori, mVert;
	cv::reduce(mSrc, mHori, 0, CV_REDUCE_SUM, CV_32F);
	cv::reduce(mSrc, mVert, 1, CV_REDUCE_SUM, CV_32F);
	mHori /= 255;
	mVert /= 255;
	
	double headW, minV, headH;
	minMaxLoc(mVert, &minV, &headW);
	minMaxLoc(mHori, &minV, &headH);
	Rect rectHead1 = m_rectHead;
	if(mHori.at<float>(0, m_rectHead.x) <m_rectHead.height && 
		mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1) <m_rectHead.height) {
			if(mVert.at<float>(m_rectHead.y, 0) < m_rectHead.width) {
				int hL = m_rectHead.height - mHori.at<float>(0, m_rectHead.x);
				int hR = m_rectHead.height - mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1);
				if(hL < hR)  rectHead1 = Rect(Point(pt1.x, pt1.y+hR), pt2);
				else rectHead1 = Rect(Point(pt1.x, pt1.y+hL), pt2);
				MainFrame::myMsgOutput("cut top\n");
			}else{
				int hL = m_rectHead.height - mHori.at<float>(0, m_rectHead.x);
				int hR = m_rectHead.height - mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1);
				if(hL < hR)  rectHead1 = Rect(pt1, Point(pt2.x, pt2.y-hR));
				else rectHead1 = Rect(pt1, Point(pt2.x, pt2.y-hL));				
				MainFrame::myMsgOutput("cut bottom\n");		
			}
	}else if(mVert.at<float>(m_rectHead.y, 0) <m_rectHead.width && 
		mVert.at<float>(m_rectHead.y+ m_rectHead.height-1, 0) <m_rectHead.width) {
			if(mHori.at<float>(0, m_rectHead.x) < m_rectHead.height) {
				int wT = m_rectHead.width - mVert.at<float>(m_rectHead.y, 0);
				int wB = m_rectHead.width - mVert.at<float>(m_rectHead.y+ m_rectHead.height-1);
				if(wT < wB)  rectHead1 = Rect(Point(pt1.x+wB, pt1.y), pt2);
				else rectHead1 = Rect(Point(pt1.x+wT, pt1.y), pt2);
				MainFrame::myMsgOutput("cut left\n");
			}else{
				int wT = m_rectHead.width - mVert.at<float>(m_rectHead.y, 0);
				int wB = m_rectHead.width - mVert.at<float>(m_rectHead.y+ m_rectHead.height-1, 0);
				if(wT < wB)  rectHead1 = Rect(pt1, Point(pt2.x-wB, pt2.y));
				else rectHead1 = Rect(pt1, Point(pt2.x-wT, pt2.y));			
				MainFrame::myMsgOutput("cut right\n");				
			}
	}else if(mHori.at<float>(0, m_rectHead.x) <m_rectHead.height &&
			  mVert.at<float>(m_rectHead.y, 0) <m_rectHead.width) {
			int w = m_rectHead.width - mVert.at<float>(m_rectHead.y, 0);
			int h = m_rectHead.height - mHori.at<float>(0, m_rectHead.x);
			if(w < h) rectHead1 = Rect(Point(pt1.x+w, pt1.y), pt2);	// cut left
			else rectHead1 = Rect(Point(pt1.x, pt1.y+h), pt2);	// cut top
			MainFrame::myMsgOutput("cut top-left\n");
		
	}else if(mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1) <m_rectHead.height &&
			  mVert.at<float>(m_rectHead.y, 0) <m_rectHead.width) {
			int w = m_rectHead.width - mVert.at<float>(m_rectHead.y, 0);
			int h = m_rectHead.height - mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1);
			if(w < h) rectHead1 = Rect(pt1, Point(pt2.x-w, pt2.y));	// cut right
			else rectHead1 = Rect(Point(pt1.x, pt1.y+h), pt2);	// cut top				  
			MainFrame::myMsgOutput("cut top-right\n");
		
	}else if(mHori.at<float>(0, m_rectHead.x) <m_rectHead.height &&
			  mVert.at<float>(m_rectHead.y+ m_rectHead.height-1, 0) <m_rectHead.width) {
			int w = m_rectHead.width - mVert.at<float>(m_rectHead.y+ m_rectHead.height-1, 0);
			int h = m_rectHead.height - mHori.at<float>(0, m_rectHead.x);
			if(w < h) rectHead1 = Rect(Point(pt1.x+w, pt1.y), pt2);	// cut left
			else rectHead1 = Rect(pt1, Point(pt2.x, pt2.y-h));	// cut bottom				  
			MainFrame::myMsgOutput("cut bottom-left\n");
		
	}else if(mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1) <m_rectHead.height &&
			  mVert.at<float>(m_rectHead.y+ m_rectHead.height-1, 0) <m_rectHead.width) {
			int w = m_rectHead.width - mVert.at<float>(m_rectHead.y+ m_rectHead.height-1, 0);
			int h = m_rectHead.height - mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1);
			if(w < h) rectHead1 = Rect(pt1, Point(pt2.x-w, pt2.y));	// cut right
			else rectHead1 = Rect(pt1, Point(pt2.x, pt2.y-h));	// cut bottom						  
			MainFrame::myMsgOutput("cut bottom-right\n");
	}
	m_rectHead = rectHead1;
//	MainFrame::myMsgOutput("m_rectHead w %d, headW %f\n", m_rectHead.width, headW);
//	MainFrame::myMsgOutput("m_rectHead h %d, headH %f\n", m_rectHead.height, headH);
//	MainFrame::myMsgOutput("mVert %f, %f\n", mVert.at<float>(0, m_rectHead.y), mVert.at<float>(0, m_rectHead.y+ m_rectHead.height-1));
//	MainFrame::myMsgOutput("mHori %f, %f\n", mHori.at<float>(0, m_rectHead.x), mHori.at<float>(0, m_rectHead.x+ m_rectHead.width-1));
}
void CRat::computeROIRect()
{
	int h = m_vecMat[0].rows;
	int w = m_vecMat[0].cols;
	
	Point pt1, pt2;
	pt1 = Point(m_ptEarL-m_offsetEar);
	pt2 = Point(m_ptEarL+m_offsetEar);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectEarL = Rect(pt1, pt2);
	////////////////////////////////////////////////
	
	pt1 = Point(m_ptEarR-m_offsetEar);
	pt2 = Point(m_ptEarR+m_offsetEar);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectEarR = Rect(pt1, pt2);	
	
	////////////////////////////////////////////////
	pt1 = Point(m_ptBelly-m_offsetAPB);
	pt2 = Point(m_ptBelly+m_offsetAPB);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectBelly = Rect(pt1, pt2);	
	////////////////////////////////////////////////
/*
	
	pt1 = Point(m_ptRed-m_offsetAPB);
	pt2 = Point(m_ptRed+m_offsetAPB);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectRed = Rect(pt1, pt2);	
	////////////////////////////////////////////////
	
	pt1 = Point(m_ptCyan-m_offsetAPB);
	pt2 = Point(m_ptCyan+m_offsetAPB);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectCyan = Rect(pt1, pt2);	
	////////////////////////////////////////////////
	*/
	pt1 = Point(m_ptHead-m_offsetEye);
	pt2 = Point(m_ptHead+m_offsetEye);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectHead = Rect(pt1, pt2);	
}

bool CRat::genReferenceFrameSignal(Point& ptBelly, int& nLED2)
{
	m_ptBelly = ptBelly;
//	m_ptCyan = ptCyan;

	/////////////////////////////////////////////////////////////optical flow
	MyConfigData  configData;
	MainFrame::m_pThis->getConfigData(configData);
	
	bool bNewOpFlowV1;
	int  newReferFrame;
	int  newFrameSteps;
	int frameStep = configData.m_frameStep;	
	double threshold = configData.m_threshold;
	bool bLEDLine = configData.m_bLED;
	bool bBigHead = configData.m_bBigHead;
	bool bVerLine = configData.m_bVerLine;

	bool bEyeMove = configData.m_bEyeMove;
	bool bGrayDiff = configData.m_bGrayDiff;
	bool bBelly = configData.m_bBelly;
	bool bEar = configData.m_bEar;
	bool bOpticalPDF = configData.m_bOpticalPDF;
	bool bOpFlowV1 = configData.m_bOpFlowV1;
	bool bSaveFile = configData.m_bSaveFile;

	double verLine = configData.m_verLine;
	double ymin = configData.m_ymin;
	double ymax = configData.m_ymax;
	m_ROIEar = configData.m_szROIEar;
	m_ROIBelly = configData.m_szROIBelly;
	long referFrame = configData.m_referFrame;
	double gainHead = configData.m_gainHead;
	double gainBelly = configData.m_gainBelly;
	double xSD = configData.m_xSD;
	int	refSignal = configData.m_refSignal;
    bool bSaveDiffSignal;
	
	bool bUserLED2;
	if(nLED2 >0)  bUserLED2 = true;
	else bUserLED2 = false;
	
	if(bUserLED2 && nLED2 > 0) {
		m_nLED2 = nLED2-1;
		MainFrame::myMsgOutput("User-specified LED2 %d (0-based)\n", m_nLED2);
	}

	/////////////////////////// for computing ROI rect

	m_offsetAPB = Point(m_ROIBelly/2, m_ROIBelly/2);
	
	//computeROIRect();
	int h = m_vecMat[0].rows;
	int w = m_vecMat[0].cols;
	
	Point pt1 = Point(m_ptBelly-m_offsetAPB);
	Point pt2 = Point(m_ptBelly+m_offsetAPB);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectBelly = Rect(pt1, pt2);	
	////////////////////////////////////////////////
/*	
	pt1 = Point(m_ptCyan-m_offsetAPB);
	pt2 = Point(m_ptCyan+m_offsetAPB);
	
	if(pt1.x <0) pt1.x = 0;
	if(pt1.y <0) pt1.y = 0;
	if(pt2.x >= w) pt2.x = w-1;
	if(pt2.y >= h) pt2.y = h-1;	
	m_rectCyan = Rect(pt1, pt2);	
	////////////////////////////////////////////////
	*/
	/////////////////////////////////// just for findReferenceFrame

	
//	int ref = 50;
//	graylevelDiff(ref, m_rectBelly, vecRedGrayDiff);
//	Notch_removal(vecRedGrayDiff, ref);	
	newReferFrame = findReferenceFrame(m_rectBelly);
	
	MainFrame::myMsgOutput("Reference frame %d\n", newReferFrame);
	if(newReferFrame <0) {
		wxLogMessage("cannot find reference frame");
		return false;
	}
      
//////////////////////////////////////////////////////////////////////	
	wxFileName fileName = m_strSrcPath;
	wxString title =fileName.GetName();
	int lenTitle = strlen(title);
	if(lenTitle <=3) {
		wxUniChar sep = fileName.GetPathSeparator();
		int len = m_strSrcPath.Len();
		int p1 = m_strSrcPath.find_last_of(sep, len -lenTitle-1);
		int p2 = m_strSrcPath.find_last_of(sep, p1-1);
		int p3 = m_strSrcPath.find_last_of(sep, p2-1);
		title = m_strSrcPath.Right(len-p3);
		title.Replace("\\", "/");
//		MainFrame:: myMsgOutput("strUpperPath "+ m_strSrcPath.Right(len-p3)+ "\n" );
	}
	
	wxUniChar sep = fileName.GetPathSeparator();
	wxString  strParentPath =  m_strSrcPath.BeforeLast(sep);	
	int len = title.Len();
	title = title.Right(len-1);
	title.Replace("/", "_");
	
	vector <float>  vecRedGrayDiff;
	vector <float>  vecBellyGray;	
	
	int szSeries = m_nLED2+1; //vecRedGrayDiff.size();
	vecBellyGray.resize(szSeries);
	
	int  start = 110; // 24
	Mat  mRef(szSeries, start*2+1, CV_32F);  // 49
	Mat  mRef1B(start*2+1, 2, CV_32F);   // 49
	//Mat  mRef(m_nLED2+1, m_nLED2, CV_32F);
	
	for(int i=-start; i<=start; i++) {  // -24 .. 24
		int ref = newReferFrame + i;
	//for(int i=0; i<m_nLED2; i++) {
		//int ref = i;
		graylevelDiff(ref, m_rectBelly, vecRedGrayDiff, szSeries, true);
	
		std::copy ( vecRedGrayDiff.begin(), vecRedGrayDiff.end(), vecBellyGray.begin() );
		
		DC_removal(m_nLED2, vecBellyGray);			
		Notch_removal(vecBellyGray, ref);
	
		mRef.at<float>(0, i+start) = ref;
		mRef1B.at<float>(i+start, 0) = ref;
		mRef1B.at<float>(i+start, 1) = vecBellyGray[ref-1];
		//mRef.at<float>(0, i) = ref;
		for(int k=1; k<szSeries; k++) {
			mRef.at<float>(k, i+start) = vecBellyGray[k-1];
			//mRef.at<float>(k, i) = vecBellyGray[k-1];
		}

	}
	wxString  diffSignalName;
	diffSignalName << title << "_Diff_Ref_" << newReferFrame << ".csv";
	wxFileName newFullMarkerName(strParentPath, diffSignalName); 
	_OutputMat(mRef, newFullMarkerName.GetFullPath());
	MainFrame::myMsgOutput("save: "+diffSignalName+"\n");	
	
	wxString  name1B;
	name1B << title << "_Diff_Ref_1B" << ".csv";
	wxFileName newName(strParentPath, name1B); 
	_OutputMat(mRef1B, newName.GetFullPath());
	
	return true;
}

bool CRat::process(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR, Point& ptBelly, int& nLED2)
{
	m_ptEyeL = ptEyeL;
	m_ptEyeR = ptEyeR;
	
	m_ptEarL = ptEarL;
	m_ptEarR = ptEarR;
    
	m_ptBelly = ptBelly;
//	m_ptCyan = ptCyan;
	
	m_vecEyeL.clear();
	m_vecEyeR.clear();

	/////////////////////////////////////////////////////////////optical flow
	MyConfigData  configData;
	MainFrame::m_pThis->getConfigData(configData);
	
	int  newReferFrame;
//	int  newFrameSteps;
	int frameStep = configData.m_frameStep;	
	double threshold = configData.m_threshold;
	bool bLEDLine = configData.m_bLED;
	bool bBigHead = configData.m_bBigHead;
	bool bVerLine = configData.m_bVerLine;

	bool bEyeMove = configData.m_bEyeMove;
	bool bGrayDiff = configData.m_bGrayDiff;
	bool bBelly = configData.m_bBelly;
	bool bEar = configData.m_bEar;
	bool bOpticalPDF = configData.m_bOpticalPDF;
	bool bOpFlowV1 = configData.m_bOpFlowV1;
	bool bSaveFile = configData.m_bSaveFile;
	bool bSaveSignalPlot = configData.m_bSaveSignalPlot;
	bool bShowPeaks = configData.m_bShowPeaks;
	bool bShowAxisLabel = configData.m_bShowAxisLabel;

	double verLine = configData.m_verLine;
	double ymin = configData.m_ymin;
	double ymax = configData.m_ymax;
	double ysnd = configData.m_ysnd;
	
	double intvymin = configData.m_intvymin;
	double intvymax = configData.m_intvymax;
	double intvysnd = configData.m_intvysnd;
	
	double  smoothEar = configData.m_smoothEar;
    double  smoothBelly = configData.m_smoothBelly;
	
	m_ROIEar = configData.m_szROIEar;
	m_ROIBelly = configData.m_szROIBelly;
	long referFrame = configData.m_referFrame;
	double gainHead = configData.m_gainHead;
	double gainBelly = configData.m_gainBelly;
	double xSD = configData.m_xSD;
	int	refSignal = configData.m_refSignal;
	
	bool bUserLED2;
	if(nLED2 >0)  {
		bUserLED2 = true;
		m_nLED2 = nLED2;
	}
	else bUserLED2 = false;
	MainFrame::myMsgOutput("CRat::process::m_nLED2 %d (0-based)\n", m_nLED2);

	
	m_bShowEye = bEyeMove;
	m_bShowEar = bEar;
	m_bShowBelly = bBelly;

	/////////////////////////// for computing ROI rect
	m_offsetEar = Point(m_ROIEar/2, m_ROIEar/2);
	m_offsetAPB = Point(m_ROIBelly/2, m_ROIBelly/2);
	
	double dist = sqrt((m_ptEyeL.x - m_ptEyeR.x)*(m_ptEyeL.x - m_ptEyeR.x) + 
				(m_ptEyeL.y - m_ptEyeR.y)*(m_ptEyeL.y - m_ptEyeR.y));	
	
	if(bBigHead)  
		m_offsetEye = Point(dist*EYE_OFFSET_BIG_RATIO, dist*EYE_OFFSET_BIG_RATIO);	
	else
		m_offsetEye =  Point(dist*EYE_OFFSET_RATIO, dist*EYE_OFFSET_RATIO);	
		
	computeEyeMaskCenter(m_ptHead, bBigHead);
	computeROIRect();
	if(!bBigHead) recomputeHeadROI();
	
	/////////////////////////////////// just for findReferenceFrame

	vector <float>  vecBellyGray;	
			
    if(referFrame<=0) {
		if(refSignal==0) {
			newReferFrame = findReferenceFrame(m_rectBelly);
		}else if (refSignal==1)
			newReferFrame = findReferenceFrame(m_rectHead);
		else if(refSignal==2)
			newReferFrame = findReferenceFrame(m_rectEarL);
    }else
        newReferFrame = referFrame;
        
	MainFrame::myMsgOutput("Reference frame %d, based on signal %d\n", newReferFrame, refSignal);
	if(newReferFrame <0) {
		wxLogMessage("cannot find reference frame");
		return false;
	}
	
//	if(m_bShowEye) {
//		findEyeCenter(ptEyeL, m_vecEyeL, m_vecEyeLMove, newReferFrame);
//		findEyeCenter(ptEyeR, m_vecEyeR, m_vecEyeRMove, newReferFrame);
//	}else {
		m_vecEyeR.resize(m_nSlices);
		m_vecEyeL.resize(m_nSlices);
		for(int i=0; i<m_nSlices;i++) {
			m_vecEyeR[i] = ptEyeR;
			m_vecEyeL[i] = ptEyeL;
		}
//	}
      
//////////////////////////////////////////////////////////////////////	
	wxFileName fileName = m_strSrcPath;
	wxString title =fileName.GetName();
	int lenTitle = strlen(title);
	if(lenTitle <=3) {
		wxUniChar sep = fileName.GetPathSeparator();
		int len = m_strSrcPath.Len();
		int p1 = m_strSrcPath.find_last_of(sep, len -lenTitle-1);
		int p2 = m_strSrcPath.find_last_of(sep, p1-1);
		int p3 = m_strSrcPath.find_last_of(sep, p2-1);
		title = m_strSrcPath.Right(len-p3);
		title.Replace("\\", "/");
//		MainFrame:: myMsgOutput("strUpperPath "+ m_strSrcPath.Right(len-p3)+ "\n" );
	}
    
	clock_t start, finish;
	double  duration;
	start = clock();
	
	wxBeginBusyCursor();
	
	vector <float>  vecEyeFlowPdfL;	
	vector <float>  vecLEarFlowPdf;
	vector <float>  vecREarFlowPdf;
	vector <float>  vecRedFlowPdf;
//	vector <float>  vecCyanFlowPdf;
	vector <float>  vecBellyPdf;

	
	int szVecFlow = m_vecFlow.size();
    if(bOpticalPDF ) {
   //     if(newFrameSteps != frameStep || szVecFlow ==0 || m_referFrame != newReferFrame || bNewOpFlowV1 != bOpFlowV1) {
            if(bOpFlowV1)
				opticalFlow_v1(frameStep, newReferFrame);
			else
				opticalFlow_v2(newReferFrame);
 //       }
    }
//	bOpFlowV1 = bNewOpFlowV1;
//	frameStep = newFrameSteps;
	m_referFrame = newReferFrame;
    
	MainFrame:: myMsgOutput("y range [%.2f, %.2f], Ear ROI %d, APB ROI %d, m_nLED2 %d, bSave %d, smooth %.2f %.2f\n", 
			ymin, ymax, m_ROIEar, m_ROIBelly, m_nLED2, bSaveFile, smoothEar, smoothBelly);	
	MainFrame:: myMsgOutput("PDF threshold %f, frame steps %d, bOpFlowV1 %d, headGain %.2f, bellyGain %.2f\n", 
			threshold, frameStep, bOpFlowV1, gainHead, gainBelly);
			
	if(m_bShowBelly)
		MainFrame:: myMsgOutput("Belly Point (%d, %d)\n", ptBelly.x, ptBelly.y);
	
	vector <float>  vecLEarGrayDiff;
	vector <float>  vecREarGrayDiff;
	vector <float>  vecRedGrayDiff;
//	vector<float> smoothL;	
//	vector<float> smoothR;
        
//	int maxPointL, maxPointR;
//	maxPointL = maxPointR = -1;
	if(bGrayDiff ) {
		if(m_bShowEar) {
			graylevelDiff(m_referFrame, m_rectEarL, m_rectEarR, vecLEarGrayDiff, vecREarGrayDiff);
//			smoothData(vecLEarGrayDiff, smoothL, 2);	
//			smoothData(vecREarGrayDiff, smoothR, 2);	
			
//			maxPointL = findMaxMotionPoint(smoothL);
//			maxPointR = findMaxMotionPoint(smoothR);	

			DC_removal(m_nLED2, vecLEarGrayDiff);
			DC_removal(m_nLED2, vecREarGrayDiff);
			
			Notch_removal(vecLEarGrayDiff, m_referFrame);
			Notch_removal(vecREarGrayDiff, m_referFrame);
		}
		if(m_bShowBelly) {
            //if(bSaveDiffSignal)  ref = referFrame;
            //else ref = m_referFrame;
			
			graylevelDiff(newReferFrame, m_rectBelly, vecRedGrayDiff);
			
			vecBellyGray.resize(vecRedGrayDiff.size());
			std::copy ( vecRedGrayDiff.begin(), vecRedGrayDiff.end(), vecBellyGray.begin() );
			
			vecRedGrayDiff.clear();
						
			DC_removal(m_nLED2, vecBellyGray);			
			Notch_removal(vecBellyGray, newReferFrame);
		}
	}
	
	float sdHead; //, sdBelly, sdLEar, sdREar;
	vector <float>  vecBellySmooth;
	vector <float>  vecLEarSmooth;	
	vector <float>  vecREarSmooth;	
	
	vector<Point2f> peakBelly;	
	vector<Point2f> peakLEar;	
	vector<Point2f> peakREar;	
	
	vector<Rect> 	vDrawRect;
	vector<Point> 	vDrawPt;

	vector<Point2f> peakMinMax;
	
	vector<float> 	vPeakBellyDistX, vPeakBellyDistY;
	vector<float> 	vPeakLEarDistX, vPeakLEarDistY;
	vector<float> 	vPeakREarDistX, vPeakREarDistY;
	float meanPeriodBelly, sdPeriodBelly, meanAmpBelly, sdAmpBelly;
	float meanPeriodLEar, sdPeriodLEar, meanAmpLEar, sdAmpLEar;
	float meanPeriodREar, sdPeriodREar, meanAmpREar, sdAmpREar;
	
	if(bOpticalPDF) {
		int sz = m_vecFlow.size();

        if(bSaveFile) {
            m_vmOpPDFMap.resize(sz);
            for(int i=0; i<sz; i++)
                m_vmOpPDFMap[i] = Mat::zeros(m_vecFlow[0].size(), CV_8UC1);
        }
		if(m_bShowEar) {
			opticalMovement(m_rectEarL, vecLEarFlowPdf, m_vmDistEarL, threshold, bOpFlowV1);
			opticalMovement(m_rectEarR, vecREarFlowPdf, m_vmDistEarR, threshold, bOpFlowV1);
			
			DC_removal(m_nLED2, vecLEarFlowPdf);
			DC_removal(m_nLED2, vecREarFlowPdf);
			
			Notch_removal(vecLEarFlowPdf, m_referFrame);
			Notch_removal(vecREarFlowPdf, m_referFrame);
		
			smoothData(vecLEarFlowPdf, vecLEarSmooth, smoothEar);	
			smoothData(vecREarFlowPdf, vecREarSmooth, smoothEar);	
			
			findPeaks(vecLEarFlowPdf, vecLEarSmooth, peakLEar);
			findPeaks(vecREarFlowPdf, vecREarSmooth, peakREar);
			
			peakPeriodAnalysis(peakLEar, vPeakLEarDistX, vPeakLEarDistY, m_nLED2, meanPeriodLEar, sdPeriodLEar);
			peakAmplitudeAnalysis(peakLEar, m_nLED2, meanAmpLEar, sdAmpLEar);

			peakPeriodAnalysis(peakREar, vPeakREarDistX, vPeakREarDistY, m_nLED2, meanPeriodREar, sdPeriodREar);
			peakAmplitudeAnalysis(peakREar, m_nLED2, meanAmpREar, sdAmpREar);
	
			if(bSaveFile) {
                opticalAssignThresholdMap(m_vmDistEarL, threshold, m_rectEarL);
                opticalAssignThresholdMap(m_vmDistEarR, threshold, m_rectEarR);
                vDrawRect.push_back(m_rectEarL);
                vDrawRect.push_back(m_rectEarR);
                vDrawPt.push_back(ptEarL);
                vDrawPt.push_back(ptEarR);
            }
		}
		
		if(m_bShowBelly) {
			opticalMovement(m_rectBelly, vecRedFlowPdf, m_vmDistRed, threshold, bOpFlowV1);
			vecBellyPdf.resize(vecRedFlowPdf.size());

			std::copy ( vecRedFlowPdf.begin(), vecRedFlowPdf.end(), vecBellyPdf.begin() );
			vecRedFlowPdf.clear();
		
			if(gainBelly!=1) {
				for(int i=0; i<sz; i++) {
					vecBellyPdf[i] = vecBellyPdf[i]*gainBelly;
				}
			}
			DC_removal(m_nLED2, vecBellyPdf);
			
			Notch_removal(vecBellyPdf, m_referFrame);
			//sdBelly = computeSD(vecBellyPdf, m_nLED2);
			
			smoothData(vecBellyPdf, vecBellySmooth, smoothBelly);	
			findPeaks(vecBellyPdf, vecBellySmooth, peakBelly);
			peakPeriodAnalysis(peakBelly, vPeakBellyDistX, vPeakBellyDistY, m_nLED2, meanPeriodBelly, sdPeriodBelly);
			peakAmplitudeAnalysis(peakBelly, m_nLED2, meanAmpBelly, sdAmpBelly);
			
			Point2f pt(314, vecBellyPdf[314]);
			peakMinMax.push_back(pt);
			Point2f pt2(336, vecBellyPdf[336]);
			peakMinMax.push_back(pt2);
			
			if(bSaveFile) {
				opticalAssignThresholdMap(m_vmDistRed, threshold, m_rectBelly);
				vDrawRect.push_back(m_rectBelly);
				vDrawPt.push_back(ptBelly);
            }            
		}
		
		if(m_bShowEye) {
			opticalMovement(m_rectHead, vecEyeFlowPdfL, m_vmDistEyeL, threshold, bOpFlowV1);			
			if(gainHead!=1) {
				for(int i=0; i<sz; i++) {
					vecEyeFlowPdfL[i] = vecEyeFlowPdfL[i]*gainHead;
				}
			} 
			
			DC_removal(m_nLED2, vecEyeFlowPdfL);
			
			float baselineL = Notch_removal(vecEyeFlowPdfL, m_referFrame);
			sdHead=computeSD(vecEyeFlowPdfL, m_nLED2);
            if(bSaveFile) {
                opticalAssignThresholdMap(m_vmDistEyeL, threshold, m_rectHead);
                vDrawRect.push_back(m_rectHead);
                vDrawPt.push_back(m_ptHead);                   
            } 
		}
        //////////////////////////////////////save file
        if(bSaveFile) {
			opticalDrawFlowmapWithPDF(vDrawRect, vDrawPt, frameStep);
			opticalSavePlot("scatter", "dots", threshold);
			opticalSavePlot("pdf", "lines", threshold);
        }
	}
	 
	wxEndBusyCursor(); 
	
///////////G N U P L O T//////////////////////////////////////////////////////////////////////////////
	vector<float> 	vxTicks(m_nSlices);
	for(int i=0; i<m_nSlices; i++)
		vxTicks[i] = i- m_nLED2;
	
	int xStart = floor((0-m_nLED2)/25.) * 25;
	int xEnd = ceil((m_nSlices-m_nLED2)/25.) * 25;

	_gnuplotInit(gPlotL, title.ToAscii(), ymin, ymax);
	_gnuplotInit(gPlotR, title.ToAscii(), ymin, ymax);
	_gnuplotInit(gPlotP, title.ToAscii(), intvymin, intvymax);
	
	if(bShowAxisLabel) {
		gPlotL.set_xlabel("Peri-stimulus Frame");
		gPlotL.set_ylabel("Movement index (arb. unit)");	
		gPlotR.set_xlabel("Peri-stimulus Frame");
		gPlotR.set_ylabel("Movement index (arb. unit)");

		gPlotP.set_xlabel("Peri-stimulus Frame");
		gPlotP.set_ylabel("Inter-peak interval (arb. unit)");
	}

	gPlotL.set_legend("left");
	gPlotR.set_legend("left");	
	gPlotP.set_legend("left");	
	gPlotL.cmd("set termoption noenhanced");
	gPlotR.cmd("set termoption noenhanced");
	gPlotP.cmd("set termoption noenhanced");
	
	gPlotL.set_xrange(xStart, xEnd);
	gPlotR.set_xrange(xStart, xEnd);
	gPlotP.set_xrange(xStart, xEnd);
	//gPlotR.cmd("set xtics -300,50,250");

	float highAmp = (ymax-ymin) / 35.;
	float highIntv = (intvymax-intvymin)/35.;
	_gnuplotSoundOnset(gPlotL, m_nLED2, m_nSlices, ysnd, highAmp, 100);
	_gnuplotSoundOnset(gPlotR, m_nLED2, m_nSlices, ysnd, highAmp, 100);
	_gnuplotSoundOnset(gPlotP, m_nLED2, m_nSlices, intvysnd, highIntv, 100);
	
	if(bLEDLine && (m_nLED1>0 || m_nLED2 >0)) {
		_gnuplotLED(gPlotL, m_nLED1, m_nLED2);
		_gnuplotLED(gPlotR, m_nLED1, m_nLED2);
	}
	if(bLEDLine) {
		_gnuplotVerticalLine(gPlotL, m_referFrame);
		_gnuplotVerticalLine(gPlotR, m_referFrame);        
    }
	
	if(bVerLine && verLine >0) {
		_gnuplotVerticalLine(gPlotL, verLine);
		_gnuplotVerticalLine(gPlotR, verLine);		
	}
	if(bGrayDiff) {
		if(m_bShowEar) {
			_gnuplotLine(gPlotL, "LEarGraylevelDiff", vecLEarGrayDiff, "#000000ff", ".");
			_gnuplotLine(gPlotR, "REarGraylevelDiff", vecREarGrayDiff, "#000000ff", ".");
		}
		if(m_bShowBelly) {
            //_OutputVec(vecRedGrayDiff, "_redgray.csv");
			_gnuplotLine(gPlotL, "Abdomen GraylevelDiff", vecBellyGray, "#00008000", ".");
			_gnuplotLine(gPlotR, "Abdomen GraylevelDiff", vecBellyGray, "#00008000", ".");

		}
		if(bEyeMove) {
			_gnuplotLine(gPlotL, "LEyePosMove", m_vecEyeLMove, "#008B0000", ".");
			_gnuplotLine(gPlotR, "REyePosMove", m_vecEyeLMove, "#008B0000", ".");
		
		}		
	}
	
	wxString strLine;
	strLine.Printf("%.1fxSD", xSD);
			
	wxUniChar sep = fileName.GetPathSeparator();
	wxString  strParentPath =  m_strSrcPath.BeforeLast(sep);	
	int len = title.Len();
	title = title.Right(len-1);
	title.Replace("/", "_");
	if(bOpticalPDF) {
		vector <float>  vX, vY;
		if(m_bShowEar) {
			_gnuplotLineXY(gPlotL, "LEar", vxTicks, vecLEarFlowPdf, "#000000ff");
			_gnuplotLineXY(gPlotR, "REar", vxTicks, vecREarFlowPdf, "#000000ff");

			_gnuplotHoriLine(gPlotL, xStart, xEnd, meanAmpLEar + sdAmpLEar*xSD, "#00100800", "..-", strLine);
			_gnuplotHoriLine(gPlotR, xStart, xEnd, meanAmpREar + sdAmpREar*xSD, "#00100800", "..-", strLine);
			_gnuplotHoriLine(gPlotL, xStart, xEnd, meanAmpLEar - sdAmpLEar*xSD, "#00100800",  "..-");
			_gnuplotHoriLine(gPlotR, xStart, xEnd, meanAmpREar - sdAmpREar*xSD, "#00100800",  "..-");
	
			wxString  newMarkerName = title+"_LEar.csv";
			wxFileName newFullMarkerName(strParentPath, newMarkerName);
			_OutputVec(vecLEarFlowPdf, newFullMarkerName.GetFullPath());
			
			newMarkerName =  title+"_REar.csv";
			wxFileName newFullMarkerName1(strParentPath, newMarkerName);
			_OutputVec(vecREarFlowPdf, newFullMarkerName1.GetFullPath());	

			if(bShowPeaks) {
				for(int i=0; i<peakLEar.size(); i++) {
					peakLEar[i].x -= m_nLED2;
					peakREar[i].x -= m_nLED2;
				}
				_gnuplotPoint(gPlotR, peakREar, "#00008f00" );
				_gnuplotPoint(gPlotL, peakLEar, "#00008f00" );
			}
			
			for(int i=0; i<vPeakLEarDistX.size(); i++) 
				vPeakLEarDistX[i] -= m_nLED2;
			for(int i=0; i<vPeakREarDistX.size(); i++) 	
				vPeakREarDistX[i] -= m_nLED2;
					
			_gnuplotSteps(gPlotP, vPeakLEarDistX, vPeakLEarDistY, "#00FF0000", "Inter-peak interval");
			_gnuplotSteps(gPlotP, vPeakREarDistX, vPeakREarDistY, "#00FF0000", "Inter-peak interval");		

			_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriodLEar+xSD*sdPeriodLEar, "#00100800", "..-", strLine);			
			_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriodLEar-xSD*sdPeriodLEar, "#00100800", "..-");

			_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriodREar+xSD*sdPeriodREar, "#00100800", "..-", strLine);			
			_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriodREar-xSD*sdPeriodREar, "#00100800", "..-");		
		}
		if(m_bShowBelly) {
            //_OutputVec(vecRedFlowPdf, "_redw.csv");
			//_gnuplotLine(gPlotL, "Abdomen", vecBellyPdf, "#00008000");
			//_gnuplotLine(gPlotR, "Abdomen", vecBellyPdf, "#00008000");

			
			_gnuplotLineXY(gPlotL, "Abdomen", vxTicks, vecBellyPdf, "#00008000");
			_gnuplotLineXY(gPlotR, "Abdomen", vxTicks, vecBellyPdf, "#00008000");
			

			_gnuplotHoriLine(gPlotL, xStart, xEnd, meanAmpBelly + sdAmpBelly*xSD, "#00100800", "..-", strLine);
			_gnuplotHoriLine(gPlotR, xStart, xEnd, meanAmpBelly + sdAmpBelly*xSD, "#00100800", "..-", strLine);
			_gnuplotHoriLine(gPlotL, xStart, xEnd, meanAmpBelly - sdAmpBelly*xSD, "#00100800",  "..-");
			_gnuplotHoriLine(gPlotR, xStart, xEnd, meanAmpBelly - sdAmpBelly*xSD, "#00100800",  "..-");
	
			wxString  newMarkerName = title+"_Belly.csv";
			wxFileName newFullMarkerName(strParentPath, newMarkerName);			
			

				
			_OutputVec(vecBellyPdf, newFullMarkerName.GetFullPath());
//			_gnuplotPoint(gPlotL, peakMinMax, "#00008f00" );
//			_gnuplotPoint(gPlotR, peakMinMax /*peakBelly*/, "#00008f00" );
			if(bShowPeaks) {
				for(int i=0; i<peakBelly.size(); i++)
					peakBelly[i].x -= m_nLED2;
				_gnuplotPoint(gPlotR, peakBelly, "#00008f00" );
			}
			for(int i=0; i<vPeakBellyDistX.size(); i++)
				vPeakBellyDistX[i] -= m_nLED2;
				
			_gnuplotSteps(gPlotP, vPeakBellyDistX, vPeakBellyDistY, "#00FF0000", "Inter-peak interval");
			//_gnuplotHoriLine(gPlotP, m_nSlices, meanPeriodBelly, "#000000FF");		

			_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriodBelly+xSD*sdPeriodBelly, "#00100800", "..-", strLine);			
			_gnuplotHoriLine(gPlotP, xStart, xEnd, meanPeriodBelly-xSD*sdPeriodBelly, "#00100800", "..-");			
			//_gnuplotVerticalLine(gPlotP, m_nLED2, "#000000FF");
 				
		}
		if(m_bShowEye) {
			vX.clear();
			vY.clear();
			vX.push_back(xStart);
			vX.push_back(xEnd);
			vY.push_back(sdHead);
			vY.push_back(sdHead);	
			_gnuplotLineXY(gPlotL, "", vX, vY, "#008B8800");	
			_gnuplotLineXY(gPlotR, "", vX, vY, "#008B8800");
			
            _gnuplotLineXY(gPlotL, "Head", vxTicks, vecEyeFlowPdfL, "#008B0000");
			_gnuplotLineXY(gPlotR, "Head", vxTicks, vecEyeFlowPdfL, "#008B0000");
            
		
			wxString  newMarkerName = title+"_Head.csv";
			wxFileName newFullMarkerName(strParentPath, newMarkerName);	
			_OutputVec(vecEyeFlowPdfL, newFullMarkerName.GetFullPath()); 
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

float CRat::findMode(vector<float>& inData, float sigma)
{
	float scale = 2;
	auto minMax = std::minmax_element (inData.begin(), inData.end());
	float maxValue = (*minMax.second) * scale;
	float minValue = (*minMax.first) * scale;
	int range = maxValue - minValue +1;
	int n = inData.size();	
	double* pData = new double[range];
	double  *out = new double[range];

	for(int i=0; i<n; i++)  {
		pData[(int)(inData[i]*scale-minValue)]++;
		MainFrame::myMsgOutput("%.2f  ", inData[i]);
	}
	MainFrame::myMsgOutput("\n");
	CKDE::MSKernel kde_kernel = CKDE::Gaussian;
	
	CKDE kde1x(pData, out, range);
	kde1x.KernelDensityEstimation(kde_kernel, sigma);

	int maxIdx;
	double max = -1000;
	for(int i=0; i<range; i++)  {
		MainFrame::myMsgOutput("%.2f  ", out[i]);
		if(out[i] > max) {
			max = out[i];
			maxIdx = i;
		}
	}
	
	float mode = (maxIdx+minValue)/scale;
	return mode;	
}

int CRat::peakAmplitudeAnalysis(vector<Point2f>& peaks, int nLED2, float& mean, float& sd)
{
	vector<float> vPeakAmp;
	int n = peaks.size();
	int led = -1;

	for(int i=0; i<n; i++) {
		if(peaks[i].x <= nLED2)
			vPeakAmp.push_back(peaks[i].y);
		else {
			led = i;
			break;
		}
	}	
	cv::Scalar meanS, stddev;
	cv::meanStdDev(vPeakAmp, meanS, stddev);
	sd = stddev(0);
	mean = meanS(0);
	
	return led;
}

int CRat::peakPeriodAnalysis(vector<Point2f>& peaks, vector<float>& vPeakBellyDistX, vector<float>& vPeakBellyDistY, int nLED2, float& mean, float& sd)
{
	int n = peaks.size();
	int led = -1;

	vPeakBellyDistX.resize(n-1);
	vPeakBellyDistY.resize(n-1);
	for(int i=0; i<n-1; i++) {
		vPeakBellyDistY[i] = peaks[i+1].x - peaks[i].x;
		vPeakBellyDistX[i] = peaks[i].x;
		if(peaks[i+1].x > nLED2 && led<0) led = i;
//		MainFrame::myMsgOutput("peaks[i+1].x %.2f  ", peaks[i+1].x);
	}
//	ledPos = led + (float)(nLED2-peaks[led].x)/(peaks[led+1].x - peaks[led].x);
	
	vector<float> vPeakBefore;
	vPeakBefore.resize(led);
	std::copy ( vPeakBellyDistY.begin(), vPeakBellyDistY.begin()+led, vPeakBefore.begin() );
	
	cv::Scalar meanS, stddev;
	cv::meanStdDev(vPeakBefore, meanS, stddev);
	sd = stddev(0);
	mean = meanS(0);
/*	
	for(int i=0; i<n-1; i++)  {
		MainFrame::myMsgOutput("%.2f  ", vPeakBellyDistY[i]);
		if(i==led-1)  MainFrame::myMsgOutput(" | ");
	}
*/
//	MainFrame::myMsgOutput("Before LED period mean = %.3f, sd = %.3f\n", mean, sd);
//	return ledPos;

	return led;
}
void CRat::findPeaks(vector<float>& inDataOri, vector<float>& inData, vector<Point2f>& peaks)
{
	peaks.clear();
	int n = inData.size();
	/*
	for(int i=3; i<7-3; i++) {
		if(inData[i-3] < inData[i-1] && inData[i-1] < inData[i] &&
			inData[i+3] < inData[i+1] && inData[i+1] < inData[i]) {
				Point2f pt(i, inDataOri[i]);
				peaks.push_back(pt);
		}
	}*/
	for(int i=3; i<n-3; i++) {
		if(/*inData[i-6] < inData[i-5] && inData[i-4] < inData[i-3] &&*/ inData[i-3] < inData[i-2] && inData[i-2] < inData[i-1] && inData[i-1] < inData[i] &&
			/*inData[i+6] < inData[i+5] && inData[i+4] < inData[i+3] && inData[i+3] < inData[i+2] &&*/ inData[i+2] < inData[i+1] && inData[i+1] < inData[i]) {
				float maxV = -999;
				int maxIdx;
				for(int k=-3; k<=3; k++) {
					if(inDataOri[i+k] > maxV) {
						maxV = inDataOri[i+k];
						maxIdx = i+k;
					}
				}
				if(maxIdx<= 3) continue;
				Point2f pt(maxIdx, inDataOri[maxIdx]);
				peaks.push_back(pt);				
		}
	}
	
//	for(int i=0; i<peaks.size(); i++) {
//		MainFrame::myMsgOutput("peaks[%d].x %.2f \n", i, peaks[i].x);
//	}	
}

float CRat::computeSD(vector<float>& vSignal, int nLED2)
{
	vector<float>  data(nLED2);
	copy(vSignal.begin(), vSignal.begin()+nLED2, data.begin());

	cv::Scalar mean, stddev;
	cv::meanStdDev(data, mean, stddev);
	float sd = stddev(0);
	return sd;
}

void CRat::drawOnDestImage(bool bSaveFile)
{
	////////////////////// save result to dest
	m_vecDest.resize(m_nSlices);
/*	
	Point ptL1 (m_ptEarL-m_offsetEar);
	Point ptL2 (m_ptEarL+m_offsetEar);
	Point ptR1 (m_ptEarR-m_offsetEar);
	Point ptR2 (m_ptEarR+m_offsetEar);
	
	Point ptD1 (m_ptRed-m_offsetAPB);
	Point ptD2 (m_ptRed+m_offsetAPB);
	Point ptC1 (m_ptCyan-m_offsetAPB);
	Point ptC2 (m_ptCyan+m_offsetAPB);
	
	Point ptEye1 (m_ptHead-m_offsetEye);
	Point ptEye2 (m_ptHead+m_offsetEye);
*/		
	for (int i = 0; i < m_nSlices; i++)
		cvtColor(m_vecMat[i], m_vecDest[i], CV_GRAY2BGR);

	for (int i = 0; i < m_nSlices; i++)
	{
		Mat mDestColor;
		mDestColor = m_vecDest[i];
		// original ears
        if(m_bShowEar) {
            rectangle(mDestColor, m_rectEarL, Scalar(255,0, 0));
            rectangle(mDestColor, m_rectEarR, Scalar(255,0, 0));
        }
        if(m_bShowBelly) {
 //           if(m_BigRedPdf>0)
                rectangle(mDestColor, m_rectBelly, Scalar(0,128,0));
//            else
//                rectangle(mDestColor, m_rectCyan, Scalar(0,128,0));
        }
		if(m_bShowEye) {
            rectangle(mDestColor, m_rectHead, Scalar(0, 0,255));
			//rectangle(mDestColor, m_rectHead1, Scalar(255, 0,255));
            //rectangle(mDestColor, Rect(ptEyeR1, ptEyeR2), Scalar(255, 0,255));
        }
		// original eyes
//		circle(mDestColor, Point(m_ptEyeL.x, m_ptEyeL.y), 2, Scalar(0, 0, 255), -1);	
//		circle(mDestColor, Point(m_ptEyeR.x, m_ptEyeR.y), 2, Scalar(0, 0, 255), -1);	
//		circle(mDestColor, Point(m_ptHead.x, m_ptHead.y), 5, Scalar(255, 0, 255), -1);	
	}

	if(bSaveFile)
		saveResult("dest", m_vecDest);	
}
void CRat::opticalMovement(Rect rect, vector <float>& vecPdfMove, vector <Mat>& vecmDist, float threshold, bool bOpFlowV1)
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
		opticalBuildPDF(mFlow, mGaus, vecmDist[i], rect);
		if(bOpFlowV1)
			vecPdfMove[i] = optical_compute_movement_v1(mFlow, vecmDist[i], threshold, rect);	
		else
			vecPdfMove[i] = optical_compute_movement_v2(mFlow, vecmDist[i], threshold, rect);	
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
	
	int wImg = m_vecMat[0].cols;
	int hImg = m_vecMat[0].rows;
	vecEye.resize(m_nSlices);
	vecEyeMove.resize(m_nSlices);	
	
	Point	offset(20, 20);
	Point 	ptEye = ptEye0;
	double sigma = 4;	
	
	for (int i = 0; i < m_nSlices; i++)	{	
		Point pt1 (ptEye-offset);
		Point pt2 (ptEye+offset);
	
		if(pt1.x <0) pt1.x = 0;
		if(pt1.y <0) pt1.y = 0;
		if(pt2.x >= wImg) pt2.x = wImg-1;
		if(pt2.y >= hImg) pt2.y = hImg-1;
	
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
void CRat::saveEarROI(int stable, int motion, Point& pt, Point	offset)
{
	int wImg = m_vecMat[0].cols;
	int hImg = m_vecMat[0].rows;
	
	Point pt1 (pt-offset);
	Point pt2 (pt+offset);
	
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

int CRat::findReferenceFrame(Rect rect)
{
	int start, end, lenSeg;
	start = 0;
    if(m_nLED2>0)
        end = m_nLED2;// + 10;
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
        mu = avgROI(mDiff, rect);
        vSeries[i] = mu;
    }
    
  	int  idxMin= -1;  
    double min = vSeries[end/2];
    for(int i=0; i<end; i++) {
        if(vSeries[i] <= min) {
            min = vSeries[i];
            idxMin = i;
        }
	}
	return idxMin;
}

double CRat::avgROI(Mat &mDiff, Rect rectEar)
{
	Mat mROI(mDiff, rectEar);
	
	Scalar sSum = cv::mean(mROI);
	
	return sSum[0];
}

void CRat::smoothData(vector<float>& inData, vector<float>& outData, int bw)
{
	int  n = inData.size();	

	double* pData = new double[n];
	double  *out = new double[n];

	for(int i=0; i<n; i++)  pData[i] = inData[i];
	
	CKDE::MSKernel kde_kernel = CKDE::Gaussian;
	
	CKDE kde1x(pData, out, n);
	kde1x.KernelDensityEstimation(kde_kernel, bw);

	outData.resize(n);
	for(int i=0; i<n; i++)  outData[i] = out[i];
	//outData.assign (out,out+n);	
	
	delete [] pData;
	delete [] out;
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
void CRat::pointGraylevel(Point ptBellyRed, Point ptBellyCyan, vector <float>& vecRedPoint, vector <float>& vecCyanPoint)
{
	vecRedPoint.resize(m_nSlices);
	vecCyanPoint.resize(m_nSlices);

	for(int i=0; i<m_nSlices; i++) {
		Mat mSrc = m_vecMat[i];
		vecRedPoint[i] = mSrc.at<uchar>(ptBellyRed.y, ptBellyRed.x);
		vecCyanPoint[i] = mSrc.at<uchar>(ptBellyCyan.y, ptBellyCyan.x);
	}
	
}
void CRat::graylevelDiff(int refer, Rect rectEarL, Rect rectEarR, vector <float>& vLEarGray,  vector <float>& vREarGray)
{
	vLEarGray.resize(m_nSlices);
	vREarGray.resize(m_nSlices);
	
	Mat mSrc1, mSrc2, mDiff;
	
	m_vecMat[refer].convertTo(mSrc1, CV_16S);;
	
	for(int i=0; i<m_nSlices; i++) {
		double errSumL, errSumR;
		m_vecMat[i].convertTo(mSrc2, CV_16S);
		cv::absdiff(mSrc1, mSrc2, mDiff);
		
		errSumL = avgROI(mDiff, rectEarL);
		errSumR = avgROI(mDiff, rectEarR);
		vLEarGray[i] = errSumL;
		vREarGray[i] = errSumR;
	}
}

void CRat::graylevelDiff(int refer, cv::Rect rect, vector <float>& vGray, int szSeries, bool bAbs)
{
	int numSlices;
	if(szSeries <0)
		numSlices = m_nSlices;
	else
		numSlices = szSeries;
		
	vGray.resize(numSlices);	
	
	Mat mSrc1, mSrc2, mDiff;
	
	m_vecMat[refer].convertTo(mSrc2, CV_16S);;
	
	for(int i=0; i<numSlices; i++) {
		double errSumL, errSumR;
		m_vecMat[i].convertTo(mSrc1, CV_16S);
		if(bAbs)
			cv::absdiff(mSrc1, mSrc2, mDiff);
		else
			mDiff = mSrc1 - mSrc2;
		
		errSumL = avgROI(mDiff, rect);
		vGray[i] = errSumL;
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

void CRat::opticalAssignThresholdMap(vector<Mat>& vmPDF, float th, Rect rect)
{
    int sz = m_vecFlow.size();

    for(int i=0; i<sz; i++) {
        Mat& mFlow = m_vecFlow[i]; 
        Mat& mPdf = vmPDF[i];
        Mat& mThMap = m_vmOpPDFMap[i];
        
        Mat mROI(mFlow, rect);
		int count = 0;
        for(int y = 0; y < mROI.rows; y ++) {
            for(int x = 0; x < mROI.cols; x ++)
            {
                Point2f fxy = mROI.at<Point2f>(y, x);
                Point2f fxy1 = fxy + Point2f(PDF_SIZE/2, PDF_SIZE/2);
                
                if(fxy1.x+0.5 >= PDF_SIZE || fxy1.x <0)  {
//                    wxString str;
//                    str.Printf("[%f, %f]", fxy1.x, fxy1.y);
//                    wxLogMessage(str);
                    continue;
                }
                if(fxy1.y+0.5 >= PDF_SIZE || fxy1.y <0)  continue;
                
                float probability = mPdf.at<float>(fxy1.y+0.5, fxy1.x+0.5);
                if(probability >= th) 	{
                    mThMap.at<uchar>(y+rect.y,x+rect.x) = 1;
					count ++;
				}
            } 
		}
    }
}

void CRat::opticalDrawFlowmapWithPDF(vector<Rect>& vDrawRect, vector<Point>& vPt, int nFrameSteps)
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

        for(int k=0; k<vDrawRect.size(); k++) {
            Rect rect = vDrawRect[k];
            Point pt = vPt[k];
            cv::rectangle(mFlowmapColor,rect, cv::Scalar(0, 255, 255)); 
 //           cv::circle(mFlowmapColor, pt, 2, Scalar(255, 255, 0), -1);	       
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
void CRat::opticalDrawFlowmap(Point pt1, Point pt2, Point offset, int nFrameSteps, char type)
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

		cv::rectangle(mFlowmapColor,pt1-offset, pt1+offset, cv::Scalar(0, 0, 255));
		cv::rectangle(mFlowmapColor,pt2-offset, pt2+offset, cv::Scalar(0, 0, 255));
		if(type =='E') {
			// new positions of eyes 
			circle(mFlowmapColor, Point(m_vecEyeL[i].x, m_vecEyeL[i].y), 3, Scalar(0, 0, 255), -1);	
			circle(mFlowmapColor, Point(m_vecEyeR[i].x, m_vecEyeR[i].y), 3, Scalar(0, 0, 255), -1);
			// reference eyes
			circle(mFlowmapColor, Point(m_vecEyeL[m_referFrame].x, m_vecEyeL[m_referFrame].y), 2, Scalar(0, 255, 255), -1);	
			circle(mFlowmapColor, Point(m_vecEyeR[m_referFrame].x, m_vecEyeR[m_referFrame].y), 2, Scalar(0, 255, 255), -1);	
	   
			cv::rectangle(mFlowmapColor,ptEyeC-offset,ptEyeC+offset, cv::Scalar(255, 0, 255));
		}
	}

	saveResult("flow", vecFlowmap);	
}

bool CRat::prepareGnuPlot(Gnuplot& plotSave, int numPlots, char* subpath, int range)
{
    wxString str;
    
    if(numPlots!= 2 && numPlots!= 4 && numPlots!= 1) {
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
    else if(numPlots==1) 
        plotSave.cmd("set terminal png size 400, 400");
#else
    if(numPlots==2) 
        plotSave.cmd("set terminal pngcairo size 800, 400");
    else if(numPlots==4) 
        plotSave.cmd("set terminal pngcairo size 800, 800");
    else if(numPlots==1) 
        plotSave.cmd("set terminal pngcairo size 400, 400");
#endif	
    
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
		plotSave.cmd("set ticslevel 0");
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
    if(m_bShowBelly)  numScatterPlot+=1;
    if(m_bShowEye)  numScatterPlot+=1;
	int range = 5; //PDF_SIZE/2;
    bool bRet = prepareGnuPlot(plotSave, numScatterPlot, subpath, range);   
    
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
        
        if(numScatterPlot ==1) {
			szX = 1;
            szY = 0.97;
            if(m_bShowBelly) {
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_rectBelly, m_vmDistRed,
                        "_Belly", "Abdomen", threshold, szX, szY, oriX, oriY); 
            }  
            if(m_bShowEye) {
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_rectHead, m_vmDistEyeL,
                        "_Head", "Head", threshold, szX, szY, oriX, oriY); 
                //oriX = 0.5; oriY = 0;
                //plotOneSpot(type, plotSave, i, saveName, m_ptEyeR, offset, m_vmDistEyeR,
                //        "_EyeR", "Right Eye", threshold, szX, szY, oriX, oriY); 
            }                   
        }else if(numScatterPlot ==2) {
			szX = 0.5;
            szY = 0.97;
            if(m_bShowEar) {
                oriX = 0; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_rectEarL, m_vmDistEarL,
                        "_EarL", "Left Ear", threshold, szX, szY, oriX, oriY); 
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_rectEarR, m_vmDistEarR,
                        "_EarR", "Right Ear", threshold, szX, szY, oriX, oriY); 
            }else {
            
                oriX = 0; oriY = 0;
				plotOneSpot(type, plotSave, i, saveName, m_rectBelly, m_vmDistRed,
                        "_Belly", "Abdomen", threshold, szX, szY, oriX, oriY); 
         
                oriX = 0.5; oriY = 0;
                plotOneSpot(type, plotSave, i, saveName, m_rectHead, m_vmDistEyeL,
                        "_Head", "Head", threshold, szX, szY, oriX, oriY); 
                //oriX = 0.5; oriY = 0;
                //plotOneSpot(type, plotSave, i, saveName, m_ptEyeR, offset, m_vmDistEyeR,
                //        "_EyeR", "Right Eye", threshold, szX, szY, oriX, oriY); 
            }                   
		}else if(numScatterPlot ==4) {
			szX = 0.5;
			szY = 0.5;

            oriX = 0; oriY = 0.5;
            plotOneSpot(type, plotSave, i, saveName, m_rectEarL, m_vmDistEarL,
                    "_EarL", "Left Ear", threshold, szX, szY, oriX, oriY); 
            oriX = 0.5; oriY = 0.5;
            plotOneSpot(type, plotSave, i, saveName, m_rectEarR, m_vmDistEarR,
                    "_EarR", "Right Ear", threshold, szX, szY, oriX, oriY); 
                

            oriX = 0; oriY = 0;                
       
			plotOneSpot(type, plotSave, i, saveName, m_rectBelly, m_vmDistRed,
                    "_Belly", "Abdomen", threshold, szX, szY, oriX, oriY); 
                    
            oriX = 0.5; oriY = 0;           
            plotOneSpot(type, plotSave, i, saveName, m_rectHead, m_vmDistEyeL,
                    "_Head", "Head", threshold, szX, szY, oriX, oriY); 
                        
        }

        plotSave.cmd("unset multiplot");
        
    }
    plotSave.cmd("reset");    
}
void CRat::plotOneSpot(char* type, Gnuplot& plotSave, int i, wxFileName& saveName, Rect rect, vector<Mat>& vmPDF,
								const char* extName1, const char* title1, float threshold,
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
		plotDotScatter(plotSave, mFlow, rect, strOutName, mPdf, threshold);
	else
		plotDistribution(plotSave, mPdf, strOutName, threshold);
}

void CRat::plotDistribution(Gnuplot& plot, Mat& mDist, wxString& strOutName, float threshold)
{
	double maxVal=0;
	minMaxLoc(mDist, 0, &maxVal, 0, 0);
	plot.set_zrange(0,maxVal+0.01);
	
	wxString fNameBin = strOutName + ".bin";
	_OutputMatGnuplotBinData(mDist, fNameBin.ToAscii(), -PDF_SIZE/2, PDF_SIZE/2);
	
	std::ostringstream cmdstr1;
	cmdstr1 << "z=" << threshold;
	plot.cmd(cmdstr1.str());
	plot.cmd("splot z");	
	
	std::ostringstream cmdstr;
    cmdstr << "splot '" << fNameBin.ToAscii() << "' binary matrix using 1:2:3";
    plot.cmd(cmdstr.str());	
}

void CRat::plotDotScatter(Gnuplot& plotSavePGN, Mat& mFlow, Rect rect, wxString& strOutName, Mat& mPdf, float threshold)
{
	Mat mROI(mFlow, rect);
	
	vector<Point2f>  vDataUp;
	vector<Point2f>  vDataBelow;
	
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
			Point2f fxy = mROI.at<Point2f>(y, x);
			Point2f fxy1 = fxy + Point2f(PDF_SIZE/2, PDF_SIZE/2);
			
			if(fxy1.x+0.5 >= PDF_SIZE || fxy1.x <0) {
				vDataBelow.push_back(fxy);
				continue;
			}
			if(fxy1.y+0.5 >= PDF_SIZE || fxy1.y <0) {
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
    cmdstr1 << "plot '" << fName.ToAscii() << "'  with points pointtype 1 linecolor '#FF0022'";
    plotSavePGN.cmd(cmdstr1.str());	
}


float CRat::optical_compute_movement_v1(Mat& mFlow, Mat& mDistEar, float threshold, Rect rect)
{	
	Mat mROI(mFlow, rect);
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
			
			if(fxy.x+0.5 >= PDF_SIZE || fxy.x <0) continue;
			if(fxy.y+0.5 >= PDF_SIZE || fxy.y <0) continue;
			
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

float CRat::optical_compute_movement_v2(Mat& mFlow, Mat& mDistEar, float threshold, Rect rect)
{	
	Mat mROI(mFlow, rect);
	float movement = 0;
	float count = 0;
	Point2f fxysum(0,0);
    for(int y = 0; y < mROI.rows; y ++)
        for(int x = 0; x < mROI.cols; x ++)
        {
			Point2f fxy = mROI.at<Point2f>(y, x);
			Point2f fxy0 = fxy;
		
			fxy += Point2f(PDF_SIZE/2, PDF_SIZE/2);
			
			if(fxy.x+0.5 >= PDF_SIZE || fxy.x <0) continue;
			if(fxy.y+0.5 >= PDF_SIZE || fxy.y <0) continue;
			
			float Pr = mDistEar.at<float>(fxy.y+0.5, fxy.x+0.5);

			if(Pr >= threshold) {
				fxysum += fxy0;
				count++; 
			}
		}
	
	movement  = cv::norm(fxysum);
	if(count!=0)
		movement = (movement /count);
	  	
	return (movement);
}

void CRat::opticalBuildPDF(Mat& mFlow, Mat& mGaus, Mat& mDist, Rect rect)
{
	Mat mROI(mFlow, rect);
	
	int ksize = mGaus.rows;
	int border = ksize /2;
	int pdfCenter = PDF_SIZE /2;
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
