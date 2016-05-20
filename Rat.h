#pragma once

#include <vector>
#include <string>
#include <algorithm> 
#include <opencv2/opencv.hpp>

#include "gnuplot_i.h"

#define NUM_FEATURE	 6
//#define m_ROIsz	40

using namespace std;
//using namespace cv;

typedef cv::Vec<float, NUM_FEATURE+1> TrainPattern;

class CEarCandidate{

public:
	CEarCandidate(void) { bEar = false; }

	void copyContourFrom(vector<cv::Point> con) {
		int szContour = con.size();
		contour.resize(szContour);
		std::copy(con.begin(), con.end(), contour.begin());
	}
	void setcentroid(cv::Point c) { centroid = c; }
	void setDist(float d) { dist = d; }

public:
	vector<cv::Point> contour;
	cv::Point centroid;
	float dist;
	bool  bEar;
	double area;
};


class MyConfigData{
public:	
	long 	m_frameStep;	
	double 	m_threshold;
	bool 	m_bLED;
	bool   	m_bBigHead;
	bool 	m_bVerLine;

	bool 	m_bEyeMove;
	bool   	m_bEar;
	bool 	m_bBelly;	
	bool 	m_bGrayDiff;
	bool 	m_bOpticalPDF;
	bool	m_bOpFlowV1;
	bool 	m_bAccumulate;
	bool	m_bSaveFile;
	bool	m_bSaveSignalPlot;
	bool	m_bShowPeaks;
	bool	m_bShowAxisLabel;
	double 	m_verLine;
	double 	m_ymin;
	double	m_ymax;	
	double  m_ysnd;
	double 	m_intvymin;
	double	m_intvymax;	
	double  m_intvysnd;	
	double 	m_xSD;
	double  m_smoothEar;
    double  m_smoothBelly;
	
	long	m_szROIEar;
	long	m_szROIBelly;
	long   	m_referFrame;

	double	m_gainHead;
	double	m_gainBelly;
	int		m_refSignal;
	
	void Init() {
		m_gainHead = 1;
		m_gainBelly = 1;
		m_szROIEar = 60;
		m_szROIBelly = 80;
		m_refSignal = 0;
		m_ymin = -1;
		m_ymax = 2;
		m_ysnd = -0.65;
		
		m_intvymin = 00;
		m_intvymax = 70;
		m_intvysnd = 20;
		
		m_xSD = 3;
		m_smoothEar = 3;
        m_smoothBelly = 3;
		m_bSaveSignalPlot = false;
		m_bShowPeaks = false;
		m_bShowAxisLabel = false;
	}
};


class CRat
{
public:
	CRat();
	~CRat(void);

	void	clearData();
	int		readData(wxString path);
	cv::Size	getImgSize()  { return m_szImg; }
	cv::Mat &   getSrcImg(int idx) { return m_vecMat[idx]; }
	cv::Mat &   getResultImg(int idx) { return m_vecDest[idx]; }

	int		getNumFrames()  { return m_nSlices; }
	void	saveResult(const char *subpath, vector <cv::Mat> &vecDest);
	bool	detectLED(int nCageLine);
	cv::Point	aspectRatio(vector<cv::Point> &con, double &ratio, double &angle );
	void	cropImage(bool& bCutTop, int nCageLine);
	void 	DC_removal(int nFirstLED, vector <float>& vecSignal);
	float   Notch_removal(vector <float>& vecSignal, int refFrame);
        
	void	getLightRange(int& from, int& to) { from = m_nLED1; to = m_nLED_End; }
	void 	computeEyeMaskCenter(cv::Point& ptNewMaskCenter, bool bBigHead);
	void	computeROIRect();
	void	recomputeHeadROI();
	static float	computeSD(vector<float>& vSignal, int nLED2);
    bool	process(cv::Point& ptEyeL, cv::Point& ptEyeR, cv::Point& ptEarL, cv::Point& ptEarR, cv::Point& ptBelly, int& nLED2);
	bool	genReferenceFrameSignal(cv::Point& ptBelly, int& nLED2);
	
	void 	drawOnDestImage(bool bSaveFile);
	void 	findEyeCenter(cv::Point& ptEye0, vector <cv::Point>& vecEye, vector <float>&  vecEyeMove, int referFrame);
	void  	findNewEarCenter(vector <cv::Point>& vecEye, cv::Point ptEar0, vector <cv::Point>& vecEar, int referFrame);
	static void	findPeaks(vector<float>& inDataOri, vector<float>& inData, vector<cv::Point2f>& peaks);
	static int	peakPeriodAnalysis(vector<cv::Point2f>& peaks, vector<float>& vPeakDistX, vector<float>& vPeakDistY, int nLED2, float& mean, float& sd);
	static int peakAmplitudeAnalysis(vector<cv::Point2f>& peaks, int nLED2, float& mean, float& sd);
	float	findMode(vector<float>& inData, float sigma);
	
	static void 	smoothData(vector<float>& inData, vector<float>& outData, int bw=5);
	double  avgROI(cv::Mat &mDiff, cv::Rect rectEar);
	int		findReferenceFrame(cv::Rect rect);
	int 	findMaxMotionPoint(vector<float>& inData);
//	void	graylevelDiff_Eye(int refer, Point ptEar, Point offset, vector <Point>& vecEye, vector <float>& vecEarGrayDiff);
	void 	graylevelDiff(int refer, cv::Rect rectEarL, cv::Rect rectEarR, vector <float>& vLEarGray,  vector <float>& vREarGray);
	void 	graylevelDiff(int refer, cv::Rect rect, vector <float>& vGray, int szSeries=-1, bool bAbs=true);
	void	pointGraylevel(cv::Point ptBellyRed, cv::Point ptBellyCyan, vector <float>& vecRedPoint, vector <float>& vecCyanPoint);
	int     isRedSignificant(vector<float>& vecRed, vector<float>& vecCyan);
	cv::Point findMostSignificantPt(deque<cv::Point>&  dqBellyPts, int nLed2, long	szROIBelly);
    
	void 	imageDiff(vector<cv::Mat>& vecDiff, vector <float>& vecAdjDiff, int nFrameSteps);
	void 	saveEarROI(int stable, int motion, cv::Point& pt, cv::Point	offset);
	void 	saveEyeTrajectory();
	
	void 	opticalFlow_v2(int referFrame);
	void	opticalFlow_v1(int nFrameSteps, int referFrame);
	void	opticalMovement(cv::Rect rect, vector<float>& vecPdfMove, vector <cv::Mat>& vecmDist, float threshold, bool bOpFlowV1);
	void 	opticalDrawFlowmap(cv::Point pt1, cv::Point pt2, cv::Point offset, int nFrameSteps, char type);
    void 	opticalDrawFlowmapWithPDF(vector<cv::Rect>& vDrawRect, vector<cv::Point>& vPt, int nFrameSteps);

	void	drawOptFlowMap(cv::Mat& cflowmap, const cv::Mat& flow, int step, const cv::Scalar& color);
    void	drawOptFlowMapWithPDF(cv::Mat& cflowmap, const cv::Mat& flow,  int step, cv::Mat &mPdf);    
    									
    bool    prepareGnuPlot(Gnuplot& plotSave, int numPlots, char* subpath, int range);
    void    opticalSavePlot(char* subpath, char* type, float threshold);
    void    plotOneSpot(char* type, Gnuplot& plotSave, int i, wxFileName& saveName, cv::Rect rect, vector<cv::Mat>& vmPDF,
								const char* extName1, const char* title1, float threshold,
                                float sizeX, float sizeY, float oriX, float oriY);
								
	float 	optical_compute_movement_v1(cv::Mat& mFlow, cv::Mat& mDistEar, float threshold, cv::Rect rect);
	float 	optical_compute_movement_v2(cv::Mat& mFlow, cv::Mat& mDistEar, float threshold, cv::Rect rect);
	void 	plotDistribution(Gnuplot& plot, cv::Mat& mDist, wxString& strOutName, float threshold);
	void 	plotDotScatter(Gnuplot& plotSavePGN, cv::Mat& mFlow, cv::Rect rect, wxString& strOutName, cv::Mat& mPdf, float threshold);	
	void	opticalBuildPDF(cv::Mat& mFlow, cv::Mat& mGaus, cv::Mat& mDist, cv::Rect rect);

	void    opticalAssignThresholdMap(vector<cv::Mat>& vmPDF, float th, cv::Rect rect);

    bool    opticalLoadPDFfile(const char* filename, cv::Mat &mPdf);
	void    saveDistributionAsCSVandPNG();
	void 	linearRegression(vector <float>& vecSignal, vector <float>& vecOut, vector <float>& vecSubOut);
	

	float	findAvgMotion(cv::Mat& mFlowROI);
	
	void 	createGaussianMask(double& sigma, int& ksize, cv::Mat& mKernel);

//	void	itkLaplacianOfGaussian(Mat& mSrc, Mat& mDest, double sigma);


	void	ContourSmooth(vector<cv::Point> & contour, int sigma);
	void	BresLine(int Ax, int Ay, int Bx, int By, vector<cv::Point> & contour);
	double	lineSquareError(vector<cv::Point>& contour, cv::Point& pt);

	cv::Mat		accumulate();
	void 	saveEarImage();
	
public:
	vector <cv::Mat> m_vecMat;
	vector <cv::Mat> m_vecDest;
	vector <cv::Mat> m_vecFlow;

	vector <cv::Mat> m_vmDistEarL;
	vector <cv::Mat> m_vmDistEarR;
	vector <cv::Mat> m_vmDistEyeL;
	vector <cv::Mat> m_vmDistEyeR;
	vector <cv::Mat> m_vmDistRed;
//	vector <cv::Mat> m_vmDistCyan;
    
    vector <cv::Mat> m_vmOpPDFMap;
	
	vector <cv::Point>  m_vecEyeL;
	vector <cv::Point>  m_vecEyeR;
	vector <float>  m_vecEyeLMove;
	vector <float>  m_vecEyeRMove;
	
//    int     m_BigRedPdf;
//    int     m_BigRedGray;
//	Point	m_offsetEar; //(50, 50);

	cv::Point 	m_ptEyeL;
	cv::Point	m_ptEyeR;
//	Point	m_ptEyeC;
	cv::Point	m_ptEarL;
	cv::Point	m_ptEarR;
//	cv::Point 	m_ptRed;
//	cv::Point	m_ptCyan;
	cv::Point	m_ptHead;
	cv::Point	m_ptBelly;
	
	cv::Point 	m_offsetEye;
	cv::Point 	m_offsetEar;
	cv::Point 	m_offsetAPB;	
	
	cv::Rect	m_rectHead;
	cv::Rect  	m_rectEarL;
	cv::Rect  	m_rectEarR;	
	cv::Rect  	m_rectBelly;
//	cv::Rect  	m_rectRed;
//	cv::Rect  	m_rectCyan;	
	
//	vector <double>  m_vecLEyeGrayDiff;
//	vector <double>  m_vecREyeGrayDiff;

    bool    m_bShowEye;
    bool    m_bShowEar;
    bool    m_bShowBelly;
    
    vector <float>  m_vecEyeFlowPdf;
    
	long	m_ROIEar;
	long	m_ROIBelly;

	
	int		m_nSlices ;
	cv::Size	m_szImg;
	wxString  m_strSrcPath;
	vector <string> m_vFilenames;
	int		m_nLED1;
	int		m_nLED2;	
	int		m_nLED_End;
	bool	m_bLED_OK;
	int		m_nCageLine;


	int 	m_referFrame;
	
	float	m_mean;
	float	m_stddev;
	int		m_nProcessFrame;
	int		m_classifier;
};

