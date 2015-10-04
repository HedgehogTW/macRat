#pragma once

#include <vector>
#include <string>
#include <algorithm> 
#include <opencv2/opencv.hpp>

#include <itkImage.h>
#include "itkImportImageFilter.h"
#include "itkRGBPixel.h"
#include "itkMetaImageIO.h"

#include "gnuplot_i.h"

#define NUM_FEATURE	 6
//#define m_ROIsz	40

using namespace std;
using namespace cv;

typedef unsigned char charPixelType;
typedef itk::Image< charPixelType, 2 > charImageType;

typedef float floatPixelType;
typedef itk::Image< floatPixelType, 2 > floatImageType;

typedef itk::ImportImageFilter< charPixelType, 2 > ImportFilterType;

typedef Vec<float, NUM_FEATURE+1> TrainPattern;

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
	double m_threshold;
	bool 	m_bLED;
	bool   m_bBigHead;
	bool 	m_bVerLine;

	bool 	m_bEyeMove;
	bool   m_bEar;
	bool 	m_bBelly;	
	bool 	m_bGrayDiff;
	bool 	m_bOpticalPDF;
	bool	m_bOpFlowV1;
	bool 	m_bAccumulate;
	bool	m_bSaveFile;
	double m_verLine;
	double m_ymin;
	double	m_ymax;	
	double m_xSD;
	
	long	m_szROIEar;
	long	m_szROIBelly;
	long   m_referFrame;

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
		m_ymax = 3;
		m_xSD = 3;
	}
};


class CRat
{
public:
	CRat();
	~CRat(void);

	void	clearData();
	int		readData(wxString path);
	Size	getImgSize()  { return m_szImg; }
	Mat &   getSrcImg(int idx) { return m_vecMat[idx]; }
	Mat &   getResultImg(int idx) { return m_vecDest[idx]; }

	int		getNumFrames()  { return m_nSlices; }
	void	saveResult(const char *subpath, vector <Mat> &vecDest);
	bool	detectLED(int nCageLine);
	Point	aspectRatio(vector<cv::Point> &con, double &ratio, double &angle );
	void	cropImage(bool& bCutTop);
	void 	DC_removal(int nFirstLED, vector <float>& vecSignal);
	float  Notch_removal(vector <float>& vecSignal, int refFrame);
        
	void	getLightRange(int& from, int& to) { from = m_nLED1; to = m_nLED_End; }
	void 	computeEyeMaskCenter(Point& ptNewMaskCenter, bool bBigHead);
	void	computeROIRect();
	void	recomputeHeadROI();
	float	computeSD(vector<float>& vSignal, int nLED2);
    bool	process(Point& ptEyeL, Point& ptEyeR, Point& ptEarL, Point& ptEarR, Point& ptRed, Point& ptCyan, int& nLED2);
	bool	genReferenceFrameSignal(Point& ptRed, Point& ptCyan, int& nLED2);
	
	void	plotSoundOnset(float baseline, float deltaY, int msec);
	void 	drawOnDestImage(bool bSaveFile);
	void 	findEyeCenter(Point& ptEye0, vector <Point>& vecEye, vector <float>&  vecEyeMove, int referFrame);
	void  	findNewEarCenter(vector <Point>& vecEye, Point ptEar0, vector <Point>& vecEar, int referFrame);
	void	findPeaks(vector<float>& inDataOri, vector<float>& inData, vector<Point2f>& peaks);
	void	peakAnalysis(vector<Point2f>& peaks, vector<float>& vPeakDistX, vector<float>& vPeakDistY, int nLED2, float& mean, float& sd);
	float	findMode(vector<float>& inData, float sigma);
	
	void 	smoothData(vector<float>& inData, vector<float>& outData, int bw=5);
	double errorSum(Mat &mDiff, Rect rectEar);
	int		findReferenceFrame(Rect rect);
	int 	findMaxMotionPoint(vector<float>& inData);
//	void	graylevelDiff_Eye(int refer, Point ptEar, Point offset, vector <Point>& vecEye, vector <float>& vecEarGrayDiff);
	void 	graylevelDiff(int refer, Rect rectEarL, Rect rectEarR, vector <float>& vLEarGray,  vector <float>& vREarGray);
	void	pointGraylevel(Point ptBellyRed, Point ptBellyCyan, vector <float>& vecRedPoint, vector <float>& vecCyanPoint);
	int    isRedSignificant(vector<float>& vecRed, vector<float>& vecCyan);
    
	void 	imageDiff(vector<Mat>& vecDiff, vector <float>& vecAdjDiff, int nFrameSteps);
	void 	saveEarROI(int stable, int motion, Point& pt, Point	offset);
	void 	saveEyeTrajectory();
	
	void 	opticalFlow_v2(int referFrame);
	void	opticalFlow_v1(int nFrameSteps, int referFrame);
	void	opticalMovement(Rect rect, vector<float>& vecPdfMove, vector <Mat>& vecmDist, float threshold, bool bOpFlowV1);
	void 	opticalDrawFlowmap(Point pt1, Point pt2, Point offset, int nFrameSteps, char type);
    void 	opticalDrawFlowmapWithPDF(vector<Rect>& vDrawRect, vector<Point>& vPt, int nFrameSteps);

	void	drawOptFlowMap(Mat& cflowmap, const Mat& flow, int step, const Scalar& color);
    void	drawOptFlowMapWithPDF(Mat& cflowmap, const Mat& flow,  int step, Mat &mPdf);    
    									
    bool    prepareGnuPlot(Gnuplot& plotSave, int numPlots, char* subpath, int range);
    void    opticalSavePlot(char* subpath, char* type, float threshold);
    void    plotOneSpot(char* type, Gnuplot& plotSave, int i, wxFileName& saveName, Rect rect, vector<Mat>& vmPDF,
								const char* extName1, const char* title1, float threshold,
                                float sizeX, float sizeY, float oriX, float oriY);
								
	float 	optical_compute_movement_v1(Mat& mFlow, Mat& mDistEar, float threshold, Rect rect);
	float 	optical_compute_movement_v2(Mat& mFlow, Mat& mDistEar, float threshold, Rect rect);
	void 	plotDistribution(Gnuplot& plot, Mat& mDist, wxString& strOutName, float threshold);
	void 	plotDotScatter(Gnuplot& plotSavePGN, Mat& mFlow, Rect rect, wxString& strOutName, Mat& mPdf, float threshold);	
	void	opticalBuildPDF(Mat& mFlow, Mat& mGaus, Mat& mDist, Rect rect);

	void    opticalAssignThresholdMap(vector<Mat>& vmPDF, float th, Rect rect);

    bool    opticalLoadPDFfile(const char* filename, Mat &mPdf);
	void    saveDistributionAsCSVandPNG();
	void 	linearRegression(vector <float>& vecSignal, vector <float>& vecOut, vector <float>& vecSubOut);
	

	float	findAvgMotion(Mat& mFlowROI);
	
	void 	createGaussianMask(double& sigma, int& ksize, Mat& mKernel);

	void	itkLaplacianOfGaussian(Mat& mSrc, Mat& mDest, double sigma);


	void	ContourSmooth(vector<cv::Point> & contour, int sigma);
	void	BresLine(int Ax, int Ay, int Bx, int By, vector<cv::Point> & contour);
	double	lineSquareError(vector<cv::Point>& contour, cv::Point& pt);

	Mat		accumulate();
	void 	saveEarImage();
	
public:
	vector <Mat> m_vecMat;
	vector <Mat> m_vecDest;
	vector <Mat> m_vecFlow;

	vector <Mat> m_vmDistEarL;
	vector <Mat> m_vmDistEarR;
	vector <Mat> m_vmDistEyeL;
	vector <Mat> m_vmDistEyeR;
	vector <Mat> m_vmDistRed;
	vector <Mat> m_vmDistCyan;
    
    vector <Mat> m_vmOpPDFMap;
	
	vector <Point>  m_vecEyeL;
	vector <Point>  m_vecEyeR;
	vector <float>  m_vecEyeLMove;
	vector <float>  m_vecEyeRMove;
	
    int     m_BigRedPdf;
    int     m_BigRedGray;
//	Point	m_offsetEar; //(50, 50);

	Point 	m_ptEyeL;
	Point	m_ptEyeR;
//	Point	m_ptEyeC;
	Point	m_ptEarL;
	Point	m_ptEarR;
	Point 	m_ptRed;
	Point	m_ptCyan;
	Point	m_ptHead;
	
	Point 	m_offsetEye;
	Point 	m_offsetEar;
	Point 	m_offsetAPB;	
	
	Rect	m_rectHead;
	Rect  	m_rectEarL;
	Rect  	m_rectEarR;	
	Rect  	m_rectRed;
	Rect  	m_rectCyan;	
	
//	vector <double>  m_vecLEyeGrayDiff;
//	vector <double>  m_vecREyeGrayDiff;

    bool    m_bShowEye;
    bool    m_bShowEar;
    bool    m_bShowBelly;
    
    vector <float>  m_vecEyeFlowPdf;
    
	long	m_ROIEar;
	long	m_ROIBelly;

	
	int		m_nSlices ;
	Size	m_szImg;
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

