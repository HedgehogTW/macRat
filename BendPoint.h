#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;

class CSeries{
public:
	CSeries() { };
	CSeries(int i, double s, bool p) { 
		idx = i; 
		slope = s; 
		minDiff = 0;
		peak = p;
	};

	bool    peak;
	int		idx;
	double	slope;
	double  minDiff;

	bool operator < (const CSeries &m)const {
                return idx < m.idx;
    }
};


class CPeak{
public:
	int		idx;
	double	slope;

	//bool operator < (const CPeak &m)const {
 //               return slope > m.slope;
 //   }
};
class CNotch{
public:
	int		idx;
	double	slope;

	//bool operator < (const CNotch &m)const {
 //               return slope < m.slope;
 //   }
};
class CBendPoint
{
public:
	CBendPoint();
	CBendPoint(double *pData, int num, int seglen);
	~CBendPoint(void);

	int  bendingPointsByCurvature(vector<cv::Point>& contour, int k, int angleTh, vector<float>& curvature);
	bool computeKCurvature(vector<cv::Point>& contour, vector<float>& curvature, int k);
	void findBendPoint(int start, int end, int& idxMin, int& idxMax);
	void findAllBendPoints(int start, int end, double threshold);
	int  findSegmentLen(int start, int end);
	void computeMinDiff();
	void mergeByMinDiff(double threshold);
	void mergeBySlope(double threshold);

	double		*m_pDataY;
	int			m_numPts;
	int			m_nSegLen;

	double		*m_SlopeDiff;
	double		*m_MinMax;
	double		*m_Seg1;
	double		*m_Seg2;
	double		*m_x;

	vector<CNotch>  m_vNotch;
	vector<CPeak>   m_vPeak;

	vector<CSeries> m_vSeries;
	vector<CSeries> m_vSortedSeries;
	int		m_nMaxPeakIdx;
	
	vector<int>			m_bendingIdx;
};
