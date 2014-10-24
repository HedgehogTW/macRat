#pragma once

#include <vector>
#include <string>
#include <algorithm> 
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

class CLineDetector
{
public:
	CLineDetector(Mat mSrc, int ksize);
	~CLineDetector(void);

	bool detectHorizontal(vector<Vec2s>& vecLine, int scanAngle, double threshold, double stddevTh);
	bool detectVertical(vector<Vec2s>& vecLine, int scanAngle, double threshold, double stddevTh);

private:
	void createHorKernel(int width, int scanAngle);
	bool detectHorLine(Mat& mGrad, vector<Vec2s>& vecY, double threshold, double stddevTh, char*fname);

	Mat		m_Src;
	int		m_nKerSize;

	vector<Mat>		m_vmKernel;
	vector<Vec2i>	m_vKerCentralLine;
};

