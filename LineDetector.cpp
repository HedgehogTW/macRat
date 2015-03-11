
#include "LineDetector.h"


CLineDetector::CLineDetector(Mat mSrc, int ksize)
{
	m_nKerSize = ksize;  //7

	mSrc.copyTo(m_Src);
}


CLineDetector::~CLineDetector(void)
{
}

bool CLineDetector::detectHorizontal(vector<Vec2s>& vecLine, int scanAngle, double threshold, double stddevTh)
{
	// detect horizontal line
//	TRACE("detect Horizontal.............\n");
	createHorKernel(m_Src.cols, scanAngle);
	bool bRet = detectHorLine(m_Src, vecLine, threshold, stddevTh, "_line_h.csv");
	return bRet;
}

bool CLineDetector::detectVertical(vector<Vec2s>& vecLine, int scanAngle, double threshold, double stddevTh)
{
	// detect vertical line
	Mat mIn, mT;
	transpose(m_Src, mT);
	flip(mT, mIn, 1); // code = 0 - x axis; 1 - y ; -1 - both

//	TRACE("detect Vertical.............\n");
	createHorKernel(mIn.cols, scanAngle);
	bool bRet = detectHorLine(mIn, vecLine, threshold, stddevTh, "_line_v.csv");
	return bRet;
}


void CLineDetector::createHorKernel(int width, int scanAngle)
{ 	
	cv::Mat mGaus1D, mKernel;

	mGaus1D = getGaussianKernel(m_nKerSize, -1, CV_32F);
	repeat(mGaus1D, 1, width, mKernel);

	int	angleFrom = -scanAngle;
	int	angleTo = scanAngle;

	int bigRows = abs(tan(angleFrom*CV_PI/180.) * width +1);

	m_vmKernel.clear();
	m_vKerCentralLine.clear();

	int step = 1;
	int angle = angleFrom;
	do{
		cv::Mat mRotated, mLargeKer;
		mRotated.create(bigRows, width, CV_32F);
		mLargeKer = Mat::zeros(bigRows, width, CV_32F);

		float* pData = mLargeKer.ptr<float>(bigRows/2-mKernel.rows/2);
		memcpy(pData, mKernel.data, mKernel.rows*mKernel.cols * sizeof(float));

		Mat aff = cv::getRotationMatrix2D(Point(width/2, bigRows/2), angle, 1);	
		warpAffine(mLargeKer, mRotated, aff, mRotated.size());
		
		m_vmKernel.push_back(mRotated);

		// kernel central line
		int kh = (tan(angle*CV_PI/180.) * width/2 );
		Vec2i a;
		a.val[0] = kh;
		a.val[1] = -kh;
		m_vKerCentralLine.push_back(a);

		//gpMsgbar->ShowMessage("angle %d, central line %d %d, h %d\n", angle, a[0], a[1], kh);
		angle += step;
	}while(angle<=angleTo);

}

bool CLineDetector::detectHorLine(Mat& mGrad, vector<Vec2s>& vecLine, double threshold, double stddevTh, char*fname)
{
	vecLine.clear();


	int numKernels = m_vmKernel.size();

	//cv::Mat mDx, mDxAbs, mResp;
	//cv::Sobel(m_Src, mDx, CV_32F, 0, 1, 3);
	//mDxAbs = abs(mDx);
	cv::Mat mResp = Mat::zeros(mGrad.rows, numKernels,  CV_32F);

	cv::Mat mSrc2;
	mGrad.convertTo(mSrc2,CV_32F);
	//clock_t start, finish;
	//double  duration;
	//start = clock();
	int szKernel = m_vmKernel[0].rows * m_vmKernel[0].cols;
	int nKerHeigh = m_vmKernel[0].rows;
//#pragma omp parallel for 
	for(int k=0; k<numKernels; k++) {
		for(int i=0; i<mSrc2.rows - nKerHeigh; i++) {
			Mat mIn = mSrc2.rowRange(i, nKerHeigh +i);
			Mat mC = mIn.mul(m_vmKernel[k]);
			Scalar s = cv::sum(mC);

			mResp.at<float>(i+nKerHeigh/2, k) = s[0]/szKernel;
		}
	}
	//finish = clock();
	//duration = (double)(finish - start) / CLOCKS_PER_SEC;
	//gpOutput->ShowMessage("computation time: %.3f\n", duration);

	Mat mMaxY  = Mat::zeros(mResp.rows, 1,  CV_32F);
	vector<int>  vKernelResp(mResp.rows);
	for(int i=0; i<mResp.rows; i++){
		const float* mR = mResp.ptr<float>(i);
		int idxMax =0;
		float maxResp = -999;
		for(int k=0; k<numKernels; k++) {
			if(mR[k] > maxResp) {
				maxResp = mR[k];
				idxMax = k;
			}
		}
		vKernelResp[i] = idxMax;
		mMaxY.at<float>(i, 0) = maxResp;
	}
	// smooth
	int nKerSz = 11;

	//cv::reduce(mResp, mMaxY, 1, CV_REDUCE_MAX);
	Mat mSmoothY = Mat::zeros(mMaxY.rows, mMaxY.cols,  CV_32F);
	Mat mGaus1D = getGaussianKernel(nKerSz, -1, CV_32F);
	for(int i=0; i<mMaxY.rows - nKerSz; i++) {
		Mat mIn = mMaxY.rowRange(i, nKerSz +i);
		Mat mC = mIn.mul(mGaus1D);
		Scalar s = cv::sum(mC);
		mSmoothY.at<float>(i+nKerSz/2, 0) = s[0];
	}

	// find maximum
	float *data = (float*) (mSmoothY.data);
	for(int i=3; i<mMaxY.rows-3; i++) {
		if(data[i] > threshold) {
			if((data[i-3]<= data[i-2]) &&(data[i-2]<= data[i-1]) && (data[i-1]<= data[i]) && 
				(data[i] >= data[i+1]) && (data[i+1]>= data[i+2])&& (data[i+2]>= data[i+3])) 
			{		
				Vec2s a;
				a.val[0] = i+ m_vKerCentralLine[vKernelResp[i]].val[0];
				a.val[1] = i+ m_vKerCentralLine[vKernelResp[i]].val[1];
				vecLine.push_back(a);
//				TRACE("line %d, threshold %f\n", a.val[0] , data[i]);
			}
		}
	}
//	TRACE("after smooth threshold: %d line candidates\n", vecLine.size());
	// for display
	//Mat mHCanvas;
	//m_Src.convertTo(mHCanvas, CV_8U );
	//cvtColor(mHCanvas, mHCanvas, CV_GRAY2BGR);
	//cv::Point maxPt1, maxPt2;

	// check variance
	int oneSideLen = 10;
	int range;
	if(oneSideLen > nKerHeigh/2)  range = oneSideLen;
	else range = nKerHeigh/2;
	
	for(int i=0; i<vecLine.size(); i++) {
		int peak = vecLine[i].val[0];
		if(peak <= range || peak >= mGrad.rows - range) {
			vecLine.erase (vecLine.begin()+i);
			i--;
			continue;
		}
		Mat mNeighbor = mSmoothY.rowRange(peak-oneSideLen, peak+oneSideLen);
		cv::Scalar stddev, mean;
		cv::meanStdDev(mNeighbor, mean, stddev);
//		TRACE("%d, std %f", peak, stddev[0]);
		//maxPt1.x = 0; 
		//maxPt1.y = vecLine[i].val[0];
		//maxPt2.x = m_Src.cols-1; 
		//maxPt2.y = vecLine[i].val[1];
		//cv::line(mHCanvas, maxPt1, maxPt2, cv::Scalar(0,255,255), 1);

		if(stddev[0] < stddevTh) {
//			TRACE(" X\n");
			vecLine.erase (vecLine.begin()+i);
			i--;
			continue;
		}//else TRACE("\n");

	}
//	TRACE("after stddev threshold: %d line candidates\n", vecLine.size());	
	
	//cv::imshow( "Horitonal", mHCanvas);	
//	waitKey(0);

	//FILE*fp = fopen(fname, "w");
	//if(fp!=NULL) {
	//	for(int i=0; i<mMaxY.rows; i++) {	
	//			fprintf(fp, "%.3f, %.3f, %d\n", 
	//				mMaxY.at<float>(i, 0), mSmoothY.at<float>(i, 0), vKernelResp[i]) ;
	//	}
	//	fclose(fp);
	//}

	if(vecLine.size() >0) return true;
	else return false;

}

