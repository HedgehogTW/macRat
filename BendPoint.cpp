
#include "BendPoint.h"
#include <gsl/gsl_fit.h>
#include <algorithm>
#include <stdio.h>
#include <string.h>

bool sortMinDiff(const CSeries & m1, const CSeries & m2) {
        return m1.minDiff < m2.minDiff;
}

bool sortSlope(const CSeries & m1, const CSeries & m2) {
        return fabs(m1.slope) < fabs(m2.slope);
}

CBendPoint::CBendPoint()
{
	
}
CBendPoint::CBendPoint(double *pData, int num, int seglen)
{ 

	m_pDataY = pData;
	m_numPts = num;
	m_nSegLen = seglen;

	m_MinMax = new double [m_numPts];
	m_SlopeDiff = new double [m_numPts];
	memset(m_SlopeDiff, 0, m_numPts * sizeof(double));
	memset(m_MinMax, 0, m_numPts * sizeof(double));

	m_Seg1 = new double[m_nSegLen]; 
	m_Seg2 = new double[m_nSegLen]; 
	m_x = new double[m_nSegLen]; 

}

CBendPoint::~CBendPoint(void)
{

	delete [] m_SlopeDiff;
	delete [] m_MinMax;

	delete [] m_Seg1;
	delete [] m_Seg2;
	delete [] m_x;


}


int CBendPoint::bendingPointsByCurvature(vector<cv::Point>& contour, int k, int angleTh, vector<float>& curvature)
{
	curvature.clear();
	m_bendingIdx.clear();

	bool bRet = computeKCurvature(contour, curvature, k);
	if(bRet ==false) return 0;

	int sz = curvature.size();
	for(int i=0; i<sz; i++) {
		if(curvature[i] < angleTh) {
			if( (curvature[(i-3+sz)%sz]>= curvature[(i-2+sz)%sz]) &&
				(curvature[(i-2+sz)%sz]>= curvature[(i-1+sz)%sz]) && 
				(curvature[(i-1+sz)%sz]>= curvature[i]) && 
				(curvature[i] <= curvature[(i+1)%sz]) && 
				(curvature[(i+1)%sz]<= curvature[(i+2)%sz]) && 
				(curvature[(i+2)%sz]<= curvature[(i+3)%sz])) {
				
				m_bendingIdx.push_back(i);
			}
		}
	}

	return m_bendingIdx.size();
}

bool CBendPoint::computeKCurvature(vector<cv::Point>& contour, vector<float>& curvature, int k)
{
	int m = contour.size();
	if(m<k) return false;

	cv::Mat mCurv;
	mCurv.create(m, 1, CV_32F);
	for(int j=0; j<m; j++) {

		cv::Point u, v;
		u = contour[(j+k)%m] - contour[j];
		v = contour[(j+m-k)%m] - contour[j];
		float uvdot = u.x*v.x + u.y*v.y;
		float uNorm = sqrt(u.x*u.x + u.y*u.y);
		float vNorm = sqrt(v.x*v.x + v.y*v.y);
		float a;
		if( uNorm==0 || vNorm==0)  
			a= 1;
		else a = uvdot/(uNorm*vNorm);
		if(a<-1) a = -1;
		float angle = acos(a)*180./CV_PI;
		mCurv.at<float>(j, 0) = angle;
	}

	// smooth
	int nKerSz = 3;
	cv::Mat mSmooth;
	mCurv.copyTo(mSmooth);

	cv::Mat mGaus1D = cv::getGaussianKernel(nKerSz, -1, CV_32F);
	for(int i=0; i<m - nKerSz; i++) {
		cv::Mat mIn = mCurv.rowRange(i, nKerSz +i);
		cv::Mat mC = mIn.mul(mGaus1D);
		cv::Scalar s = cv::sum(mC);
		mSmooth.at<float>(i+nKerSz/2, 0) = s[0];
	}

	curvature.resize(m);
	memcpy(&(curvature[0]), mSmooth.data, m*sizeof(float));

	//FILE* fp = fopen("_curvature.txt", "w");
	//for(int i=0; i<m; i++) {
	//	fprintf(fp, "%f\n", curvature[i]);
	//}
	//fclose(fp);

	return true;
}


int CBendPoint::findSegmentLen(int start, int end)
// It does not work.
{
	int i, pivot, dataLen, len, c;
	double *seg, *x;
	double a1, b1, cov00, cov01, cov11, sumsq, mse, wave_mse;

	//gpMsgbar->ShowMessage("start %d, end %d\n", start, end);
	dataLen = end - start +1;
	len = dataLen /2;
	seg = new double[len]; 
	x = new double[len]; 

	do{
		c = 0;
		wave_mse = 0;
		for(pivot=start ; pivot<= end-len+1; pivot++) {
			for(i=0; i<len; i++) {
				seg[i] = m_pDataY[pivot+i];
				x[i] = i;
			}
			
			gsl_fit_linear(x, 1, seg, 1, len, &a1, &b1, &cov00, &cov01, &cov11, &sumsq);
			mse = sumsq/len;
			wave_mse += mse;
			c++;
		}
		//wave_mse /= c;
		//gpMsgbar->ShowMessage("wave_mse = %f, len = %d, c=%d\n", wave_mse, len, c);
		len = len * 2./3.;


	}while(len>=5);


	delete [] seg;
	delete [] x;

	return len;
}

void CBendPoint::findBendPoint(int start, int end, int& idxMin, int &idxMax)
{
	int i, pivot, wavelen;

	double a1, b1, a2, b2, cov00, cov01, cov11, sumsq;
	double	largest, smallest;

	largest = INT_MIN;
	smallest = INT_MAX;
	wavelen = end - start +1;

	for(pivot=start + m_nSegLen-1; pivot<= start + wavelen-m_nSegLen; pivot++) {
		for(i=0; i<m_nSegLen; i++) {
			m_Seg1 [i] = m_pDataY[pivot-m_nSegLen+1+i];
			m_Seg2 [i] = m_pDataY[pivot+i];
			m_x[i] = i;
		}
		gsl_fit_linear(m_x, 1, m_Seg1, 1, m_nSegLen, &a1, &b1, &cov00, &cov01, &cov11, &sumsq);
		gsl_fit_linear(m_x, 1, m_Seg2, 1, m_nSegLen, &a2, &b2, &cov00, &cov01, &cov11, &sumsq);	

		m_SlopeDiff[pivot] = b2 - b1;
	}
	// find maximum during start, end
	// find dicrotic notch from the first half of the data
	for(pivot=start + m_nSegLen-1; pivot<= start + wavelen/2-m_nSegLen; pivot++) {
		if(m_SlopeDiff[pivot] > largest) {
			largest = m_SlopeDiff[pivot];
			idxMax = pivot;
		}
	}
	// find minimum during start, end
	// find dicrotic peak from the data after dicrotic notch
	for(pivot=idxMax; pivot<= start + wavelen-m_nSegLen; pivot++) {
		if(m_SlopeDiff[pivot] < smallest) {
			smallest = m_SlopeDiff[pivot];
			idxMin = pivot;
		}
	}

	m_MinMax[idxMax] = 1;
	m_MinMax[idxMin] = -1;
}


void CBendPoint::findAllBendPoints(int start, int end, double threshold)
{
	int i, pivot, wavelen;

	double a1, b1, a2, b2, cov00, cov01, cov11, sumsq;
	double	largest, smallest;

	largest = INT_MIN;
	smallest = INT_MAX;
	wavelen = end - start +1;

	for(pivot=start + m_nSegLen-1; pivot<= start + wavelen-m_nSegLen; pivot++) {
		for(i=0; i<m_nSegLen; i++) {
			m_Seg1 [i] = m_pDataY[pivot-m_nSegLen+1+i];
			m_Seg2 [i] = m_pDataY[pivot+i];
			m_x[i] = i;
		}
		gsl_fit_linear(m_x, 1, m_Seg1, 1, m_nSegLen, &a1, &b1, &cov00, &cov01, &cov11, &sumsq);
		gsl_fit_linear(m_x, 1, m_Seg2, 1, m_nSegLen, &a2, &b2, &cov00, &cov01, &cov11, &sumsq);	

		m_SlopeDiff[pivot] = b2 - b1;
	}

	m_vSeries.clear();
	for(i=start+2; i<end -2; i++) {		
		// slope peak, signal notch
		if( m_SlopeDiff[i-2] <= m_SlopeDiff[i-1] && m_SlopeDiff[i-1] < m_SlopeDiff[i] &&
			m_SlopeDiff[i+1] < m_SlopeDiff[i] && m_SlopeDiff[i+2] <= m_SlopeDiff[i+1]) {

			m_vSeries.push_back(CSeries(i, m_SlopeDiff[i], false));
		}
		// slope notch, signal peak
		if( m_SlopeDiff[i-2] >= m_SlopeDiff[i-1] && m_SlopeDiff[i-1] > m_SlopeDiff[i] &&
			m_SlopeDiff[i+1] > m_SlopeDiff[i] && m_SlopeDiff[i+2] >= m_SlopeDiff[i+1]) {

			m_vSeries.push_back(CSeries(i, m_SlopeDiff[i], true));
		}
	}
	FILE* fp = fopen("_slope.txt", "w");
	if(fp!= NULL) {
		for(i=0; i<m_numPts; i++)
			fprintf(fp, "%d %.3f\n", i, m_SlopeDiff[i]);
		fclose(fp);
	}

	sort(m_vSeries.begin(), m_vSeries.end());

	computeMinDiff();

	mergeByMinDiff(threshold);

	mergeBySlope(threshold);

	m_vNotch.clear();
	m_vPeak.clear();
	for(i=0; i<m_vSeries.size(); i++) {
		if(m_vSeries[i].peak) {
			CPeak  node;
			node.idx = m_vSeries[i].idx;
			node.slope = m_vSeries[i].slope;
			m_vPeak.push_back(node);
		}else {
			CNotch  node;
			node.idx = m_vSeries[i].idx;	
			node.slope = m_vSeries[i].slope;
			m_vNotch.push_back(node);
		}
	}

	double maxPeakSlop = 999;
	for(i=0; i<m_vPeak.size(); i++) {
		if(m_vPeak[i].slope < maxPeakSlop) {
			m_nMaxPeakIdx = i;
			maxPeakSlop = m_vPeak[i].slope ;
		}
	}
}

void CBendPoint::mergeBySlope(double threshold)
{
	m_vSortedSeries.resize(m_vSeries.size());
	copy ( m_vSeries.begin(), m_vSeries.end(), m_vSortedSeries.begin() );
	sort(m_vSortedSeries.begin(), m_vSortedSeries.end(), sortSlope);

	for(int i=0; i<m_vSortedSeries.size(); i++) {
		if(fabs(m_vSortedSeries[i].slope) <threshold) {
			int merge = -1;
			for(int j=0; j<m_vSeries.size(); j++) {
				if(m_vSeries[j].idx == m_vSortedSeries[i].idx) {
					merge = j;					
					break;
				}
			}
			if(merge ==0) { 
				m_vSeries.erase (m_vSeries.begin());
				//m_vSeries[0].minDiff = fabs(m_vSeries[0].slope);
			}else if(merge == m_vSeries.size()-1) {
				m_vSeries.erase (m_vSeries.begin()+merge);
				//m_vSeries[merge-1].minDiff = fabs(m_vSeries[merge-1].slope);
			}else if(merge > 0 && merge < m_vSeries.size()-1) {
				double a = fabs(m_vSeries[merge-1].slope);
				double b = fabs(m_vSeries[merge+1].slope);
				double c = a+b;

				m_vSeries[merge-1].idx = (m_vSeries[merge-1].idx*a/c + m_vSeries[merge+1].idx*b/c);
				m_vSeries[merge-1].slope = m_vSeries[merge-1].slope + m_vSeries[merge+1].slope;
				m_vSeries.erase (m_vSeries.begin()+merge, m_vSeries.begin()+merge+2);

				//computeMinDiff();
			}else continue;
			//gpMsgbar->ShowMessage("remove node %d\n", m_vSortedSeries[i].idx);			
			m_vSortedSeries.resize(m_vSeries.size());
			copy ( m_vSeries.begin(), m_vSeries.end(), m_vSortedSeries.begin() );
			sort(m_vSortedSeries.begin(), m_vSortedSeries.end(), sortSlope);


			//for(int j=0; j<m_vSeries.size(); j++)
			//	gpMsgbar->ShowMessage("[%d %.2f %d] ", 
			//		m_vSeries[j].idx, m_vSeries[j].slope, m_vSeries[j].peak);
			//gpMsgbar->ShowMessage("\n");
			i--;
		}
	}
}
void CBendPoint::mergeByMinDiff(double threshold)
{
	m_vSortedSeries.resize(m_vSeries.size());
	copy ( m_vSeries.begin(), m_vSeries.end(), m_vSortedSeries.begin() );
	sort(m_vSortedSeries.begin(), m_vSortedSeries.end(), sortMinDiff);

	for(int i=0; i<m_vSortedSeries.size(); i++) {
		if(m_vSortedSeries[i].minDiff <threshold) {
			int merge = -1;
			for(int j=0; j<m_vSeries.size(); j++) {
				if(m_vSeries[j].idx == m_vSortedSeries[i].idx) {
					merge = j;					
					break;
				}
			}
			if(merge ==0) { 
				m_vSeries.erase (m_vSeries.begin());
				m_vSeries[0].minDiff = fabs(m_vSeries[0].slope);
			}else if(merge == m_vSeries.size()-1) {
				m_vSeries.erase (m_vSeries.begin()+merge);
				m_vSeries[merge-1].minDiff = fabs(m_vSeries[merge-1].slope);
			}else if(merge > 0 && merge < m_vSeries.size()-1) {
				double a = fabs(m_vSeries[merge-1].slope);
				double b = fabs(m_vSeries[merge+1].slope);
				double c = a+b;

				m_vSeries[merge-1].idx = (m_vSeries[merge-1].idx*a/c + m_vSeries[merge+1].idx*b/c);
				m_vSeries[merge-1].slope = m_vSeries[merge-1].slope + m_vSeries[merge+1].slope;
				m_vSeries.erase (m_vSeries.begin()+merge, m_vSeries.begin()+merge+2);

				computeMinDiff();
			}else continue;
			//gpMsgbar->ShowMessage("remove node %d\n", m_vSortedSeries[i].idx);			
			m_vSortedSeries.resize(m_vSeries.size());
			copy ( m_vSeries.begin(), m_vSeries.end(), m_vSortedSeries.begin() );
			sort(m_vSortedSeries.begin(), m_vSortedSeries.end(), sortMinDiff);


			//for(int j=0; j<m_vSeries.size(); j++)
			//	gpMsgbar->ShowMessage("[%d %.2f %.2f %d] ", 
			//		m_vSeries[j].idx, m_vSeries[j].slope, m_vSeries[j].minDiff, m_vSeries[j].peak);
			//gpMsgbar->ShowMessage("\n");

			i--;
		}
	}
}
void CBendPoint::computeMinDiff()
{
	int i, sz;
	sz = m_vSeries.size();
	m_vSeries[0].minDiff = fabs(m_vSeries[0].slope);
	if(sz <=1) return;
	for(i=1; i<sz-1; i++) {
		double a = fabs(m_vSeries[i].slope - m_vSeries[i-1].slope);
		double b = fabs(m_vSeries[i].slope - m_vSeries[i+1].slope);
		if(a < b) 
			m_vSeries[i].minDiff = a;
		else
			m_vSeries[i].minDiff = b;
	}
	
	m_vSeries[i].minDiff = fabs(m_vSeries[i].slope);
}