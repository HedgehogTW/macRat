
#define _USE_MATH_DEFINES
#include <FLOAT.H>
#include <math.h>
#include <stdlib.h>
#include "KDE.h"
#include <gsl/gsl_math.h>

CKDE::CKDE(double pdf[], double output[], int nBin)
{
	m_pdf= new double [nBin];
	m_pdfInv = new double [nBin];
	m_output = output;
	m_nBin = nBin;
	m_offset = 0;

	double min = DBL_MAX;
	double max = DBL_MIN;
	for(int i=0; i<nBin; i++) {
		if(pdf[i] < min)  min = pdf[i];
		if(pdf[i] > max)  max = pdf[i];
	}

	for(int i=0; i<nBin; i++) {
		m_pdf[i] = pdf[i];//* 1000;
		m_pdfInv[i] = max +2 - pdf[i];
	}

	if(min >=0) {
		m_offset = 0;
	}else {
		m_offset = -min ;
		for(int i=0; i<nBin; i++) 
			m_pdf[i] = m_pdf[i] + m_offset;
	}
	//FILE *fp = fopen("_pdf.txt", "wt");
	//int i;
	//for(i=0; i<m_nBin; i++) 
	//	fprintf(fp, "%f  %f\n", m_pdf[i], 1./(m_pdf[i]+m_inverse));
	//fclose(fp);

	m_bFindMax = true;

	//gpMsgbar->ShowMessage("KDE: min= %f, offset = %f\n", min, m_offset); 
}

CKDE::~CKDE(void)
{
	delete [] m_pdf;
	delete [] m_pdfInv;
}

double CKDE::MeanShift_Forward(bool bFindMax, double x, double& h, double alpha, double eplison, int maxIter, bool bNeighbor) 
// use Epanechnikov Kernel
{
	int  iter ;
	double diffX, oldX;
	double lowerBound;
	int		oldSgn, sgn, oscillation;

	
	iter = 0;
	oscillation = oldSgn = sgn = 0;
	do{
		oldSgn = sgn ;
		oldX = x;
		lowerBound = -(1-1.*exp(-1.*iter/alpha));
		x = WeightedMean(x, h, lowerBound, bFindMax);

		diffX = x - oldX;
		sgn = GSL_SIGN(diffX);
		oscillation += abs(sgn - oldSgn)/2;

//		TRACE("[%2d] diff X %.3f, new X = %.3f, h=%.3f, low=%.3f, FindMax %d\n", iter, diffX, x, h, lowerBound, bFindMax);
		if(fabs(diffX) < eplison || oscillation >20) {
			oscillation = 0;
//			break;
			h *= 0.85;
			if(h < 2) break;
		}
		iter++;
	}while(iter <maxIter);

//	gpMsgbar->ShowMessage("max_min %d, find (%.3f, %.3f) ==>", bFindMax, x,  m_pdf[(int)(x+0.5)]);
	// neighborhood search
	if(bNeighbor) {
		int i, start, end;
		start = (int)(x-h - 1);
		if(start < 0) start = 0;
		end = (int)(x+h+1);
		if(end >= m_nBin) end = m_nBin -1;

		for(i=start; i<=end; i++) {
			if(bFindMax) 
				if(m_pdf[i] > m_pdf[(int)(x+0.5)]) x =  i;
			if(! bFindMax) 
				if(m_pdf[i] < m_pdf[(int)(x+0.5)]) x =  i;
		}
	}
//	gpMsgbar->ShowMessage(" (%.3f, %.3f) \n", x,  m_pdf[(int)(x+0.5)]);

//	gpMainDlg->ShowMessage("best position %d\n", bestPos);

	return x;
}
double CKDE::WeightedMean(double x, double h, double lowerBound, bool bFindMax)
{
	double dist, g, numerator, denominator, mean, num, deno;

	do{
		numerator = denominator = 0;		
		for(int i=0; i<m_nBin; i++) {
			dist = (i-x)/h;
	//		g = g_Epanechnikov(dist, lowerBound);
			//g = g_Gaussian(dist, lowerBound, sigma);
			g = g_Gaussian(dist, lowerBound, h);
			if(bFindMax) {
				num = i * g * m_pdf[i];
				deno = g * m_pdf[i];
			}else {
	//			g *= 1./(m_pdf[i]+m_inverse);
				//g *= 1 -m_pdf[i];
				num = i * g * m_pdfInv[i];
				deno = g * m_pdfInv[i];
			}

			numerator += num;
			denominator += deno;
		}
		if(denominator==0) x++;
	}while(denominator==0);

	mean = numerator / denominator;

	return mean;
}

void CKDE::KernelDensityEstimation(MSKernel k, double h)
{
	int i, x, nh, n;
	double f, c, coef, norm2;

//	n = m_amount;
	n = 0;
	for(i=0; i<m_nBin; i++)	{	
		m_output[i] = 0;
		n += m_pdf[i];
	}


	nh = n * h;
	c = 1;

	//coef = c/nh;
	coef = c/h;

	for(x=0; x<m_nBin; x++) {
		f = 0;
		for(i=0; i<m_nBin; i++) {
			norm2 = ((x-i)/h) * ((x-i)/h);
			f += m_pdf[i] * Kernel(k, norm2) ;
		}
		m_output[x] = f* coef;;
	}
	if(	m_offset!=0)
		for(x=0; x<m_nBin; x++) 
			m_output[x] -= m_offset;

}

double CKDE::Kernel(MSKernel k, double norm2)
{
	double  val;
	switch(k) {
		case Gaussian:
			val = KernelGaussian(norm2);
			break;
		case Epanechnikov:
			val = KernelEpanechnikov(norm2);
			break;
	}
	return val;
}

double CKDE::KernelEpanechnikov(double norm2)
{
	double k ;

	if(norm2 <= 1)
		k = (1- norm2);
	else k = 0;

	return k;
}
double CKDE::g_Epanechnikov(double norm2)
// g = -k'
{
	double g ;

	if(norm2 <= 1)
		g =  1;
	else g = 0;

	return g;
}

double CKDE::g_Epanechnikov(double dist, double lowerBound)
// g = -k'
{
	double g ;

	if(dist >=0)
		if(dist <= 1)
			g =  1;
		else g = 0;
	else
		if(dist >= lowerBound)
			g =  1;
		else g = 0;


	return g;
}

double CKDE::KernelGaussian(double norm2)
{
	double k ;

	k = exp(-norm2/2.)/sqrt(2*M_PI);

	return k;

}
double CKDE::g_Gaussian(double norm2)
// g = -k'
{
	double g ;

	g = exp(-0.5 * norm2) * 0.5;

	return g;
}
double CKDE::g_Gaussian(double dist, double lowerBound, double sigma)
// g = -k'
{
	double g, norm2, norm ;

	norm = fabs(dist);
	norm2 = norm * norm;

	//if(dist >=0)
	//	if(dist <= 1){
	//		//g = exp(- norm2 / (2*sigma2)) * norm/sigma2 ;
	//		g = -exp(- norm2 /2) * (-norm)/sigma ;
	//		g /= sqrt(2*M_PI)*sigma;
	//	}else g = 0;
	//else
	//	if(dist >= lowerBound){
	//		//g = exp(- norm2 / (2*sigma2)) * norm/sigma2 ;
	//		g = -exp(- norm2 /2) * (-norm)/sigma ;
	//		g /= sqrt(2*M_PI)*sigma;
	//	}else g = 0;

	g = -exp(- norm2 /2) * (-norm)/sigma ;
	g /= sqrt(2*M_PI)*sigma;
	return g;
}
