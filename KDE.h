#pragma once

class CKDE
{
public:
enum MSKernel{ Gaussian, Epanechnikov }  ;
	CKDE(double pdf[], double output[], int nBin);
	~CKDE(void);
	void KernelDensityEstimation(MSKernel k, double h);
	double MeanShift_Forward(bool bFindMax,double x, double &h, double alpha,double eplison, int maxIter, bool bNeighbor) ;
	double WeightedMean(double x, double h, double low, bool bFindMax);
	double WeightedMean_DWave(int start, int end, double x, double h, double sigma);

private:
	double Kernel(MSKernel k, double norm2);
	double KernelGaussian(double norm2);
	double KernelEpanechnikov(double norm2);
	double g_Epanechnikov(double norm2);
	double g_Epanechnikov(double dist, double lowerBound);
	double g_Gaussian(double norm2);
	double g_Gaussian(double dist, double lowerBound, double sigma);
private:
	bool		m_bFindMax;
	double*		m_pdf;
	double*		m_pdfInv;
	double*		m_output;
	int			m_nBin;
	double		m_offset;


};
