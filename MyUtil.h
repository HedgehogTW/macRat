#if !defined(__MY_UTIL__)
#define __MY_UTIL__

#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "Rat.h"

using namespace cv;

#define MIN3(x,y,z)  ((y) <= (z) ? \
                         ((x) <= (y) ? (x) : (y)) \
                     : \
                         ((x) <= (z) ? (x) : (z)))

#define MAX3(x,y,z)  ((y) >= (z) ? \
                         ((x) >= (y) ? (x) : (y)) \
                     : \
                         ((x) >= (z) ? (x) : (z)))

using namespace std;
	
void	_scalingTraining(Mat& mTrainData);
void	_scalingData(Mat& mData, vector<Vec2d>& scalePara);
void	_scalingLoadPara(vector<Vec2d>& scalePara);

void _redirectStandardOutputToFile ( string filePath, bool toPromptAlso );
void _OutputMat(cv::Mat m, char *filename, bool bAppend=false);
void _OutputBinaryMat(cv::Mat m, char *filename);
void _rgbMat2hsvMat(cv::Mat &mRGB, cv::Mat &mHSV, bool plus360);
void rgb2hsv(uchar r, uchar g, uchar b, float &h, float &s, float &v, bool plus360);

/// TYPE CASTING
template<class T1, class T2>
T1* type_cast(T2* data, int sz)
{
	T1* out = new T1[sz];

	for( int i=0; i<sz; i++ )
		out[i] = (T1)data[i];

	return out;
}

template <class T>  
inline int	 OtsuThresholding(T* pBuf, int szBuf, int bin)
{
	double	*histo;
	int		i;
	double  uT = 0;               // total mean level
	double  w = 0;                // first order cumulative
	double  u = 0;                // second order cumulative
	double  topItem, ratio, max;  // working variables
	int		thresh = 0;

	// create histogram
	histo = new double[bin+1];	// 1..bin
	memset(histo, 0, (bin+1)*sizeof(double) );
	//ZeroMemory(histo, (bin+1)*sizeof(double) );
	for(i=0; i<szBuf; i++)	histo[pBuf[i]+1]++;

	uT = 0;
	for (i=1; i<=bin; i++) {
		histo[i] = histo[i] / szBuf;
		uT+=(i*histo[i]);
	}

	// Find optimal threshold value
	w = u = 0;
	max = -99999;
	for (i=1; i<=bin; i++) {
		w+=histo[i];
		u+=(i*histo[i]);
		topItem = (uT * w - u);
		ratio = (topItem * topItem) / ( w * (1.0-w) );
		if (ratio > max) {
			max = ratio;
			thresh = i;
		}
	}

	thresh --;

	delete [] histo;

	return thresh;
}

//////////////////////////////////////

//namespace
//{
//IMPLEMENT_PARAM_CLASS(IsGreaterThan, bool)
//IMPLEMENT_PARAM_CLASS(InputSize, int)
//IMPLEMENT_PARAM_CLASS(SortMethod, int)


typedef Vec<float, 5> Vec5f;
typedef Vec<float, 7> Vec7f;
typedef Vec<float, 8> Vec8f;
typedef Vec<float, 9> Vec9f;

template<class T>
struct KV_CVTYPE{ static int toType() {return 0;} };

template<> struct KV_CVTYPE<int>  { static int toType() {return CV_32SC1;} };
template<> struct KV_CVTYPE<float>{ static int toType() {return CV_32FC1;} };
template<> struct KV_CVTYPE<Vec2i>{ static int toType() {return CV_32SC2;} };
template<> struct KV_CVTYPE<Vec2f>{ static int toType() {return CV_32FC2;} };
template<> struct KV_CVTYPE<Vec5f>{ static int toType() {return CV_32FC(5);} };
template<> struct KV_CVTYPE<Vec6f>{ static int toType() {return CV_32FC(6);} };
template<> struct KV_CVTYPE<Vec7f>{ static int toType() {return CV_32FC(7);} };
template<> struct KV_CVTYPE<Vec8f>{ static int toType() { return CV_32FC(8); } };
template<> struct KV_CVTYPE<Vec9f>{ static int toType() { return CV_32FC(9); } };

template<class key_type, class val_type>
bool kvgreater(pair<key_type, val_type> p1, pair<key_type, val_type> p2)
{
    return p1.first > p2.first;
}

template<class key_type, class val_type>
bool kvless(pair<key_type, val_type> p1, pair<key_type, val_type> p2)
{
    return p1.first < p2.first;
}

template<class key_type, class val_type>
void toKVPair(
    MatConstIterator_<key_type> kit,
    MatConstIterator_<val_type> vit,
    int vecSize,
    vector<pair<key_type, val_type> >& kvres
    )
{
    kvres.clear();
    for(int i = 0; i < vecSize; i ++)
    {
        kvres.push_back(make_pair(*kit, *vit));
        ++kit;
        ++vit;
    }
}

template<class key_type, class val_type>
void kvquicksort(Mat& keys, Mat& vals, bool isGreater = false)
{
    vector<pair<key_type, val_type> > kvres;
    toKVPair(keys.begin<key_type>(), vals.begin<val_type>(), keys.rows, kvres);

    if(isGreater)
    {
        std::sort(kvres.begin(), kvres.end(), kvgreater<key_type, val_type>);
    }
    else
    {
        std::sort(kvres.begin(), kvres.end(), kvless<key_type, val_type>);
    }
    key_type * kptr = keys.ptr<key_type>();
    val_type * vptr = vals.ptr<val_type>();
    for(int i = 0; i < keys.rows; i ++)
    {
        kptr[i] = kvres[i].first;
        vptr[i] = kvres[i].second;
    }
}

class SortByKey_STL
{
public:
    static void sort(cv::Mat& keys, cv::Mat& vals, bool is_gt) {
		int cols = vals.cols;
		int channels = vals.channels();
		if (cols == 1) {
			instance.quick_sorters[keys.type()][vals.type()](keys, vals, is_gt);
		}
		else {
			vals = vals.reshape(channels*cols);
			instance.quick_sorters[keys.type()][vals.type()](keys, vals, is_gt);
			vals = vals.reshape(channels);
		}
	}
private:
    typedef void (*quick_sorter)(cv::Mat&, cv::Mat&, bool);
    quick_sorter quick_sorters[CV_32FC(9)][CV_32FC(9)];  // ³Ì¤jªºtype­È
    static SortByKey_STL instance;

    SortByKey_STL() 
	{
		memset(instance.quick_sorters, 0, sizeof(quick_sorters));
	#define NEW_SORTER(KT, VT) \
		instance.quick_sorters[KV_CVTYPE<KT>::toType()][KV_CVTYPE<VT>::toType()] = kvquicksort<KT, VT>;

		NEW_SORTER(int, int);
		NEW_SORTER(int, Vec2i);
		NEW_SORTER(int, float);
		NEW_SORTER(int, Vec2f);
		NEW_SORTER(int, Vec5f);
		NEW_SORTER(int, Vec6f);
		NEW_SORTER(int, Vec7f);
		NEW_SORTER(int, Vec8f);
		NEW_SORTER(int, Vec9f);

		NEW_SORTER(float, int);
		NEW_SORTER(float, Vec2i);
		NEW_SORTER(float, float);
		NEW_SORTER(float, Vec2f);
		NEW_SORTER(float, Vec5f);
		NEW_SORTER(float, Vec6f);
		NEW_SORTER(float, Vec7f);
		NEW_SORTER(float, Vec8f);
		NEW_SORTER(float, Vec9f);
	#undef NEW_SORTER
	}

};


#endif