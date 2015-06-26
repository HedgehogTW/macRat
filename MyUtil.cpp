#include "MainFrame.h"
#include "MyUtil.h"

#include <wx/msgdlg.h> 

#define _MIN_Y	-5
#define _MAX_Y	20

 
//Gnuplot gnuPlot("lines");
void _gnuplotInit(Gnuplot& gnuPlot, const char* title, double ymin, double ymax)
{
	gnuPlot.reset_all();
//	gnuPlot.cmd("set termoption noenhanced");
	gnuPlot.set_title(title);
	gnuPlot.set_grid();	
	
	if(ymin!=0 || ymax!=0) {
		gnuPlot.set_yrange(ymin, ymax);
	}//else gnuPlot.set_yrange(_MIN_Y, _MAX_Y);

}
void _gnuplotLED(Gnuplot& gnuPlot, int LED1, int LED2)
{
	if (LED1 > 0) _gnuplotVerticalLine(gnuPlot, LED1);
	if (LED2 > 0)	_gnuplotVerticalLine(gnuPlot, LED2);
}


void _gnuplotVerticalLine(Gnuplot& gnuPlot, float x, const char* dataName)
{
	if (x > 0) {
		double ymin, ymax;
		gnuPlot.getYRange(ymin, ymax);
		
		std::vector<double> vecx;
		std::vector<double> vecy;
		vecy.push_back(ymin);
		vecx.push_back(x);
		vecy.push_back(ymax);
		vecx.push_back(x);	
		gnuPlot.set_style("lines").plot_xy(vecx, vecy, 1, dataName);
	}	
}


///////////////////////////////
SortByKey_STL SortByKey_STL::instance = SortByKey_STL();
//////////////////////
void _scalingLoadPara(vector<Vec2d>& scalePara)
{
	scalePara.clear();
	FILE* fps = fopen("_scale_para.txt", "r");
	for(int i=0; i<NUM_FEATURE; i++) {
		double a, b;
		fscanf(fps, "%lf,%lf", &a, &b);
		Vec2d  v;
		v.val[0] = a;
		v.val[1] = b;
		scalePara.push_back(v);
		//gpOutput->ShowMessage("%f  %f", a, b);
	}
	fclose(fps);
}

void _scalingData(Mat& mData, vector<Vec2d>& scalePara)
{
	int cols = mData.cols;
	for(int i=0; i<cols; i++) {
		Mat mC = mData.col(i);
		double alpha, beta;
		alpha = scalePara[i].val[0];
		beta = scalePara[i].val[1];
		mC.convertTo(mC, CV_32F, alpha, beta);
	}

	//Mat mC = mData.col(cols);
	//double min, max, alpha, beta;
	//minMaxLoc(mC, &min, &max);
	//alpha = 2.0/(max-min);
	//beta = -(2*min/(max-min) +1.);
	//mC.convertTo(mC, CV_32F, alpha, beta);

}
void _scalingTraining(Mat& mData)
{
	FILE* fp = fopen("_scale_para.txt", "w");
	int cols = mData.cols;
	for(int i=0; i<cols; i++) {
		Mat mC = mData.col(i);
		//Mat mN;
		double min, max, alpha, beta;
		minMaxLoc(mC, &min, &max);
		alpha = 2.0/(max-min);
		beta = -(2*min/(max-min) +1.);
		mC.convertTo(mC, CV_32F, alpha, beta);
		//mN.copyTo(mData.col(i));
		fprintf(fp, "%f, %f\n", alpha, beta);
	}
	fclose(fp);
	MainFrame ::myMsgOutput("scale training data, save parameters in _scale_para.txt");
}


ofstream fileOutputStream;		// this must be kept somewhere global
void _redirectStandardOutputToFile ( string filePath, bool toPromptAlso )
{
	std::streambuf* sbuf = cout.rdbuf();
	if(fileOutputStream.is_open() )  fileOutputStream.close();
	fileOutputStream.open( filePath.c_str() );
	cout.rdbuf(fileOutputStream.rdbuf());
	if ( toPromptAlso )
		cout.rdbuf(sbuf);
}

void _OutputBinaryMat(cv::Mat m, char *filename)
{
	int lines = m.rows;
	int cols = m.cols;
	int i, j, channel, depth;

	depth = m.depth();
	channel = m.channels();
	if(channel !=1) 	return;

	FILE *fp = fopen(filename, "wb");
	if(fp==NULL) {
		wxMessageOutputMessageBox().Printf(_T("cannot create output file"));
		return;
	}
	switch(depth) {
		case CV_8U:
		case CV_8S:
			fwrite(m.data, sizeof(uchar), lines*cols, fp);
			break;
		case CV_16U:
		case CV_16S:
			fwrite(m.data, sizeof(short), lines*cols, fp);
			break;
		case CV_32S:				
		case CV_32F:
			fwrite(m.data, sizeof(float), lines*cols, fp);
			break;
		case CV_64F:
			fwrite(m.data, sizeof(double), lines*cols, fp);
			break;
	}


	fclose(fp);
}


void _OutputMatPoint2f(cv::Mat m, const char *filename, bool bAppend)
{
	FILE *fp;
	if(bAppend==false)
		fp = fopen(filename, "w");
	else
		fp = fopen(filename, "a");
		
	if(fp==NULL) {
		wxMessageOutputMessageBox().Printf(_T("cannot create output file"));
		return;
	}

	int rows = m.rows;
	int cols = m.cols;

	for(int y = 0; y < rows; y ++)
        for(int x = 0; x < cols; x ++)
        {
            const Point2f& fxy = m.at<Point2f>(y, x);
			fprintf(fp, "%f, %f\n", fxy.x, fxy.y);

        }

	fclose(fp);	
}
void _OutputMat(cv::Mat m, const char *filename, bool bhasComma)
{
	FILE *fp;
	fp = fopen(filename, "w");
	if(fp==NULL) {
		wxMessageOutputMessageBox().Printf(_T("cannot create output file"));
		return;
	}

	int lines = m.rows;
	int cols = m.cols;
	int i, j, channel, depth, elems;

	depth = m.depth();
	channel = m.channels();
	elems = cols*channel;
//	MainFrame:: myMsgOutput("channels %d, elements per row %d\n", channel, elems);
	
	for(i=0; i<lines; i++) {
		
		switch(depth) {
			case CV_8U: 
			{
				const uchar* p = m.ptr<uchar>(i);
				for (j = 0; j <elems; j++) {
					fprintf(fp, "%d ", p[j]);
					if (j != elems - 1 && bhasComma) fprintf(fp, ", ");
				}
				break;
			}
			case CV_8S:
			{
				const char* p = m.ptr<char>(i);
				for (j = 0; j <elems; j++) {
					fprintf(fp, "%d ", p[j]);
					if (j != elems - 1 && bhasComma) fprintf(fp, ", ");
				}
				break;
			}
			case CV_16U:
			case CV_16S:
			{
				const short* p = m.ptr<short>(i);
				for (j = 0; j <elems; j++) {
					fprintf(fp, "%d ", p[j]);
					if (j != elems - 1 && bhasComma) fprintf(fp, ", ");
				}
				break;
			}
			case CV_32S:
			{
				const int* p = m.ptr<int>(i);
				for (j = 0; j <elems; j++) {
					fprintf(fp, "%d ", p[j]);
					if (j != elems - 1 && bhasComma) fprintf(fp, ", ");
				}
				break;
			}
			case CV_32F:
			{
				const float* p = m.ptr<float>(i);
				for (j = 0; j <elems; j++) {
					fprintf(fp, "%f ", p[j]);
					if (j != elems - 1 && bhasComma) fprintf(fp, ", ");
				}
				break;
			}
			case CV_64F:
			{
				const double* p = m.ptr<double>(i);
				for (j = 0; j <elems; j++) {
					fprintf(fp, "%f ", p[j]);
					if (j != elems - 1 && bhasComma) fprintf(fp, ", ");
				}
				break;
			}
		}
		fprintf(fp, "\n");
	}
	fclose(fp);

	//MainFrame:: myMsgOutput("output file: %s, rows %d, cols %d\n", filename, lines, cols);
}

int fwrite_matrix( FILE *fout, cv::Mat &m, int xsize, int ysize, float *rt, float *ct)
{
    int j;
    int status;
    float length = ysize;

    if ((status = fwrite((char *) &length, sizeof(float), 1, fout)) != 1) {
		fprintf(stderr, "fwrite 1 returned %d\n", status);
		return (0);
    }
    fwrite((char *) ct, sizeof(float), ysize, fout);
    for (j = 0; j < xsize; j++) {
		fwrite((char *) &rt[j], sizeof(float), 1, fout);
		const float* Mi = m.ptr<float>(j);
		fwrite((char *) (Mi), sizeof(float), ysize, fout);
    }

    return (1);
}

void _OutputMatGnuplotBinData(cv::Mat m, const char *filename, int low, int high)
{
	int channel, depth;

	depth = m.depth();
	channel = m.channels();
	if(channel !=1) 	return;
	
	if(depth != CV_32F)  m.convertTo(m, CV_32F);
	
	FILE *fp;
	fp = fopen(filename, "wb");
	if(fp==NULL) {
		wxMessageOutputMessageBox().Printf(_T("cannot create output file"));
		return;
	}	
	//////////////////////////////////
    int i, j;
    float x, y;
    float *rt, *ct;
    int xsize, ysize;

/*  Create a few standard test interfaces */
	float xmin, xmax;
	float ymin, ymax;
	
	xmin = low;
	xmax = high;
	ymin = low;
	ymax = high;	
	
   	xsize = (xmax - xmin) ;
	ysize = (ymax - ymin) ;

	rt = new float [xsize];
	ct = new float [ysize];
	
	for (y = ymin, j = 0; j < ysize; j++, y += 1.0 )     ct[j] = y;
	for (x = xmin, i = 0; i < xsize; i++, x += 1.0 )     rt[i] = x;
	
	if(fwrite_matrix(fp, m, xsize, ysize, rt, ct)==0) {
		wxMessageOutputMessageBox().Printf(_T("fwrite_matrix ERROR"));
	}
	
	fclose(fp);
	
	delete [] rt;
	delete [] ct;
}
void _rgbMat2hsvMat(cv::Mat &mRGB, cv::Mat &mHSV, bool plus360)
// h: 0..360, s: 0..1, v: 0..max(r,g,b)
{
	int type = mRGB.type();
	if(type != CV_8UC3) return;

	int rows = mRGB.rows;
	int cols = mRGB.cols;

	mHSV.create(rows, cols, CV_32FC3);

	for(int y=0; y<rows; y++) {
		for(int x=0; x<cols; x++) {

			cv::Vec3b c = mRGB.at<cv::Vec3b>(y, x);
			uchar b = c.val[0];
			uchar g = c.val[1];
			uchar r = c.val[2];
			float h, s, v;
			rgb2hsv(r, g, b, h, s, v, plus360);

			cv::Vec3f hsv;
			hsv.val[0] = h;
			hsv.val[1] = s;
			hsv.val[2] = v;
			mHSV.at<cv::Vec3f>(y, x) = hsv;
		}
	}
		
}

void rgb2hsv(uchar r, uchar g, uchar b, float &h, float &s, float &v, bool plus360)
{
	struct rgb_color {
		float r, g, b;    /* Channel intensities between 0.0 and 1.0 */
	};
    float min, max;//, delta;
	float rgb_min, rgb_max;

	struct rgb_color rgb;
	rgb.r = r;
	rgb.g = g;
	rgb.b = b;

    min = r < g ? r : g;
    min = min  < b ? min  : b;

    max = r > g ? r : g;
    max = max  > b ? max  : b;

	v = max;    
	if(v ==0) {
		h = s = 0;
		return;
	}

	/* Normalize value to 1 */
	rgb.r /= v;
	rgb.g /= v;
	rgb.b /= v;
	rgb_min = MIN3(rgb.r, rgb.g, rgb.b);
	rgb_max = MAX3(rgb.r, rgb.g, rgb.b);

	// <<compute saturation>>=
	s = rgb_max - rgb_min;
	if (s == 0) {
		h = 0;
		return ;
	}

	/* Normalize saturation to 1 */
	rgb.r = (rgb.r - rgb_min)/(rgb_max - rgb_min);
	rgb.g = (rgb.g - rgb_min)/(rgb_max - rgb_min);
	rgb.b = (rgb.b - rgb_min)/(rgb_max - rgb_min);
	rgb_min = MIN3(rgb.r, rgb.g, rgb.b);
	rgb_max = MAX3(rgb.r, rgb.g, rgb.b);

	/* Compute hue */
	if (rgb_max == rgb.r) {
		h = 0.0 + 60.0*(rgb.g - rgb.b);
		if (h < 0.0 && plus360) {
			h += 360.0;
		}
	} else if (rgb_max == rgb.g) {
		h = 120.0 + 60.0*(rgb.b - rgb.r);
	} else /* rgb_max == rgb.b */ {
		h = 240.0 + 60.0*(rgb.r - rgb.g);
	}
}
