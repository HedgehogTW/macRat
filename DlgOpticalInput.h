#ifndef DLGOPTICALINPUT_H
#define DLGOPTICALINPUT_H
#include "dlg_optical_input_base.h"

class DlgOpticalInput : public DlgOpticalInputBase
{
public:
    DlgOpticalInput(long nFrameSteps, double threshold, wxWindow* parent);
    virtual ~DlgOpticalInput();
	
	void setVerticalLine(bool bLED, bool bPinna, bool bVerLine, double x=0);
	void setSeriesLine(bool bEyeMove, bool bEarGray, bool bEarOptical, bool bEarOpticalPDF, bool bAccumulate);
	void setYRange(int min, int max);

	void getVerticalLine(bool& bLED, bool& bPinna, bool& bVerLine, double& x);
	void getSeriesLine(bool& bEyeMove, bool& bEarGray, bool& bEarOptical, bool& bEarOpticalPDF, bool& bAccumulate);
	void getYRange(int& min, int& max);
	
	double getThreshold() { 
		wxString  str = m_textCtrlThreshold->GetValue();
		double  value;
		str.ToDouble(&value);
		return value;
	}
	
	long getFrameSteps() { 
		wxString  str = m_textCtrlFrameSteps->GetValue();
		long  value;
		str.ToLong(&value);
		return value;
	}	
};
#endif // DLGOPTICALINPUT_H
