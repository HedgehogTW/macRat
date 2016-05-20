#ifndef DLGOPTICALINPUT_H
#define DLGOPTICALINPUT_H
#include "dlg_optical_input_base.h"

class DlgOpticalInput : public DlgOpticalInputBase
{
public:
    DlgOpticalInput(long nFrameSteps, double threshold, wxWindow* parent);
    virtual ~DlgOpticalInput();
	
	void setVerticalLine(bool bLED, bool bBigHead, bool bUserLED2, int nLED2, bool bVerLine, double x=0);
	void setSeriesLine(bool bEyeMove, bool bEar, bool bGrayDiff, bool bBelly);
	void setOptions(bool bOpticalPDF, bool bOpFlowV1, bool bSave, bool bSaveSignalPlot, bool bShowPeaks, bool bShowAxisLabel, int refSignal);
	void setAmpYRange(double min, double max, double ysnd, long szROIEar, long szROIBelly, long referFrame);
	void setIntervalYRange(double min, double max, double ysnd);
	void setGain(double gainHead, double gainBelly, double xSD);
	void setSmooth(double smoothWidthEar, double smoothWidthBelly);
		
	void getVerticalLine(bool& bLED, bool& bBigHead, bool& bUserLED2, int& nLED2, bool& bVerLine, double& x);
	void getSeriesLine(bool& bEyeMove, bool& bEar, bool& bGrayDiff, bool& bBelly);
	void getOptions(bool& bOpticalPDF, bool& bOpFlowV1, bool& bSave, bool& bSaveSignalPlot, bool& bShowPeaks, bool& bShowAxisLabel, int& refSignal);
	void getAmpYRange(double& min, double& max, double& ysnd, long& szROIEar, long& szROIBelly, long& referFrame);
	void getIntervalYRange(double& min, double& max, double& ysnd);
	void getGain(double& gainHead, double &gainBelly, double &xSD);
	void getSmooth(double& smoothWidthEar, double& smoothWidthBelly);
	
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
protected:

};
#endif // DLGOPTICALINPUT_H
