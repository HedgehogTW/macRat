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
	void setOptions(bool bOpticalPDF, bool bOpFlowV1, bool bSave, int refSignal);
	void setYRange(double min, double max, long szROIEar, long szROIBelly, long referFrame);
	void setGain(double gainHead, double gainBelly, double xSD);
	
	void getVerticalLine(bool& bLED, bool& bBigHead, bool& bUserLED2, int& nLED2, bool& bVerLine, double& x);
	void getSeriesLine(bool& bEyeMove, bool& bEar, bool& bGrayDiff, bool& bBelly);
	void getOptions(bool& bOpticalPDF, bool& bOpFlowV1, bool& bSave, int& refSignal, bool& bDiffSignal);
	void getYRange(double& min, double& max, long& szROIEar, long& szROIBelly, long& referFrame);
	
	void getGain(double& gainHead, double &gainBelly, double &xSD);
	
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
    virtual void OnGenRefFrame(wxCommandEvent& event);
};
#endif // DLGOPTICALINPUT_H
