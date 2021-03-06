#include "DlgOpticalInput.h"

DlgOpticalInput::DlgOpticalInput(long nFrameSteps, double threshold, wxWindow* parent)
    : DlgOpticalInputBase(parent)
{
	m_textCtrlThreshold->Clear();
	m_textCtrlFrameSteps->Clear();

	wxString  str1;
	str1 << nFrameSteps;
	*m_textCtrlFrameSteps << str1;
	
	wxString  str2;
	str2 << threshold;
	*m_textCtrlThreshold << str2;
}

DlgOpticalInput::~DlgOpticalInput()
{
}

void DlgOpticalInput::setVerticalLine(bool bLED, bool bBigHead, bool bUserLED2, int nLED2, bool bVerLine, double x)
{
	m_checkBoxLED->SetValue(bLED);
	m_checkBoxBigHead->SetValue(bBigHead);
	m_checkBoxLED2->SetValue(bUserLED2);
	m_checkBoxVerLine->SetValue(bVerLine);
	
	wxString  str1, str2;
	str1 << x;
	*m_textCtrlVerLine << str1;
	
	str2 << nLED2;
	*m_textCtrlLED2 << str2;	
}

void DlgOpticalInput::setSeriesLine(bool bEyeMove, bool bEar, bool bGrayDiff, bool bBelly)
{
	m_checkBoxEyeMove->SetValue(bEyeMove);
	m_checkBoxGrayDiff->SetValue(bGrayDiff);
    m_checkBoxEar->SetValue(bEar);
	m_checkBoxBelly->SetValue(bBelly);
}
void DlgOpticalInput::setOptions(bool bOpticalPDF, bool bOpFlowV1, bool bSave, bool bSaveSignalPlot, int refSignal)
{
	m_checkBoxOpticalPDF->SetValue(bOpticalPDF);	
	m_radioOpV1->SetValue(bOpFlowV1);	
	m_radioOpV2->SetValue(! bOpFlowV1);
	m_checkBoxSaveFlow->SetValue(bSave);
	m_checkBoxSaveSignalPlot->SetValue(bSaveSignalPlot);
	switch(refSignal) {
		case 0:
			m_radioButtonRefBelly->SetValue(true);
			m_radioButtonRefHead->SetValue(false);
			m_radioButtonRefEar->SetValue(false);
			break;
		case 1:
			m_radioButtonRefBelly->SetValue(false);
			m_radioButtonRefHead->SetValue(true);
			m_radioButtonRefEar->SetValue(false);
			break;		
		case 2:
			m_radioButtonRefBelly->SetValue(false);
			m_radioButtonRefHead->SetValue(false);
			m_radioButtonRefEar->SetValue(true);
			break;		
	}
}
void DlgOpticalInput::setYRange(double min, double max, long szROIEar, long szROIBelly, long referFrame)
{
	wxString  str1, str2, str3, str4, str5;
	str1 << min;
	*m_textCtrlYmin << str1;
	
	str2 << max;
	*m_textCtrlYmax << str2;
	
	str3 << szROIEar;
	*m_textCtrlROIEar << str3;
 
	str4 << szROIBelly;
	*m_textCtrlROIBelly << str4;
   
 	str5 << referFrame;
	*m_textCtrlReferFrame << str5;   
}

void DlgOpticalInput::setGain(double gainHead, double gainBelly, double xSD)
{
	wxString  str1, str2, str3;
	str1 << gainHead;
	*m_textCtrlHeadGain << str1;
	
	str2 << gainBelly;
	*m_textCtrlBellyGain << str2;
	
	str3 << xSD;
	*m_textCtrlXSD << str3;	
}

void DlgOpticalInput::getVerticalLine(bool& bLED, bool& bBigHead, bool& bUserLED2, int& nLED2, bool& bVerLine, double& x)
{
	bLED = m_checkBoxLED->GetValue();
	bBigHead = m_checkBoxBigHead->GetValue();
	bUserLED2 = m_checkBoxLED2->GetValue();
	bVerLine = m_checkBoxVerLine->GetValue();
	
	wxString  str = m_textCtrlVerLine->GetValue();
	double  value;
	str.ToDouble(&value);
	x = value;
	
	if(bUserLED2) {
		long a;
		str = m_textCtrlLED2->GetValue();
		str.ToLong(&a);
		nLED2 = a;
	}
}

void DlgOpticalInput::getSeriesLine(bool& bEyeMove, bool& bEar, bool& bGrayDiff, bool& bBelly)
{
	bEyeMove = m_checkBoxEyeMove->GetValue();
	bGrayDiff = m_checkBoxGrayDiff->GetValue();
    bEar = m_checkBoxEar->GetValue();
	bBelly = m_checkBoxBelly->GetValue();
}	
void DlgOpticalInput::getOptions(bool& bOpticalPDF, bool& bOpFlowV1, bool& bSave, bool& bSaveSignalPlot, int& refSignal)
{
	bOpticalPDF = m_checkBoxOpticalPDF->GetValue();
	bOpFlowV1 = m_radioOpV1->GetValue();
	bSave = m_checkBoxSaveFlow->GetValue();
	bSaveSignalPlot = m_checkBoxSaveSignalPlot->GetValue();
	
	bool b1 =  m_radioButtonRefBelly->GetValue();
	bool b2 =  m_radioButtonRefHead->GetValue();
	bool b3 =  m_radioButtonRefEar->GetValue();
    
	if(b1) refSignal = 0;
	else if(b2) refSignal = 1;
	else if(b3) refSignal = 2;
}	

void DlgOpticalInput::getYRange(double& min, double& max, long& szROIEar, long& szROIBelly, long& referFrame)
{
	wxString  str = m_textCtrlYmin->GetValue();
	double  value;
	str.ToDouble(&value);	
	min = value;
	
	str = m_textCtrlYmax->GetValue();
	str.ToDouble(&value);	
	max = value;
	
	long  a;
	str = m_textCtrlROIEar->GetValue();
	str.ToLong(&a);	
	szROIEar = a;	
    
	str = m_textCtrlROIBelly->GetValue();
	str.ToLong(&a);	
	szROIBelly = a;	
    
   	str = m_textCtrlReferFrame->GetValue();
	str.ToLong(&a);	
	referFrame = a;	 
}

void DlgOpticalInput::getGain(double& gainHead, double &gainBelly, double &xSD)
{
	wxString  str = m_textCtrlHeadGain->GetValue();
	double  value;
	str.ToDouble(&value);	
	gainHead = value;	

	str = m_textCtrlBellyGain->GetValue();
	str.ToDouble(&value);	
	gainBelly = value;	

	str = m_textCtrlXSD->GetValue();
	str.ToDouble(&value);	
	xSD = value;	
}
void DlgOpticalInput::OnGenRefFrame(wxCommandEvent& event)
{
}
