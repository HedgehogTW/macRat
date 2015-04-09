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

void DlgOpticalInput::setVerticalLine(bool bLED, bool bRefLine, bool bPinna, bool bVerLine, double x)
{
	m_checkBoxLED->SetValue(bLED);
    m_checkBoxRef->SetValue(bRefLine);
	m_checkBoxPinna->SetValue(bPinna);
	m_checkBoxVerLine->SetValue(bVerLine);
	
	wxString  str1;
	str1 << x;
	*m_textCtrlVerLine << str1;
}

void DlgOpticalInput::setSeriesLine(bool bEyeMove, bool bEar, bool bGrayDiff, bool bAbdo, bool bOptical, bool bOpticalPDF, bool bAccumulate)
{
	m_checkBoxEyeMove->SetValue(bEyeMove);
	m_checkBoxGrayDiff->SetValue(bGrayDiff);
    m_checkBoxEar->SetValue(bEar);
	m_checkBoxAbdo->SetValue(bAbdo);
	m_checkBoxOptical->SetValue(bOptical);	
	m_checkBoxOpticalPDF->SetValue(bOpticalPDF);	
	m_radioButtonAccumu->SetValue(bAccumulate);	
	m_radioButtonInstan->SetValue(! bAccumulate);	
}

void DlgOpticalInput::setYRange(double min, double max, long szROI, long referFrame)
{
	wxString  str1, str2, str3, str4;
	str1 << min;
	*m_textCtrlYmin << str1;
	
	str2 << max;
	*m_textCtrlYmax << str2;
	
	str3 << szROI;
	*m_textCtrlROISize << str3;
    
 	str4 << referFrame;
	*m_textCtrlReferFrame << str4;   
}

void DlgOpticalInput::setGain(double gainEye, double gainPDF)
{
	wxString  str1, str2;
	str1 << gainEye;
	*m_textCtrlEyeGain << str1;
	
	str2 << gainPDF;
	*m_textCtrlPDFGain << str2;
}

void DlgOpticalInput::getVerticalLine(bool& bLED, bool& bRefLine, bool& bPinna, bool& bVerLine, double& x)
{
	bLED = m_checkBoxLED->GetValue();
    bRefLine = m_checkBoxRef->GetValue();
	bPinna = m_checkBoxPinna->GetValue();
	bVerLine = m_checkBoxVerLine->GetValue();
	
	wxString  str = m_textCtrlVerLine->GetValue();
	double  value;
	str.ToDouble(&value);
	x = value;
}

void DlgOpticalInput::getSeriesLine(bool& bEyeMove, bool& bEar, bool& bGrayDiff, bool& bAbdo, bool& bOptical, bool& bOpticalPDF, bool& bAccumulate)
{
	bEyeMove = m_checkBoxEyeMove->GetValue();
	bGrayDiff = m_checkBoxGrayDiff->GetValue();
    bEar = m_checkBoxEar->GetValue();
	bAbdo = m_checkBoxAbdo->GetValue();
	bOptical = m_checkBoxOptical->GetValue();
	bOpticalPDF = m_checkBoxOpticalPDF->GetValue();
	bAccumulate = m_radioButtonAccumu->GetValue();
}	

void DlgOpticalInput::getYRange(double& min, double& max, long& szROI, long& referFrame)
{
	wxString  str = m_textCtrlYmin->GetValue();
	double  value;
	str.ToDouble(&value);	
	min = value;
	
	str = m_textCtrlYmax->GetValue();
	str.ToDouble(&value);	
	max = value;
	
	long  a;
	str = m_textCtrlROISize->GetValue();
	str.ToLong(&a);	
	szROI = a;	
    
   	str = m_textCtrlReferFrame->GetValue();
	str.ToLong(&a);	
	referFrame = a;	 
}

void DlgOpticalInput::getGain(double& gainEye, double &gainPDF)
{
	wxString  str = m_textCtrlEyeGain->GetValue();
	double  value;
	str.ToDouble(&value);	
	gainEye = value;	

	str = m_textCtrlPDFGain->GetValue();
	str.ToDouble(&value);	
	gainPDF = value;	

}