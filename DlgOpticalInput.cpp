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

void DlgOpticalInput::setVerticalLine(bool bLED, bool bPinna, bool bVerLine, double x)
{
	m_checkBoxLED->SetValue(bLED);
	m_checkBoxPinna->SetValue(bPinna);
	m_checkBoxVerLine->SetValue(bVerLine);
	
	wxString  str1;
	str1 << x;
	*m_textCtrlVerLine << str1;
}

void DlgOpticalInput::setSeriesLine(bool bEyeMove, bool bGrayDiff, bool bAdjDiff, bool bOptical, bool bOpticalPDF, bool bAccumulate)
{
	m_checkBoxEyeMove->SetValue(bEyeMove);
	m_checkBoxGrayDiff->SetValue(bGrayDiff);
	m_checkBoxAdjDiff->SetValue(bAdjDiff);
	m_checkBoxOptical->SetValue(bOptical);	
	m_checkBoxOpticalPDF->SetValue(bOpticalPDF);	
	m_radioButtonAccumu->SetValue(bAccumulate);	
	m_radioButtonInstan->SetValue(! bAccumulate);	
}

void DlgOpticalInput::setYRange(double min, double max, long szROI)
{
	wxString  str1, str2, str3;
	str1 << min;
	*m_textCtrlYmin << str1;
	
	str2 << max;
	*m_textCtrlYmax << str2;
	
	str3 << szROI;
	*m_textCtrlROISize << str3;
}

void DlgOpticalInput::getVerticalLine(bool& bLED, bool& bPinna, bool& bVerLine, double& x)
{
	bLED = m_checkBoxLED->GetValue();
	bPinna = m_checkBoxPinna->GetValue();
	bVerLine = m_checkBoxVerLine->GetValue();
	
	wxString  str = m_textCtrlVerLine->GetValue();
	double  value;
	str.ToDouble(&value);
	x = value;
}

void DlgOpticalInput::getSeriesLine(bool& bEyeMove, bool& bGrayDiff, bool& bAdjDiff, bool& bOptical, bool& bOpticalPDF, bool& bAccumulate)
{
	bEyeMove = m_checkBoxEyeMove->GetValue();
	bGrayDiff = m_checkBoxGrayDiff->GetValue();
	bAdjDiff = m_checkBoxAdjDiff->GetValue();
	bOptical = m_checkBoxOptical->GetValue();
	bOpticalPDF = m_checkBoxOpticalPDF->GetValue();
	bAccumulate = m_radioButtonAccumu->GetValue();
}	

void DlgOpticalInput::getYRange(double& min, double& max, long& szROI)
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
}