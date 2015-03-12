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

