#include "DlgOpticalInput.h"

DlgOpticalInput::DlgOpticalInput(long nFrameSteps, double threshold, wxWindow* parent)
    : DlgOpticalInputBase(parent)
{
	m_textCtrlThreshold->Clear();
	m_textCtrlFrameSteps->Clear();
	*m_textCtrlThreshold << threshold;
	*m_textCtrlFrameSteps << nFrameSteps;
}

DlgOpticalInput::~DlgOpticalInput()
{
}

