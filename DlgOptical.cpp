#include "DlgOptical.h"

DlgOptical::DlgOptical(long nPyrLayers, long nWinSize, long nIter, long nNeighbor, double dblSigma, long nFrameSteps, wxWindow* parent)
    : DlgOpticalBase(parent)
{
	*m_textCtrl_PYR_LAYERS << nPyrLayers;
	*m_textCtrl_WIN_SIZE << nWinSize;
	*m_textCtrl_ITER << nIter;
	*m_textCtrl_NEIGHBOR << nNeighbor;
	*m_textCtrl_SIGMA << dblSigma;
	*m_textCtrl_FRAME_STEPS << nFrameSteps;
}

DlgOptical::~DlgOptical()
{
}

