#ifndef DLGOPTICAL_H
#define DLGOPTICAL_H
#include "optical.h"

class DlgOptical : public DlgOpticalBase
{
public:
    DlgOptical(long nPyrLayers, long nWinSize, long nIter, long nNeighbor, double dblSigma, long nFrameSteps, wxWindow* parent);
    virtual ~DlgOptical();
	
	// optical flow
	long getIter() { 
		wxString  str = m_textCtrl_ITER->GetValue();
		long  iter;
		str.ToLong(&iter);
		return iter;
	}
	long getNeighbor() { 
		wxString  str = m_textCtrl_NEIGHBOR->GetValue();
		long  value;
		str.ToLong(&value);
		return value;
	}
	long getPyrLayers() { 
		wxString  str = m_textCtrl_PYR_LAYERS->GetValue();
		long  value;
		str.ToLong(&value);
		return value;
	}	
	double getSigma() { 
		wxString  str = m_textCtrl_SIGMA->GetValue();
		double  value;
		str.ToDouble(&value);
		return value;
	}	
	long getWinSize() { 
		wxString  str = m_textCtrl_WIN_SIZE->GetValue();
		long  value;
		str.ToLong(&value);
		return value;
	}	
	long getFrameSteps() { 
		wxString  str = m_textCtrl_FRAME_STEPS->GetValue();
		long  value;
		str.ToLong(&value);
		return value;
	}
	
	
};
#endif // DLGOPTICAL_H
