#ifndef DLGCYCLEOCCUR_H
#define DLGCYCLEOCCUR_H
#include "dlg_cycleOccur_base.h"
#include <vector>

using namespace std;

struct CycleItem{	
	int rat;
	int db;
	int ampItval;
	int cycle;
};

class DlgCycleOccur : public DlgCycleOccurBase
{
public:
    DlgCycleOccur(wxWindow* parent, wxString& strInitFilename);
    virtual ~DlgCycleOccur();
	int readData();
	void createHistogram(wxString filename);
	
	wxString	m_strFilename;
	wxUniChar 	m_sep;
	wxString 	m_strParentPath;
	vector<CycleItem>  m_Data;
	vector<int>  m_histo;
	int			m_maxCycle;
protected:
    virtual void OnGroupAmpIntvl(wxCommandEvent& event);
    virtual void OnGroupIncDec(wxCommandEvent& event);
    virtual void OnGroupSound(wxCommandEvent& event);
    virtual void OnButtonBrowse(wxCommandEvent& event);
};
#endif // DLGCYCLEOCCUR_H
