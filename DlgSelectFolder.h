#ifndef DLGSELECTFOLDER_H
#define DLGSELECTFOLDER_H
#include "dlg_select_folder.h"

class DlgSelectFolder : public DlgSelectFolderBase
{
public:
    DlgSelectFolder(wxWindow* parent, wxString& strIniDir);
    virtual ~DlgSelectFolder();
	
	void	setParam(double s, bool bCheckOnlyFirst);
	void	getParam(double& s, bool &bCheckOnlyFirst);
	
	bool	showSymbol() { return m_checkBoxSymbol->GetValue(); }
	void	setShowSymbol(bool b) { m_checkBoxSymbol->SetValue(b);	}
	
	wxString	m_strDir;
protected:
    virtual void OnOK(wxCommandEvent& event);
    virtual void OnButtonSelectFolder(wxCommandEvent& event);
};
#endif // DLGSELECTFOLDER_H
