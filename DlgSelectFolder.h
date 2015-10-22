#ifndef DLGSELECTFOLDER_H
#define DLGSELECTFOLDER_H
#include "dlg_select_folder.h"

class DlgSelectFolder : public DlgSelectFolderBase
{
public:
    DlgSelectFolder(wxWindow* parent);
    virtual ~DlgSelectFolder();
	
	wxString	m_strDir;
protected:
    virtual void OnButtonSelectFolder(wxCommandEvent& event);
};
#endif // DLGSELECTFOLDER_H
