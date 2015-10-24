#include "DlgSelectFolder.h"
#include <wx/dirdlg.h> 

DlgSelectFolder::DlgSelectFolder(wxWindow* parent, wxString& strIniDir)
    : DlgSelectFolderBase(parent)
{
	m_strDir = strIniDir;
}

DlgSelectFolder::~DlgSelectFolder()
{
}

void DlgSelectFolder::OnButtonSelectFolder(wxCommandEvent& event)
{
	//wxString dir
	m_strDir = wxDirSelector("Choose a folder", m_strDir);
	if ( !m_strDir.empty() ){
		m_textCtrlFolder->SetValue(m_strDir);
	}


}
