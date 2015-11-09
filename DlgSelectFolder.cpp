#include "DlgSelectFolder.h"
#include <wx/dirdlg.h> 

DlgSelectFolder::DlgSelectFolder(wxWindow* parent, wxString& strIniDir)
    : DlgSelectFolderBase(parent)
{
	m_strDir = strIniDir;
	m_textCtrlFolder->SetValue(m_strDir);
}

DlgSelectFolder::~DlgSelectFolder()
{
}

void DlgSelectFolder::getSmoothWidth(double& s)
{
	wxString  str = m_textCtrlSmooth->GetValue();
	double  value;
	str.ToDouble(&value);
	s = value;
}

void DlgSelectFolder::setSmoothWidth(double s)
{
	wxString  str1;
	str1 << s;
//	*m_textCtrlVerLine << str1;
	m_textCtrlSmooth->SetValue(str1);
}
void DlgSelectFolder::OnButtonSelectFolder(wxCommandEvent& event)
{
	//wxString dir
	m_strDir = wxDirSelector("Choose a folder", m_strDir);
	if ( !m_strDir.empty() ){
		m_textCtrlFolder->SetValue(m_strDir);
	}


}
void DlgSelectFolder::OnOK(wxCommandEvent& event)
{
}
