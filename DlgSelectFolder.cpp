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

void DlgSelectFolder::getParam(double& s, bool& bCheckOnlyFirst)
{
	wxString  str = m_textCtrlSmooth->GetValue();
	double  value;
	str.ToDouble(&value);
	s = value;
	
	bCheckOnlyFirst = m_checkBoxFirst->GetValue();
	
}

void DlgSelectFolder::setParam(double s, bool bCheckOnlyFirst)
{
	wxString  str1;
	str1 << s;
//	*m_textCtrlVerLine << str1;
	m_textCtrlSmooth->SetValue(str1);
	
	m_checkBoxFirst->SetValue(bCheckOnlyFirst);
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
	m_strDir = m_textCtrlFolder->GetValue();
	
	if ( Validate() && TransferDataFromWindow() )
	{
		if ( IsModal() )
			EndModal(wxID_OK); // If modal
		else
		{
			SetReturnCode(wxID_OK);
			this->Show(false); // If modeless
		}
	}	
}
