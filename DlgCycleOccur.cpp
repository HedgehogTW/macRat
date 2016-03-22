#include "DlgCycleOccur.h"
#include <wx/filedlg.h> 
DlgCycleOccur::DlgCycleOccur(wxWindow* parent, wxString& strInitFilename)
    : DlgCycleOccurBase(parent)
{
	m_strFilename = strInitFilename;
	m_textCtrlFilename->SetValue(m_strFilename);
}

DlgCycleOccur::~DlgCycleOccur()
{
}

void DlgCycleOccur::OnButtonBrowse(wxCommandEvent& event)
{
	m_strFilename = wxFileSelector("Choose a file", m_strFilename, m_strFilename, "csv", "csv files (*.csv)|*.csv" );
	if ( !m_strFilename.empty() ){
		m_textCtrlFilename->SetValue(m_strFilename);
	}	
}
