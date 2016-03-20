#include "DlgProcessNext.h"

DlgProcessNext::DlgProcessNext(wxWindow* parent)
    : DlgProcessNextBase(parent)
{
}

DlgProcessNext::~DlgProcessNext()
{
}
void DlgProcessNext::setValues(double ampL, double ampU, double intvalL, double intvalU, 
				double ampSnd, double intvalSnd)
{
	wxString  str1, str2, str3, str4, str5, str6;
	str1 << ampL;
	*m_textCtrlAmpLow << str1;

	str2 << ampU;
	*m_textCtrlAmpUp << str2;

	str3 << intvalL;
	*m_textCtrlIntervalLow << str3;

	str4 << intvalU;
	*m_textCtrlIntervalUp << str4;	
	
	str5 << ampSnd;
	*m_textCtrlAmpSnd << str5;

	str6 << intvalSnd;
	*m_textCtrlIntervalSnd << str6;		
}
void DlgProcessNext::getValues(double& ampL, double& ampU, double& intvalL, double& intvalU, 
				double& ampSnd, double& intvalSnd)
{
	wxString  str = m_textCtrlAmpLow->GetValue();
	str.ToDouble(&ampL);
	
	str = m_textCtrlAmpUp->GetValue();
	str.ToDouble(&ampU);

	str = m_textCtrlIntervalLow->GetValue();
	str.ToDouble(&intvalL);
	
	str = m_textCtrlIntervalUp->GetValue();
	str.ToDouble(&intvalU);

	str = m_textCtrlAmpSnd->GetValue();
	str.ToDouble(&ampSnd);
	
	str = m_textCtrlIntervalSnd->GetValue();
	str.ToDouble(&intvalSnd);	
}
void DlgProcessNext::OnReplot(wxCommandEvent& event)
{
	EndModal(999);
}
