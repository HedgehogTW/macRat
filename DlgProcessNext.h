#ifndef DLGPROCESSNEXT_H
#define DLGPROCESSNEXT_H
#include "dlg_process_next.h"

class DlgProcessNext : public DlgProcessNextBase
{
public:
    DlgProcessNext(wxWindow* parent);
    virtual ~DlgProcessNext();
	
	void setValues(double ampL, double ampU, double intvalL, double intvalU, double ampSnd, double intvalSnd);
	void getValues(double& ampL, double& ampU, double& intvalL, double& intvalU, double& ampSnd, double& intvalSnd);
protected:
    virtual void OnReplot(wxCommandEvent& event);
    
};
#endif // DLGPROCESSNEXT_H
