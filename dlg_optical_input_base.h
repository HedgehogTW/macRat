//////////////////////////////////////////////////////////////////////
// This file was auto-generated by codelite's wxCrafter Plugin
// wxCrafter project file: dlg_optical_input_base.wxcp
// Do not modify this file by hand!
//////////////////////////////////////////////////////////////////////

#ifndef DLG_OPTICAL_INPUT_BASE_BASE_CLASSES_H
#define DLG_OPTICAL_INPUT_BASE_BASE_CLASSES_H

#include <wx/settings.h>
#include <wx/xrc/xmlres.h>
#include <wx/xrc/xh_bmp.h>
#include <wx/dialog.h>
#include <wx/iconbndl.h>
#include <wx/artprov.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/gbsizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/statline.h>
#include <wx/checkbox.h>
#include <wx/radiobut.h>
#include <wx/button.h>

class DlgOpticalInputBase : public wxDialog
{
protected:
    wxPanel* m_panel38;
    wxStaticText* m_staticText718;
    wxTextCtrl* m_textCtrlFrameSteps;
    wxStaticText* m_staticText1320;
    wxStaticText* m_staticText1923;
    wxTextCtrl* m_textCtrlThreshold;
    wxStaticLine* m_staticLine48;
    wxPanel* m_panel80;
    wxCheckBox* m_checkBoxLED;
    wxCheckBox* m_checkBoxPinna;
    wxCheckBox* m_checkBoxVerLine;
    wxTextCtrl* m_textCtrlVerLine;
    wxStaticLine* m_staticLine84;
    wxCheckBox* m_checkBoxEyeMove;
    wxCheckBox* m_checkBoxEarGray;
    wxStaticLine* m_staticLine86;
    wxCheckBox* m_checkBoxEarOptical;
    wxCheckBox* m_checkBoxEarOpticalPDF;
    wxRadioButton* m_radioButtonInstan;
    wxRadioButton* m_radioButtonAccumu;
    wxStaticLine* m_staticLine78;
    wxPanel* m_panel96;
    wxStaticText* m_staticText100;
    wxTextCtrl* m_textCtrlYmin;
    wxStaticText* m_staticText104;
    wxTextCtrl* m_textCtrlYmax;
    wxStaticLine* m_staticLine108;
    wxStdDialogButtonSizer* m_stdBtnSizer52;
    wxButton* m_button56;
    wxButton* m_button58;

protected:

public:
    DlgOpticalInputBase(wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = _("Optical Input Dialog"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize(-1,-1), long style = wxDEFAULT_DIALOG_STYLE);
    virtual ~DlgOpticalInputBase();
};

#endif
