//////////////////////////////////////////////////////////////////////
// This file was auto-generated by codelite's wxCrafter Plugin
// wxCrafter project file: dlg_select_folder.wxcp
// Do not modify this file by hand!
//////////////////////////////////////////////////////////////////////

#include "dlg_select_folder.h"


// Declare the bitmap loading function
extern void wxCEF84InitBitmapResources();

static bool bBitmapLoaded = false;


DlgSelectFolderBase::DlgSelectFolderBase(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : wxDialog(parent, id, title, pos, size, style)
{
    if ( !bBitmapLoaded ) {
        // We need to initialise the default bitmap handler
        wxXmlResource::Get()->AddHandler(new wxBitmapXmlHandler);
        wxCEF84InitBitmapResources();
        bBitmapLoaded = true;
    }
    
    wxBoxSizer* boxSizer2 = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(boxSizer2);
    
    m_panel4 = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(-1,-1), wxTAB_TRAVERSAL);
    
    boxSizer2->Add(m_panel4, 0, wxALL, 5);
    
    wxBoxSizer* boxSizer15 = new wxBoxSizer(wxVERTICAL);
    m_panel4->SetSizer(boxSizer15);
    
    wxFlexGridSizer* flexGridSizer17 = new wxFlexGridSizer(0, 2, 0, 0);
    flexGridSizer17->SetFlexibleDirection( wxBOTH );
    flexGridSizer17->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
    
    boxSizer15->Add(flexGridSizer17, 1, wxALL|wxEXPAND, 5);
    
    m_stdBtnSizer6 = new wxStdDialogButtonSizer();
    
    boxSizer2->Add(m_stdBtnSizer6, 0, wxALL|wxALIGN_RIGHT, 5);
    
    m_button10 = new wxButton(this, wxID_OK, wxT(""), wxDefaultPosition, wxSize(-1, -1), 0);
    m_stdBtnSizer6->AddButton(m_button10);
    
    m_button12 = new wxButton(this, wxID_CANCEL, wxT(""), wxDefaultPosition, wxSize(-1, -1), 0);
    m_stdBtnSizer6->AddButton(m_button12);
    m_stdBtnSizer6->Realize();
    
    SetName(wxT("DlgSelectFolderBase"));
    SetSizeHints(500,300);
    if (GetSizer()) {
         GetSizer()->Fit(this);
    }
    if(GetParent()) {
        CentreOnParent(wxBOTH);
    } else {
        CentreOnScreen(wxBOTH);
    }
#if wxVERSION_NUMBER >= 2900
    if(!wxPersistenceManager::Get().Find(this)) {
        wxPersistenceManager::Get().RegisterAndRestore(this);
    } else {
        wxPersistenceManager::Get().Restore(this);
    }
#endif
}

DlgSelectFolderBase::~DlgSelectFolderBase()
{
}
