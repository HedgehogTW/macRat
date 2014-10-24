//////////////////////////////////////////////////////////////////////
// This file was auto-generated by codelite's wxCrafter Plugin
// wxCrafter project file: optical.wxcp
// Do not modify this file by hand!
//////////////////////////////////////////////////////////////////////

#include "optical.h"


// Declare the bitmap loading function
extern void wxC718EInitBitmapResources();

static bool bBitmapLoaded = false;


DlgOpticalBase::DlgOpticalBase(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : wxDialog(parent, id, title, pos, size, style)
{
    if ( !bBitmapLoaded ) {
        // We need to initialise the default bitmap handler
        wxXmlResource::Get()->AddHandler(new wxBitmapXmlHandler);
        wxC718EInitBitmapResources();
        bBitmapLoaded = true;
    }
    
    wxFlexGridSizer* flexGridSizer2 = new wxFlexGridSizer(0, 2, 0, 5);
    flexGridSizer2->SetFlexibleDirection( wxBOTH );
    flexGridSizer2->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
    this->SetSizer(flexGridSizer2);
    
    m_staticText4 = new wxStaticText(this, wxID_ANY, _("number of pyramid layers"), wxDefaultPosition, wxSize(-1,-1), wxALIGN_LEFT);
    
    flexGridSizer2->Add(m_staticText4, 0, wxALL|wxALIGN_RIGHT, 5);
    
    m_textCtrl_PYR_LAYERS = new wxTextCtrl(this, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_textCtrl_PYR_LAYERS, 0, wxALL, 5);
    
    m_staticText8 = new wxStaticText(this, wxID_ANY, _("averaging window size"), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_staticText8, 0, wxALL|wxALIGN_RIGHT, 5);
    
    m_textCtrl_WIN_SIZE = new wxTextCtrl(this, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_textCtrl_WIN_SIZE, 0, wxALL, 5);
    
    m_staticText12 = new wxStaticText(this, wxID_ANY, _("iterations"), wxDefaultPosition, wxSize(-1,-1), wxALIGN_RIGHT);
    
    flexGridSizer2->Add(m_staticText12, 0, wxALL|wxALIGN_RIGHT, 5);
    
    m_textCtrl_ITER = new wxTextCtrl(this, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_textCtrl_ITER, 0, wxALL, 5);
    
    m_staticText16 = new wxStaticText(this, wxID_ANY, _("Poly_n, pixel neighborhood"), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_staticText16, 0, wxALL, 5);
    
    m_textCtrl_NEIGHBOR = new wxTextCtrl(this, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_textCtrl_NEIGHBOR, 0, wxALL, 5);
    
    m_staticText20 = new wxStaticText(this, wxID_ANY, _("Poly_sigma"), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_staticText20, 0, wxALL|wxALIGN_RIGHT, 5);
    
    m_textCtrl_SIGMA = new wxTextCtrl(this, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_textCtrl_SIGMA, 0, wxALL, 5);
    
    m_staticText24 = new wxStaticText(this, wxID_ANY, _("frame step"), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_staticText24, 0, wxALL|wxALIGN_RIGHT, 5);
    
    m_textCtrl_FRAME_STEPS = new wxTextCtrl(this, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_textCtrl_FRAME_STEPS, 0, wxALL, 5);
    
    m_buttonOK = new wxButton(this, wxID_OK, _("OK"), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_buttonOK, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    
    m_buttonCANCEL = new wxButton(this, wxID_CANCEL, _("Cancel"), wxDefaultPosition, wxSize(-1,-1), 0);
    
    flexGridSizer2->Add(m_buttonCANCEL, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    
    SetSizeHints(500,300);
    if ( GetSizer() ) {
         GetSizer()->Fit(this);
    }
    Centre(wxBOTH);
}

DlgOpticalBase::~DlgOpticalBase()
{
}
