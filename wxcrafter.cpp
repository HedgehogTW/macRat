//////////////////////////////////////////////////////////////////////
// This file was auto-generated by codelite's wxCrafter Plugin
// wxCrafter project file: wxcrafter.wxcp
// Do not modify this file by hand!
//////////////////////////////////////////////////////////////////////

#include "wxcrafter.h"


// Declare the bitmap loading function
extern void wxC9ED9InitBitmapResources();

static bool bBitmapLoaded = false;


MainFrameBaseClass::MainFrameBaseClass(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : wxFrame(parent, id, title, pos, size, style)
{
    if ( !bBitmapLoaded ) {
        // We need to initialise the default bitmap handler
        wxXmlResource::Get()->AddHandler(new wxBitmapXmlHandler);
        wxC9ED9InitBitmapResources();
        bBitmapLoaded = true;
    }
    
    m_menuBar = new wxMenuBar(0);
    this->SetMenuBar(m_menuBar);
    
    m_menuFile = new wxMenu();
    m_menuBar->Append(m_menuFile, _("File"));
    
    m_menuItemOpen = new wxMenuItem(m_menuFile, wxID_OPEN, _("Open"), wxT(""), wxITEM_NORMAL);
    m_menuFile->Append(m_menuItemOpen);
    
    m_menuItem7 = new wxMenuItem(m_menuFile, wxID_EXIT, _("Exit\tAlt-X"), _("Quit"), wxITEM_NORMAL);
    m_menuFile->Append(m_menuItem7);
    
    m_menuEdit = new wxMenu();
    m_menuBar->Append(m_menuEdit, _("Edit"));
    
    m_menuView = new wxMenu();
    m_menuBar->Append(m_menuView, _("View"));
    
    m_menuItemViewMsgPane = new wxMenuItem(m_menuView, wxID_ANY, _("Show Message Pane"), wxT(""), wxITEM_CHECK);
    m_menuView->Append(m_menuItemViewMsgPane);
    m_menuItemViewMsgPane->Check();
    
    m_menuRat = new wxMenu();
    m_menuBar->Append(m_menuRat, _("Rat"));
    
    m_menuItemProcess = new wxMenuItem(m_menuRat, wxID_RAT_PROCESS, _("Process"), wxT(""), wxITEM_NORMAL);
    m_menuRat->Append(m_menuItemProcess);
    
    m_menuItemShowResults = new wxMenuItem(m_menuRat, wxID_SHOW_RESULT, _("Show Results"), wxT(""), wxITEM_NORMAL);
    m_menuRat->Append(m_menuItemShowResults);
    
    m_menuItemLoadResult = new wxMenuItem(m_menuRat, wxID_LOAD_RESULT, _("Load Result"), wxT(""), wxITEM_NORMAL);
    m_menuRat->Append(m_menuItemLoadResult);
    
    m_nameHelp = new wxMenu();
    m_menuBar->Append(m_nameHelp, _("Help"));
    
    m_menuItem9 = new wxMenuItem(m_nameHelp, wxID_ABOUT, _("About..."), wxT(""), wxITEM_NORMAL);
    m_nameHelp->Append(m_menuItem9);
    
    m_auimgr11 = new wxAuiManager;
    m_auimgr11->SetManagedWindow( this );
    m_auimgr11->SetFlags( wxAUI_MGR_LIVE_RESIZE|wxAUI_MGR_TRANSPARENT_HINT|wxAUI_MGR_TRANSPARENT_DRAG|wxAUI_MGR_ALLOW_ACTIVE_PANE|wxAUI_MGR_ALLOW_FLOATING);
    m_auimgr11->GetArtProvider()->SetMetric(wxAUI_DOCKART_GRADIENT_TYPE, wxAUI_GRADIENT_NONE);
    
    m_scrollWin = new ScrolledImageComponent(this, wxID_ANY, wxDefaultPosition, wxSize(-1,-1), wxHSCROLL|wxVSCROLL);
    m_scrollWin->SetScrollRate(5, 5);
    
    m_auimgr11->AddPane(m_scrollWin, wxAuiPaneInfo().Direction(wxAUI_DOCK_CENTER).Layer(0).Row(0).Position(0).BestSize(100,100).MinSize(100,100).MaxSize(100,100).CaptionVisible(false).MaximizeButton(false).CloseButton(false).MinimizeButton(false).PinButton(false));
    
    m_panel15 = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(-1,-1), wxTAB_TRAVERSAL);
    
    m_auimgr11->AddPane(m_panel15, wxAuiPaneInfo().Name(wxT("MsgPane")).Caption(_("Message Pane")).Direction(wxAUI_DOCK_BOTTOM).Layer(0).Row(0).Position(0).BestSize(100,150).MinSize(100,100).MaxSize(100,100).CaptionVisible(true).MaximizeButton(false).CloseButton(true).MinimizeButton(false).PinButton(false));
    
    wxBoxSizer* boxSizer25 = new wxBoxSizer(wxVERTICAL);
    m_panel15->SetSizer(boxSizer25);
    
    m_richTextMsg = new wxRichTextCtrl(m_panel15, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(-1,-1), wxTE_MULTILINE|wxTE_PROCESS_TAB|wxTE_PROCESS_ENTER|wxWANTS_CHARS);
    
    boxSizer25->Add(m_richTextMsg, 1, wxALL|wxEXPAND, 0);
    
    m_auibar31 = new wxAuiToolBar(this, wxID_ANY, wxDefaultPosition, wxSize(-1,-1), wxAUI_TB_PLAIN_BACKGROUND|wxAUI_TB_DEFAULT_STYLE);
    m_auibar31->SetToolBitmapSize(wxSize(32,32));
    
    m_auimgr11->AddPane(m_auibar31, wxAuiPaneInfo().Direction(wxAUI_DOCK_TOP).Layer(0).Row(0).Position(0).BestSize(42,42).MinSize(42,42).CaptionVisible(false).MaximizeButton(false).CloseButton(false).MinimizeButton(false).PinButton(false).ToolbarPane());
    m_auimgr11->Update();
    
    m_auibar31->AddTool(wxID_OPEN, _("Tool Label"), wxArtProvider::GetBitmap(wxART_FILE_OPEN, wxART_TOOLBAR, wxSize(32, 32)), wxNullBitmap, wxITEM_NORMAL, wxT(""), wxT(""), NULL);
    
    m_auibar31->AddSeparator();
    
    m_toggleButtonMarkEyes = new wxToggleButton(m_auibar31, wxID_RAT_MARK_EYE, _("Mark eyes"), wxDefaultPosition, wxSize(-1,-1), 0);
    m_toggleButtonMarkEyes->SetValue(false);
    m_auibar31->AddControl(m_toggleButtonMarkEyes);
    
    m_toggleButtonMarkEars = new wxToggleButton(m_auibar31, wxID_RAT_MARK_EAR, _("Mark Ears"), wxDefaultPosition, wxSize(-1,-1), 0);
    m_toggleButtonMarkEars->SetValue(false);
    m_auibar31->AddControl(m_toggleButtonMarkEars);
    
    m_auibar31->AddSeparator();
    
    m_auibar31->AddTool(wxID_RAT_PROCESS, _("Process"), wxXmlResource::Get()->LoadBitmap(wxT("Mouse")), wxNullBitmap, wxITEM_NORMAL, _("Process"), wxT(""), NULL);
    
    m_auibar31->AddTool(wxID_SHOW_RESULT, _("Show Result"), wxXmlResource::Get()->LoadBitmap(wxT("result")), wxNullBitmap, wxITEM_NORMAL, _("Show Result"), wxT(""), NULL);
    
    m_auibar31->AddTool(wxID_LOAD_RESULT, _("Load Result"), wxXmlResource::Get()->LoadBitmap(wxT("loadResult")), wxNullBitmap, wxITEM_NORMAL, _("Load Result"), wxT(""), NULL);
    m_auibar31->Realize();
    
    m_statusBar = new wxStatusBar(this, wxID_ANY, wxSTB_DEFAULT_STYLE);
    m_statusBar->SetFieldsCount(1);
    this->SetStatusBar(m_statusBar);
    
    SetSizeHints(700,750);
    if ( GetSizer() ) {
         GetSizer()->Fit(this);
    }
    Centre(wxBOTH);
    // Connect events
    this->Connect(m_menuItemOpen->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnFileOpen), NULL, this);
    this->Connect(m_menuItem7->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnExit), NULL, this);
    this->Connect(m_menuItemViewMsgPane->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnViewMsgPane), NULL, this);
    this->Connect(m_menuItemViewMsgPane->GetId(), wxEVT_UPDATE_UI, wxUpdateUIEventHandler(MainFrameBaseClass::OnUpdateViewMsgPane), NULL, this);
    this->Connect(m_menuItemProcess->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnRatProcess), NULL, this);
    this->Connect(m_menuItemShowResults->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnRatShowResults), NULL, this);
    this->Connect(m_menuItemLoadResult->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnRatLoadResult), NULL, this);
    this->Connect(m_menuItem9->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnAbout), NULL, this);
    m_scrollWin->Connect(wxEVT_MOTION, wxMouseEventHandler(MainFrameBaseClass::OnMouseMotion), NULL, this);
    m_scrollWin->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MainFrameBaseClass::OnLeftButtonDown), NULL, this);
    this->Connect(wxID_OPEN, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler(MainFrameBaseClass::OnFileOpen), NULL, this);
    m_toggleButtonMarkEyes->Connect(wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler(MainFrameBaseClass::OnMarkEyes), NULL, this);
    m_toggleButtonMarkEars->Connect(wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler(MainFrameBaseClass::OnMarkEars), NULL, this);
    
}

MainFrameBaseClass::~MainFrameBaseClass()
{
    this->Disconnect(m_menuItemOpen->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnFileOpen), NULL, this);
    this->Disconnect(m_menuItem7->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnExit), NULL, this);
    this->Disconnect(m_menuItemViewMsgPane->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnViewMsgPane), NULL, this);
    this->Disconnect(m_menuItemViewMsgPane->GetId(), wxEVT_UPDATE_UI, wxUpdateUIEventHandler(MainFrameBaseClass::OnUpdateViewMsgPane), NULL, this);
    this->Disconnect(m_menuItemProcess->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnRatProcess), NULL, this);
    this->Disconnect(m_menuItemShowResults->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnRatShowResults), NULL, this);
    this->Disconnect(m_menuItemLoadResult->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnRatLoadResult), NULL, this);
    this->Disconnect(m_menuItem9->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrameBaseClass::OnAbout), NULL, this);
    m_scrollWin->Disconnect(wxEVT_MOTION, wxMouseEventHandler(MainFrameBaseClass::OnMouseMotion), NULL, this);
    m_scrollWin->Disconnect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MainFrameBaseClass::OnLeftButtonDown), NULL, this);
    this->Disconnect(wxID_OPEN, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler(MainFrameBaseClass::OnFileOpen), NULL, this);
    m_toggleButtonMarkEyes->Disconnect(wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler(MainFrameBaseClass::OnMarkEyes), NULL, this);
    m_toggleButtonMarkEars->Disconnect(wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler(MainFrameBaseClass::OnMarkEars), NULL, this);
    
    m_auimgr11->UnInit();
    delete m_auimgr11;

}
