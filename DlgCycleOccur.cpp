#include "DlgCycleOccur.h"
#include <wx/filedlg.h> 
#include "MainFrame.h"

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
void DlgCycleOccur::OnGroupAmpIntvl(wxCommandEvent& event)
{
	MainFrame::myMsgOutput("open " + m_strFilename + "\n");
	
	wxFileName fileName = m_strFilename;
	wxUniChar sep = fileName.GetPathSeparator();
	wxString strParentPath =  m_strFilename.BeforeLast(sep);
	
	wxString fileAmp = strParentPath + sep + "_amp.csv";
	wxString fileIntvl = strParentPath + sep + "_intvl.csv";	
	
	FILE* fp =  fopen(m_strFilename, "r");
	if(fp ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open file error\n");
		return;
	}
	
	FILE* fpAmp =  fopen(fileAmp, "w");
	if(fpAmp ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open fileAmp error\n");
		return;
	}	
	FILE* fpIntvl =  fopen(fileIntvl, "w");
	if(fpIntvl ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open fileIntvl error\n");
		return;
	}	
	

	
	char title[100];
	fgets(title, 100, fp);
	fprintf(fpAmp, "%s", title);
	fprintf(fpIntvl, "%s", title);
	while(!feof(fp)) {
		int a, b, c, d;
		fscanf(fp, "%d,%d,%d,%d", &a, &b, &c, &d);
		if(c==1 || c==2) {
			fprintf(fpAmp, "%d,%d,%d,%d\n", a, b, c, d);
		}else if(c==3 || c==4) {
			fprintf(fpIntvl, "%d,%d,%d,%d\n", a, b, c, d);
		}
//		MainFrame::myMsgOutput("%d,%d,%d,%d\n", a, b, c, d);
	}
	fclose(fp);
	fclose(fpAmp);
	fclose(fpIntvl);
	
	MainFrame::myMsgOutput("output: "+ fileAmp + "\n");
	MainFrame::myMsgOutput("output: "+ fileIntvl + "\n");
}
void DlgCycleOccur::OnGroupIncDec(wxCommandEvent& event)
{
	MainFrame::myMsgOutput("open " + m_strFilename + "\n");
	
	wxFileName fileName = m_strFilename;
	wxUniChar sep = fileName.GetPathSeparator();
	wxString strParentPath =  m_strFilename.BeforeLast(sep);
	
	wxString fileInc = strParentPath + sep + "_inc.csv";
	wxString fileDec = strParentPath + sep + "_dec.csv";	
	
	FILE* fp =  fopen(m_strFilename, "r");
	if(fp ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open file error\n");
		return;
	}
	
	FILE* fpInc =  fopen(fileInc, "w");
	if(fpInc ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open fileInc error\n");
		return;
	}	
	FILE* fpDec =  fopen(fileDec, "w");
	if(fpDec ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open fileDec error\n");
		return;
	}	
	

	
	char title[100];
	fgets(title, 100, fp);
	fprintf(fpInc, "%s", title);
	fprintf(fpDec, "%s", title);
	while(!feof(fp)) {
		int a, b, c, d;
		fscanf(fp, "%d,%d,%d,%d", &a, &b, &c, &d);
		if(c==1 || c==3) {
			fprintf(fpInc, "%d,%d,%d,%d\n", a, b, c, d);
		}else if(c==2 || c==4) {
			fprintf(fpDec, "%d,%d,%d,%d\n", a, b, c, d);
		}
//		MainFrame::myMsgOutput("%d,%d,%d,%d\n", a, b, c, d);
	}
	fclose(fp);
	fclose(fpInc);
	fclose(fpDec);
	
	MainFrame::myMsgOutput("output: "+ fileInc + "\n");
	MainFrame::myMsgOutput("output: "+ fileDec + "\n");	
}
void DlgCycleOccur::OnGroupSound(wxCommandEvent& event)
{
	MainFrame::myMsgOutput("open " + m_strFilename + "\n");
	
	wxFileName fileName = m_strFilename;
	wxUniChar sep = fileName.GetPathSeparator();
	wxString strParentPath =  m_strFilename.BeforeLast(sep);
	
	vector<wxString> filename(8);
	filename[0] = strParentPath + sep + "_0dB.csv";
	filename[1] = strParentPath + sep + "_5dB.csv";	
	filename[2] = strParentPath + sep + "_10dB.csv";	
	filename[3] = strParentPath + sep + "_15dB.csv";	
	filename[4] = strParentPath + sep + "_20dB.csv";	
	filename[5] = strParentPath + sep + "_25dB.csv";	
	filename[6] = strParentPath + sep + "_30dB.csv";	
	filename[7] = strParentPath + sep + "_35dB.csv";	
	
	FILE* fp =  fopen(m_strFilename, "r");
	if(fp ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open file error\n");
		return;
	}
	
	vector<FILE*> fpOut(8);
	for(int i=0; i<filename.size(); i++) {
		fpOut[i] =  fopen(filename[i], "w");
		if(fpOut[i] ==NULL)  {
			MainFrame::myMsgOutput("OnGroupSound::open fpOut error\n");
			return;
		}	
	}
	
	char title[100];
	fgets(title, 100, fp);
	
	for(int i=0; i<filename.size(); i++)  fprintf(fpOut[i], "%s", title);
	
	while(!feof(fp)) {
		int a, b, c, d;
		fscanf(fp, "%d,%d,%d,%d", &a, &b, &c, &d);
		switch(b) {
			case 0: 
				fprintf(fpOut[0], "%d,%d,%d,%d\n", a, b, c, d);  break;
			case 5: 
				fprintf(fpOut[1], "%d,%d,%d,%d\n", a, b, c, d);  break;
			case 10: 
				fprintf(fpOut[2], "%d,%d,%d,%d\n", a, b, c, d);  break;					
			case 15: 
				fprintf(fpOut[3], "%d,%d,%d,%d\n", a, b, c, d);  break;
			case 20: 
				fprintf(fpOut[4], "%d,%d,%d,%d\n", a, b, c, d);  break;
			case 25: 
				fprintf(fpOut[5], "%d,%d,%d,%d\n", a, b, c, d);  break;
			case 30: 
				fprintf(fpOut[6], "%d,%d,%d,%d\n", a, b, c, d);  break;
			case 35: 
				fprintf(fpOut[7], "%d,%d,%d,%d\n", a, b, c, d);  break;
		
		}
	}
	fclose(fp);
	for(int i=0; i<filename.size(); i++) 
		fclose(fpOut[i]);
		
	MainFrame::myMsgOutput("OnGroupSound:: output ok\n");
}
