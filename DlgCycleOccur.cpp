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

int DlgCycleOccur::readData()
{
	MainFrame::myMsgOutput("open " + m_strFilename + "\n");
	
	wxFileName fileName = m_strFilename;
	m_sep = fileName.GetPathSeparator();
	m_strParentPath =  m_strFilename.BeforeLast(m_sep);

	FILE* fp =  fopen(m_strFilename, "r");
	if(fp ==NULL)  {
		MainFrame::myMsgOutput("OnGroupAmpIntvl::open file error\n");
		return -1;
	}
	m_maxCycle = -999;
	
	m_Data.clear();
	char title[100];
	fgets(title, 100, fp);

	while(!feof(fp)) {
		int a, b, c, d;
		int n = fscanf(fp, "%d,%d,%d,%d", &a, &b, &c, &d);
		if(n!= 4) break;
		
		CycleItem  item;
		item.rat = a;
		item.db = b;
		item.ampItval = c;
		item.cycle = d;
		m_Data.push_back(item);
		
		if(d > m_maxCycle)  m_maxCycle = d;		
	}
	fclose(fp);	

	int sz = m_Data.size();
	MainFrame::myMsgOutput("Read %d items, m_maxCycle %d\n", sz, m_maxCycle);
	
	createHistogram(m_strFilename);
	
	return sz;
}

void DlgCycleOccur::createHistogram(wxString filename)
{
	if(m_maxCycle <= 0)  {
		MainFrame::myMsgOutput("cannot createHistogram, m_maxCycle %d\n", m_maxCycle);
		return;
	}
	m_histo.resize(m_maxCycle +1);
	for(int i=0; i<=m_maxCycle; i++)  m_histo[i] = 0;

	FILE* fp =  fopen(filename, "r");
	if(fp ==NULL)  {
		MainFrame::myMsgOutput("createHistogram::open file error\n");
		return ;
	}
	char title[100];
	fgets(title, 100, fp);

	while(!feof(fp)) {
		int a, b, c, d;
		int n = fscanf(fp, "%d,%d,%d,%d", &a, &b, &c, &d);
		if(n!= 4) break;	
		m_histo[d]++;
	}
	fclose(fp);
	
	wxString strBefore =  filename.BeforeLast('.');
	wxString strHistoName = strBefore + "_histo.csv";
	fp =  fopen(strHistoName, "w");
	for(int i=1; i<=m_maxCycle; i++) {
		fprintf(fp , "%d, %d\n", i, m_histo[i] );
	}
	fclose(fp);
	MainFrame::myMsgOutput("createHistogram::" + strHistoName + "\n");
	
}
void DlgCycleOccur::OnGroupAmpIntvl(wxCommandEvent& event)
{
	int sz = readData();
	if(sz <0) return;
	
	wxString fileAmp = m_strParentPath + m_sep + "_amp.csv";
	wxString fileIntvl = m_strParentPath + m_sep + "_intvl.csv";	
	
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
	

	fprintf(fpAmp, "rat,db,ampItvl,cycle\n");
	fprintf(fpIntvl, "rat,db,ampItvl,cycle\n");
	
	for(int i=0; i<sz; i++){
		if(m_Data[i].ampItval==1 || m_Data[i].ampItval==2) {
			fprintf(fpAmp, "%d,%d,%d,%d\n", 
				m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);
		}else  {
			fprintf(fpIntvl, "%d,%d,%d,%d\n", 
				m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);
		}
	}
	fclose(fpAmp);
	fclose(fpIntvl);
	
	createHistogram(fileAmp);
	createHistogram(fileIntvl);
	MainFrame::myMsgOutput("output: "+ fileAmp + "\n");
	MainFrame::myMsgOutput("output: "+ fileIntvl + "\n");
}
void DlgCycleOccur::OnGroupIncDec(wxCommandEvent& event)
{
	int sz = readData();
	if(sz <0) return;
	
	wxString fileInc = m_strParentPath + m_sep + "_inc.csv";
	wxString fileDec = m_strParentPath + m_sep + "_dec.csv";	
	
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
	

	fprintf(fpInc, "rat,db,ampItvl,cycle\n");
	fprintf(fpDec, "rat,db,ampItvl,cycle\n");
	
	for(int i=0; i<sz; i++){
		if(m_Data[i].ampItval==1 || m_Data[i].ampItval==3) {
			fprintf(fpInc, "%d,%d,%d,%d\n", 
				m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);
		}else  {
			fprintf(fpDec, "%d,%d,%d,%d\n", 
				m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);
		}
	}
	fclose(fpInc);
	fclose(fpDec);
	
	createHistogram(fileInc);
	createHistogram(fileDec);
	
	MainFrame::myMsgOutput("output: "+ fileInc + "\n");
	MainFrame::myMsgOutput("output: "+ fileDec + "\n");	
}
void DlgCycleOccur::OnGroupSound(wxCommandEvent& event)
{
	int sz = readData();
	if(sz <0) return;
	
	vector<wxString> filename(8);
	filename[0] = m_strParentPath + m_sep + "_0dB.csv";
	filename[1] = m_strParentPath + m_sep + "_5dB.csv";	
	filename[2] = m_strParentPath + m_sep + "_10dB.csv";	
	filename[3] = m_strParentPath + m_sep + "_15dB.csv";	
	filename[4] = m_strParentPath + m_sep + "_20dB.csv";	
	filename[5] = m_strParentPath + m_sep + "_25dB.csv";	
	filename[6] = m_strParentPath + m_sep + "_30dB.csv";	
	filename[7] = m_strParentPath + m_sep + "_35dB.csv";	
	
	vector<FILE*> fpOut(8);
	for(int i=0; i<filename.size(); i++) {
		fpOut[i] =  fopen(filename[i], "w");
		if(fpOut[i] ==NULL)  {
			MainFrame::myMsgOutput("OnGroupSound::open fpOut error\n");
			return;
		}	
	}
	

	for(int i=0; i<filename.size(); i++)  
		fprintf(fpOut[i], "rat,db,ampItvl,cycle\n");
	
	for(int i=0; i<sz; i++){
		switch(m_Data[i].db) {
			case 0: 
				fprintf(fpOut[0], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;
			case 5: 
				fprintf(fpOut[1], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;
			case 10: 
				fprintf(fpOut[2], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;				
			case 15: 
				fprintf(fpOut[3], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;
			case 20: 
				fprintf(fpOut[4], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;
			case 25: 
				fprintf(fpOut[5], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;
			case 30: 
				fprintf(fpOut[6], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;
			case 35: 
				fprintf(fpOut[7], "%d,%d,%d,%d\n", 
					m_Data[i].rat, m_Data[i].db, m_Data[i].ampItval, m_Data[i].cycle);  
				break;
		
		}
	}

	for(int i=0; i<filename.size(); i++)  {
		fclose(fpOut[i]);
		createHistogram(filename[i]);
	}
		
	MainFrame::myMsgOutput("OnGroupSound:: output ok\n");
}
