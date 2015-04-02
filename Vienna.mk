##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Vienna
ConfigurationName      :=Debug
WorkspacePath          := "D:\Project\macRat"
ProjectPath            := "D:\Project\macRat"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=cclee
Date                   :=04/02/15
CodeLitePath           :="C:\CodeLite"
LinkerName             :=C:/MinGW/x86_64-4.9.2/mingw64/bin/g++.exe
SharedObjectLinkerName :=C:/MinGW/x86_64-4.9.2/mingw64/bin/g++.exe -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="Vienna.txt"
PCHCompileFlags        :=
MakeDirCommand         :=makedir
RcCmpOptions           := $(shell wx-config --rcflags)
RcCompilerName         :=C:/MinGW/x86_64-4.9.2/mingw64/bin/windres.exe
LinkOptions            :=  -fopenmp $(shell wx-config --libs --debug) -mwindows
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). $(IncludeSwitch)C:/GSL/x64/include $(IncludeSwitch)C:/opencv/build/include $(IncludeSwitch)C:/ITK/x64/MinGW/include/ITK-4.7 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)gsl $(LibrarySwitch)gslcblas $(LibrarySwitch)opencv_core2411.dll $(LibrarySwitch)opencv_highgui2411.dll $(LibrarySwitch)opencv_imgproc2411.dll $(LibrarySwitch)opencv_video2411.dll $(LibrarySwitch)itkdouble-conversion-4.7.dll $(LibrarySwitch)itksys-4.7.dll $(LibrarySwitch)itkvnl_algo-4.7.dll $(LibrarySwitch)itkvnl-4.7.dll $(LibrarySwitch)itkv3p_netlib-4.7.dll $(LibrarySwitch)ITKCommon-4.7.dll $(LibrarySwitch)itkNetlibSlatec-4.7.dll $(LibrarySwitch)ITKStatistics-4.7.dll $(LibrarySwitch)ITKIOImageBase-4.7.dll $(LibrarySwitch)ITKIOBMP-4.7.dll $(LibrarySwitch)ITKIOBioRad-4.7.dll $(LibrarySwitch)ITKEXPAT-4.7.dll $(LibrarySwitch)itkopenjpeg-4.7.dll $(LibrarySwitch)itkzlib-4.7.dll $(LibrarySwitch)itkgdcmDICT-4.7.dll $(LibrarySwitch)itkgdcmMSFF-4.7.dll $(LibrarySwitch)ITKIOGDCM-4.7.dll $(LibrarySwitch)ITKIOGIPL-4.7.dll $(LibrarySwitch)itkjpeg-4.7.dll $(LibrarySwitch)ITKIOJPEG-4.7.dll $(LibrarySwitch)itktiff-4.7.dll $(LibrarySwitch)ITKIOTIFF-4.7.dll $(LibrarySwitch)ITKIOLSM-4.7.dll $(LibrarySwitch)ITKMetaIO-4.7.dll $(LibrarySwitch)ITKIOMeta-4.7.dll $(LibrarySwitch)ITKznz-4.7.dll $(LibrarySwitch)ITKniftiio-4.7.dll $(LibrarySwitch)ITKIONIFTI-4.7.dll $(LibrarySwitch)ITKNrrdIO-4.7.dll $(LibrarySwitch)ITKIONRRD-4.7.dll $(LibrarySwitch)itkpng-4.7.dll $(LibrarySwitch)ITKIOPNG-4.7.dll $(LibrarySwitch)ITKIOStimulate-4.7.dll $(LibrarySwitch)ITKIOVTK-4.7.dll $(LibrarySwitch)ITKMesh-4.7.dll $(LibrarySwitch)ITKSpatialObjects-4.7.dll $(LibrarySwitch)ITKPath-4.7.dll $(LibrarySwitch)ITKLabelMap-4.7.dll $(LibrarySwitch)ITKQuadEdgeMesh-4.7.dll $(LibrarySwitch)ITKOptimizers-4.7.dll $(LibrarySwitch)ITKPolynomials-4.7.dll $(LibrarySwitch)ITKBiasCorrection-4.7.dll $(LibrarySwitch)ITKBioCell-4.7.dll $(LibrarySwitch)ITKDICOMParser-4.7.dll $(LibrarySwitch)ITKIOXML-4.7.dll $(LibrarySwitch)ITKIOSpatialObjects-4.7.dll $(LibrarySwitch)ITKFEM-4.7.dll $(LibrarySwitch)ITKgiftiio-4.7.dll $(LibrarySwitch)ITKIOMesh-4.7.dll $(LibrarySwitch)itkhdf5_cpp-4.7.dll $(LibrarySwitch)itkhdf5-4.7.dll $(LibrarySwitch)ITKIOCSV-4.7.dll $(LibrarySwitch)ITKIOIPL-4.7.dll $(LibrarySwitch)ITKIOGE-4.7.dll $(LibrarySwitch)ITKIOSiemens-4.7.dll $(LibrarySwitch)ITKIOHDF5-4.7.dll $(LibrarySwitch)ITKIOTransformBase-4.7.dll $(LibrarySwitch)ITKIOTransformHDF5-4.7.dll $(LibrarySwitch)ITKIOTransformInsightLegacy-4.7.dll $(LibrarySwitch)ITKIOTransformMatlab-4.7.dll $(LibrarySwitch)ITKKLMRegionGrowing-4.7.dll $(LibrarySwitch)ITKVTK-4.7.dll $(LibrarySwitch)ITKWatersheds-4.7.dll $(LibrarySwitch)ITKVideoCore-4.7.dll $(LibrarySwitch)ITKVideoIO-4.7.dll $(LibrarySwitch)itkgdcmIOD-4.7.dll $(LibrarySwitch)itkgdcmCommon-4.7.dll $(LibrarySwitch)itkgdcmjpeg8-4.7.dll $(LibrarySwitch)itkgdcmjpeg12-4.7.dll $(LibrarySwitch)itkgdcmjpeg16-4.7.dll $(LibrarySwitch)ITKVNLInstantiation-4.7.dll $(LibrarySwitch)itkv3p_lsqr-4.7.dll $(LibrarySwitch)itkvcl-4.7.dll 
ArLibs                 :=  "libgsl.a" "libgslcblas.a" "libopencv_core2411.dll.a" "libopencv_highgui2411.dll.a" "libopencv_imgproc2411.dll.a" "libopencv_video2411.dll.a" "libitkdouble-conversion-4.7.dll.a" "libitksys-4.7.dll.a" "libitkvnl_algo-4.7.dll.a" "libitkvnl-4.7.dll.a" "libitkv3p_netlib-4.7.dll.a" "libITKCommon-4.7.dll.a" "libitkNetlibSlatec-4.7.dll.a" "libITKStatistics-4.7.dll.a" "libITKIOImageBase-4.7.dll.a" "libITKIOBMP-4.7.dll.a" "libITKIOBioRad-4.7.dll.a" "libITKEXPAT-4.7.dll.a" "libitkopenjpeg-4.7.dll.a" "libitkzlib-4.7.dll.a" "libitkgdcmDICT-4.7.dll.a" "libitkgdcmMSFF-4.7.dll.a" "libITKIOGDCM-4.7.dll.a" "libITKIOGIPL-4.7.dll.a" "libitkjpeg-4.7.dll.a" "libITKIOJPEG-4.7.dll.a" "libitktiff-4.7.dll.a" "libITKIOTIFF-4.7.dll.a" "libITKIOLSM-4.7.dll.a" "libITKMetaIO-4.7.dll.a" "libITKIOMeta-4.7.dll.a" "libITKznz-4.7.dll.a" "libITKniftiio-4.7.dll.a" "libITKIONIFTI-4.7.dll.a" "libITKNrrdIO-4.7.dll.a" "libITKIONRRD-4.7.dll.a" "libitkpng-4.7.dll.a" "libITKIOPNG-4.7.dll.a" "libITKIOStimulate-4.7.dll.a" "libITKIOVTK-4.7.dll.a" "libITKMesh-4.7.dll.a" "libITKSpatialObjects-4.7.dll.a" "libITKPath-4.7.dll.a" "libITKLabelMap-4.7.dll.a" "libITKQuadEdgeMesh-4.7.dll.a" "libITKOptimizers-4.7.dll.a" "libITKPolynomials-4.7.dll.a" "libITKBiasCorrection-4.7.dll.a" "libITKBioCell-4.7.dll.a" "libITKDICOMParser-4.7.dll.a" "libITKIOXML-4.7.dll.a" "libITKIOSpatialObjects-4.7.dll.a" "libITKFEM-4.7.dll.a" "libITKgiftiio-4.7.dll.a" "libITKIOMesh-4.7.dll.a" "libitkhdf5_cpp-4.7.dll.a" "libitkhdf5-4.7.dll.a" "libITKIOCSV-4.7.dll.a" "libITKIOIPL-4.7.dll.a" "libITKIOGE-4.7.dll.a" "libITKIOSiemens-4.7.dll.a" "libITKIOHDF5-4.7.dll.a" "libITKIOTransformBase-4.7.dll.a" "libITKIOTransformHDF5-4.7.dll.a" "libITKIOTransformInsightLegacy-4.7.dll.a" "libITKIOTransformMatlab-4.7.dll.a" "libITKKLMRegionGrowing-4.7.dll.a" "libITKVTK-4.7.dll.a" "libITKWatersheds-4.7.dll.a" "libITKVideoCore-4.7.dll.a" "libITKVideoIO-4.7.dll.a" "libitkgdcmIOD-4.7.dll.a" "libitkgdcmCommon-4.7.dll.a" "libitkgdcmjpeg8-4.7.dll.a" "libitkgdcmjpeg12-4.7.dll.a" "libitkgdcmjpeg16-4.7.dll.a" "libITKVNLInstantiation-4.7.dll.a" "libitkv3p_lsqr-4.7.dll.a" "libitkvcl-4.7.dll.a" 
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)C:/GSL/x64/lib $(LibraryPathSwitch)C:/opencv/build/x64/MinGW/lib $(LibraryPathSwitch)C:/ITK/x64/MinGW/lib 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := C:/MinGW/x86_64-4.9.2/mingw64/bin/ar.exe rcu
CXX      := C:/MinGW/x86_64-4.9.2/mingw64/bin/g++.exe
CC       := C:/MinGW/x86_64-4.9.2/mingw64/bin/gcc.exe
CXXFLAGS :=  -g -O0 -fopenmp -Wall -fpermissive $(shell wx-config --cflags --debug) -Wno-sign-compare -fopenmp $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall $(Preprocessors)
ASFLAGS  := 
AS       := C:/MinGW/x86_64-4.9.2/mingw64/bin/as.exe


##
## User defined environment variables
##
CodeLiteDir:=C:\CodeLite
Objects0=$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) $(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) $(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) $(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) $(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) $(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/win_resources.rc$(ObjectSuffix) $(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

$(IntermediateDirectory)/.d:
	@$(MakeDirCommand) "./Debug"

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/main.cpp$(ObjectSuffix): main.cpp $(IntermediateDirectory)/main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(DependSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/main.cpp$(DependSuffix) -MM "main.cpp"

$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix) "main.cpp"

$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix): MainFrame.cpp $(IntermediateDirectory)/MainFrame.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/MainFrame.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MainFrame.cpp$(DependSuffix): MainFrame.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/MainFrame.cpp$(DependSuffix) -MM "MainFrame.cpp"

$(IntermediateDirectory)/MainFrame.cpp$(PreprocessSuffix): MainFrame.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MainFrame.cpp$(PreprocessSuffix) "MainFrame.cpp"

$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix): wxcrafter.cpp $(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/wxcrafter.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix): wxcrafter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix) -MM "wxcrafter.cpp"

$(IntermediateDirectory)/wxcrafter.cpp$(PreprocessSuffix): wxcrafter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/wxcrafter.cpp$(PreprocessSuffix) "wxcrafter.cpp"

$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix): wxcrafter_bitmaps.cpp $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/wxcrafter_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix): wxcrafter_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix) -MM "wxcrafter_bitmaps.cpp"

$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(PreprocessSuffix): wxcrafter_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(PreprocessSuffix) "wxcrafter_bitmaps.cpp"

$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix): scrolled_image_component.cpp $(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/scrolled_image_component.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix): scrolled_image_component.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix) -MM "scrolled_image_component.cpp"

$(IntermediateDirectory)/scrolled_image_component.cpp$(PreprocessSuffix): scrolled_image_component.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/scrolled_image_component.cpp$(PreprocessSuffix) "scrolled_image_component.cpp"

$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix): KDE.cpp $(IntermediateDirectory)/KDE.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/KDE.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/KDE.cpp$(DependSuffix): KDE.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/KDE.cpp$(DependSuffix) -MM "KDE.cpp"

$(IntermediateDirectory)/KDE.cpp$(PreprocessSuffix): KDE.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/KDE.cpp$(PreprocessSuffix) "KDE.cpp"

$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix): MyUtil.cpp $(IntermediateDirectory)/MyUtil.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/MyUtil.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MyUtil.cpp$(DependSuffix): MyUtil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/MyUtil.cpp$(DependSuffix) -MM "MyUtil.cpp"

$(IntermediateDirectory)/MyUtil.cpp$(PreprocessSuffix): MyUtil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MyUtil.cpp$(PreprocessSuffix) "MyUtil.cpp"

$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix): Rat.cpp $(IntermediateDirectory)/Rat.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/Rat.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Rat.cpp$(DependSuffix): Rat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/Rat.cpp$(DependSuffix) -MM "Rat.cpp"

$(IntermediateDirectory)/Rat.cpp$(PreprocessSuffix): Rat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Rat.cpp$(PreprocessSuffix) "Rat.cpp"

$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix): BendPoint.cpp $(IntermediateDirectory)/BendPoint.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/BendPoint.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/BendPoint.cpp$(DependSuffix): BendPoint.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/BendPoint.cpp$(DependSuffix) -MM "BendPoint.cpp"

$(IntermediateDirectory)/BendPoint.cpp$(PreprocessSuffix): BendPoint.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/BendPoint.cpp$(PreprocessSuffix) "BendPoint.cpp"

$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix): gnuplot_i.cpp $(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/gnuplot_i.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix): gnuplot_i.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix) -MM "gnuplot_i.cpp"

$(IntermediateDirectory)/gnuplot_i.cpp$(PreprocessSuffix): gnuplot_i.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/gnuplot_i.cpp$(PreprocessSuffix) "gnuplot_i.cpp"

$(IntermediateDirectory)/win_resources.rc$(ObjectSuffix): win_resources.rc
	$(RcCompilerName) -i "D:/Project/macRat/win_resources.rc" $(RcCmpOptions)   $(ObjectSwitch)$(IntermediateDirectory)/win_resources.rc$(ObjectSuffix) $(RcIncludePath)
$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix): DlgOpticalInput.cpp $(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/DlgOpticalInput.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix): DlgOpticalInput.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix) -MM "DlgOpticalInput.cpp"

$(IntermediateDirectory)/DlgOpticalInput.cpp$(PreprocessSuffix): DlgOpticalInput.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/DlgOpticalInput.cpp$(PreprocessSuffix) "DlgOpticalInput.cpp"

$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix): dlg_optical_macrat_bitmaps.cpp $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/dlg_optical_macrat_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix): dlg_optical_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix) -MM "dlg_optical_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(PreprocessSuffix): dlg_optical_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(PreprocessSuffix) "dlg_optical_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix): dlg_optical_input_base_macrat_bitmaps.cpp $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/dlg_optical_input_base_macrat_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix): dlg_optical_input_base_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix) -MM "dlg_optical_input_base_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(PreprocessSuffix): dlg_optical_input_base_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(PreprocessSuffix) "dlg_optical_input_base_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix): dlg_optical_input_base.cpp $(IntermediateDirectory)/dlg_optical_input_base.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/dlg_optical_input_base.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_input_base.cpp$(DependSuffix): dlg_optical_input_base.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_input_base.cpp$(DependSuffix) -MM "dlg_optical_input_base.cpp"

$(IntermediateDirectory)/dlg_optical_input_base.cpp$(PreprocessSuffix): dlg_optical_input_base.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_input_base.cpp$(PreprocessSuffix) "dlg_optical_input_base.cpp"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


