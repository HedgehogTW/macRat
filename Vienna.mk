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
Date                   :=11/13/14
CodeLitePath           :="C:\CodeLite"
LinkerName             :=C:/MinGW-4.8.1/bin/g++.exe 
SharedObjectLinkerName :=C:/MinGW-4.8.1/bin/g++.exe -shared -fPIC
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
RcCompilerName         :=C:/MinGW-4.8.1/bin/windres.exe 
LinkOptions            :=  $(shell wx-config --libs --debug) -mwindows
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). $(IncludeSwitch)C:/GSL/include $(IncludeSwitch)C:/opencv/build/include $(IncludeSwitch)C:/ITK/MinGW/include/ITK-4.5 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)gsl $(LibrarySwitch)gslcblas $(LibrarySwitch)opencv_core249.dll $(LibrarySwitch)opencv_highgui249.dll $(LibrarySwitch)opencv_imgproc249.dll $(LibrarySwitch)opencv_video249.dll $(LibrarySwitch)itkdouble-conversion-4.5.dll $(LibrarySwitch)itksys-4.5.dll $(LibrarySwitch)itkvnl_algo-4.5.dll $(LibrarySwitch)itkvnl-4.5.dll $(LibrarySwitch)itkv3p_netlib-4.5.dll $(LibrarySwitch)ITKCommon-4.5.dll $(LibrarySwitch)itkNetlibSlatec-4.5.dll $(LibrarySwitch)ITKStatistics-4.5.dll $(LibrarySwitch)ITKIOImageBase-4.5.dll $(LibrarySwitch)ITKIOBMP-4.5.dll $(LibrarySwitch)ITKIOBioRad-4.5.dll $(LibrarySwitch)ITKEXPAT-4.5.dll $(LibrarySwitch)itkopenjpeg-4.5.dll $(LibrarySwitch)itkzlib-4.5.dll $(LibrarySwitch)itkgdcmDICT-4.5.dll $(LibrarySwitch)itkgdcmMSFF-4.5.dll $(LibrarySwitch)ITKIOGDCM-4.5.dll $(LibrarySwitch)ITKIOGIPL-4.5.dll $(LibrarySwitch)itkjpeg-4.5.dll $(LibrarySwitch)ITKIOJPEG-4.5.dll $(LibrarySwitch)itktiff-4.5.dll $(LibrarySwitch)ITKIOTIFF-4.5.dll $(LibrarySwitch)ITKIOLSM-4.5.dll $(LibrarySwitch)ITKMetaIO-4.5.dll $(LibrarySwitch)ITKIOMeta-4.5.dll $(LibrarySwitch)ITKznz-4.5.dll $(LibrarySwitch)ITKniftiio-4.5.dll $(LibrarySwitch)ITKIONIFTI-4.5.dll $(LibrarySwitch)ITKNrrdIO-4.5.dll $(LibrarySwitch)ITKIONRRD-4.5.dll $(LibrarySwitch)itkpng-4.5.dll $(LibrarySwitch)ITKIOPNG-4.5.dll $(LibrarySwitch)ITKIOStimulate-4.5.dll $(LibrarySwitch)ITKIOVTK-4.5.dll $(LibrarySwitch)ITKMesh-4.5.dll $(LibrarySwitch)ITKSpatialObjects-4.5.dll $(LibrarySwitch)ITKPath-4.5.dll $(LibrarySwitch)ITKLabelMap-4.5.dll $(LibrarySwitch)ITKQuadEdgeMesh-4.5.dll $(LibrarySwitch)ITKOptimizers-4.5.dll $(LibrarySwitch)ITKPolynomials-4.5.dll $(LibrarySwitch)ITKBiasCorrection-4.5.dll $(LibrarySwitch)ITKBioCell-4.5.dll $(LibrarySwitch)ITKDICOMParser-4.5.dll $(LibrarySwitch)ITKIOXML-4.5.dll $(LibrarySwitch)ITKIOSpatialObjects-4.5.dll $(LibrarySwitch)ITKFEM-4.5.dll $(LibrarySwitch)ITKgiftiio-4.5.dll $(LibrarySwitch)ITKIOMesh-4.5.dll $(LibrarySwitch)itkhdf5_cpp-4.5.dll $(LibrarySwitch)itkhdf5-4.5.dll $(LibrarySwitch)ITKIOCSV-4.5.dll $(LibrarySwitch)ITKIOIPL-4.5.dll $(LibrarySwitch)ITKIOGE-4.5.dll $(LibrarySwitch)ITKIOSiemens-4.5.dll $(LibrarySwitch)ITKIOHDF5-4.5.dll $(LibrarySwitch)ITKIOTransformBase-4.5.dll $(LibrarySwitch)ITKIOTransformHDF5-4.5.dll $(LibrarySwitch)ITKIOTransformInsightLegacy-4.5.dll $(LibrarySwitch)ITKIOTransformMatlab-4.5.dll $(LibrarySwitch)ITKKLMRegionGrowing-4.5.dll $(LibrarySwitch)ITKVTK-4.5.dll $(LibrarySwitch)ITKWatersheds-4.5.dll $(LibrarySwitch)ITKVideoCore-4.5.dll $(LibrarySwitch)ITKVideoIO-4.5.dll $(LibrarySwitch)itkgdcmIOD-4.5.dll $(LibrarySwitch)itkgdcmCommon-4.5.dll $(LibrarySwitch)itkgdcmjpeg8-4.5.dll $(LibrarySwitch)itkgdcmjpeg12-4.5.dll $(LibrarySwitch)itkgdcmjpeg16-4.5.dll $(LibrarySwitch)ITKVNLInstantiation-4.5.dll $(LibrarySwitch)itkv3p_lsqr-4.5.dll $(LibrarySwitch)itkvcl-4.5.dll 
ArLibs                 :=  "libgsl.a" "libgslcblas.a" "libopencv_core249.dll.a" "libopencv_highgui249.dll.a" "libopencv_imgproc249.dll.a" "libopencv_video249.dll.a" "libitkdouble-conversion-4.5.dll.a" "libitksys-4.5.dll.a" "libitkvnl_algo-4.5.dll.a" "libitkvnl-4.5.dll.a" "libitkv3p_netlib-4.5.dll.a" "libITKCommon-4.5.dll.a" "libitkNetlibSlatec-4.5.dll.a" "libITKStatistics-4.5.dll.a" "libITKIOImageBase-4.5.dll.a" "libITKIOBMP-4.5.dll.a" "libITKIOBioRad-4.5.dll.a" "libITKEXPAT-4.5.dll.a" "libitkopenjpeg-4.5.dll.a" "libitkzlib-4.5.dll.a" "libitkgdcmDICT-4.5.dll.a" "libitkgdcmMSFF-4.5.dll.a" "libITKIOGDCM-4.5.dll.a" "libITKIOGIPL-4.5.dll.a" "libitkjpeg-4.5.dll.a" "libITKIOJPEG-4.5.dll.a" "libitktiff-4.5.dll.a" "libITKIOTIFF-4.5.dll.a" "libITKIOLSM-4.5.dll.a" "libITKMetaIO-4.5.dll.a" "libITKIOMeta-4.5.dll.a" "libITKznz-4.5.dll.a" "libITKniftiio-4.5.dll.a" "libITKIONIFTI-4.5.dll.a" "libITKNrrdIO-4.5.dll.a" "libITKIONRRD-4.5.dll.a" "libitkpng-4.5.dll.a" "libITKIOPNG-4.5.dll.a" "libITKIOStimulate-4.5.dll.a" "libITKIOVTK-4.5.dll.a" "libITKMesh-4.5.dll.a" "libITKSpatialObjects-4.5.dll.a" "libITKPath-4.5.dll.a" "libITKLabelMap-4.5.dll.a" "libITKQuadEdgeMesh-4.5.dll.a" "libITKOptimizers-4.5.dll.a" "libITKPolynomials-4.5.dll.a" "libITKBiasCorrection-4.5.dll.a" "libITKBioCell-4.5.dll.a" "libITKDICOMParser-4.5.dll.a" "libITKIOXML-4.5.dll.a" "libITKIOSpatialObjects-4.5.dll.a" "libITKFEM-4.5.dll.a" "libITKgiftiio-4.5.dll.a" "libITKIOMesh-4.5.dll.a" "libitkhdf5_cpp-4.5.dll.a" "libitkhdf5-4.5.dll.a" "libITKIOCSV-4.5.dll.a" "libITKIOIPL-4.5.dll.a" "libITKIOGE-4.5.dll.a" "libITKIOSiemens-4.5.dll.a" "libITKIOHDF5-4.5.dll.a" "libITKIOTransformBase-4.5.dll.a" "libITKIOTransformHDF5-4.5.dll.a" "libITKIOTransformInsightLegacy-4.5.dll.a" "libITKIOTransformMatlab-4.5.dll.a" "libITKKLMRegionGrowing-4.5.dll.a" "libITKVTK-4.5.dll.a" "libITKWatersheds-4.5.dll.a" "libITKVideoCore-4.5.dll.a" "libITKVideoIO-4.5.dll.a" "libitkgdcmIOD-4.5.dll.a" "libitkgdcmCommon-4.5.dll.a" "libitkgdcmjpeg8-4.5.dll.a" "libitkgdcmjpeg12-4.5.dll.a" "libitkgdcmjpeg16-4.5.dll.a" "libITKVNLInstantiation-4.5.dll.a" "libitkv3p_lsqr-4.5.dll.a" "libitkvcl-4.5.dll.a" 
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)C:/GSL/lib $(LibraryPathSwitch)C:/opencv/build/MinGW/lib $(LibraryPathSwitch)C:/ITK/MinGW/lib 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := C:/MinGW-4.8.1/bin/ar.exe rcu
CXX      := C:/MinGW-4.8.1/bin/g++.exe 
CC       := C:/MinGW-4.8.1/bin/gcc.exe 
CXXFLAGS :=  -g -O0 -Wall $(shell wx-config --cflags --debug)  -Wno-sign-compare $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall $(Preprocessors)
ASFLAGS  := 
AS       := C:/MinGW-4.8.1/bin/as.exe 


##
## User defined environment variables
##
CodeLiteDir:=C:\CodeLite
UNIT_TEST_PP_SRC_DIR:=C:\UnitTest++-1.3
Objects0=$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) $(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) $(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix) $(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) $(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) $(IntermediateDirectory)/win_resources.rc$(ObjectSuffix) \
	$(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix) $(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/optical.cpp$(ObjectSuffix) 



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

$(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix): LineDetector.cpp $(IntermediateDirectory)/LineDetector.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/LineDetector.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/LineDetector.cpp$(DependSuffix): LineDetector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/LineDetector.cpp$(DependSuffix) -MM "LineDetector.cpp"

$(IntermediateDirectory)/LineDetector.cpp$(PreprocessSuffix): LineDetector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/LineDetector.cpp$(PreprocessSuffix) "LineDetector.cpp"

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

$(IntermediateDirectory)/win_resources.rc$(ObjectSuffix): win_resources.rc
	$(RcCompilerName) -i "D:/Project/macRat/win_resources.rc" $(RcCmpOptions)   $(ObjectSwitch)$(IntermediateDirectory)/win_resources.rc$(ObjectSuffix) $(RcIncludePath)
$(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix): DlgOptical.cpp $(IntermediateDirectory)/DlgOptical.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/DlgOptical.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/DlgOptical.cpp$(DependSuffix): DlgOptical.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/DlgOptical.cpp$(DependSuffix) -MM "DlgOptical.cpp"

$(IntermediateDirectory)/DlgOptical.cpp$(PreprocessSuffix): DlgOptical.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/DlgOptical.cpp$(PreprocessSuffix) "DlgOptical.cpp"

$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix): optical_macrat_01_bitmaps.cpp $(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/optical_macrat_01_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(DependSuffix): optical_macrat_01_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(DependSuffix) -MM "optical_macrat_01_bitmaps.cpp"

$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(PreprocessSuffix): optical_macrat_01_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(PreprocessSuffix) "optical_macrat_01_bitmaps.cpp"

$(IntermediateDirectory)/optical.cpp$(ObjectSuffix): optical.cpp $(IntermediateDirectory)/optical.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "D:/Project/macRat/optical.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/optical.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/optical.cpp$(DependSuffix): optical.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/optical.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/optical.cpp$(DependSuffix) -MM "optical.cpp"

$(IntermediateDirectory)/optical.cpp$(PreprocessSuffix): optical.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/optical.cpp$(PreprocessSuffix) "optical.cpp"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) ./Debug/*$(ObjectSuffix)
	$(RM) ./Debug/*$(DependSuffix)
	$(RM) $(OutputFile)
	$(RM) $(OutputFile).exe
	$(RM) ".build-debug/Vienna"


