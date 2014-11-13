##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Vienna
ConfigurationName      :=Debug
WorkspacePath          := "/Users/CCLee/Project/macRat"
ProjectPath            := "/Users/CCLee/Project/macRat"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=李建誠
Date                   :=2014-11-13
CodeLitePath           :="/Users/CCLee/Library/Application Support/codelite"
LinkerName             :=/usr/bin/g++ 
SharedObjectLinkerName :=/usr/bin/g++ -dynamiclib -fPIC
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
MakeDirCommand         :=mkdir -p
LinkOptions            :=  $(shell /usr/local/bin/wx-config --libs --debug)
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). $(IncludeSwitch)/usr/local/include $(IncludeSwitch)/usr/local/include/ITK-4.5 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)gsl $(LibrarySwitch)gslcblas $(LibrarySwitch)opencv_core $(LibrarySwitch)opencv_video $(LibrarySwitch)opencv_highgui $(LibrarySwitch)opencv_imgproc $(LibrarySwitch)itkdouble-conversion-4.5 $(LibrarySwitch)itksys-4.5 $(LibrarySwitch)itkvnl_algo-4.5 $(LibrarySwitch)itkvnl-4.5 $(LibrarySwitch)itkv3p_netlib-4.5 $(LibrarySwitch)ITKCommon-4.5 $(LibrarySwitch)itkNetlibSlatec-4.5 $(LibrarySwitch)ITKStatistics-4.5 $(LibrarySwitch)ITKIOImageBase-4.5 $(LibrarySwitch)ITKIOBMP-4.5 $(LibrarySwitch)ITKIOBioRad-4.5 $(LibrarySwitch)ITKEXPAT-4.5 $(LibrarySwitch)itkopenjpeg-4.5 $(LibrarySwitch)itkzlib-4.5 $(LibrarySwitch)itkgdcmDICT-4.5 $(LibrarySwitch)itkgdcmMSFF-4.5 $(LibrarySwitch)ITKIOGDCM-4.5 $(LibrarySwitch)ITKIOGIPL-4.5 $(LibrarySwitch)itkjpeg-4.5 $(LibrarySwitch)ITKIOJPEG-4.5 $(LibrarySwitch)itktiff-4.5 $(LibrarySwitch)ITKIOTIFF-4.5 $(LibrarySwitch)ITKIOLSM-4.5 $(LibrarySwitch)ITKMetaIO-4.5 $(LibrarySwitch)ITKIOMeta-4.5 $(LibrarySwitch)ITKznz-4.5 $(LibrarySwitch)ITKniftiio-4.5 $(LibrarySwitch)ITKIONIFTI-4.5 $(LibrarySwitch)ITKNrrdIO-4.5 $(LibrarySwitch)ITKIONRRD-4.5 $(LibrarySwitch)itkpng-4.5 $(LibrarySwitch)ITKIOPNG-4.5 $(LibrarySwitch)ITKIOStimulate-4.5 $(LibrarySwitch)ITKIOVTK-4.5 $(LibrarySwitch)ITKMesh-4.5 $(LibrarySwitch)ITKSpatialObjects-4.5 $(LibrarySwitch)ITKPath-4.5 $(LibrarySwitch)ITKLabelMap-4.5 $(LibrarySwitch)ITKQuadEdgeMesh-4.5 $(LibrarySwitch)ITKOptimizers-4.5 $(LibrarySwitch)ITKPolynomials-4.5 $(LibrarySwitch)ITKBiasCorrection-4.5 $(LibrarySwitch)ITKBioCell-4.5 $(LibrarySwitch)ITKDICOMParser-4.5 $(LibrarySwitch)ITKIOXML-4.5 $(LibrarySwitch)ITKIOSpatialObjects-4.5 $(LibrarySwitch)ITKFEM-4.5 $(LibrarySwitch)ITKgiftiio-4.5 $(LibrarySwitch)ITKIOMesh-4.5 $(LibrarySwitch)itkhdf5_cpp-4.5 $(LibrarySwitch)itkhdf5-4.5 $(LibrarySwitch)ITKIOCSV-4.5 $(LibrarySwitch)ITKIOIPL-4.5 $(LibrarySwitch)ITKIOGE-4.5 $(LibrarySwitch)ITKIOSiemens-4.5 $(LibrarySwitch)ITKIOHDF5-4.5 $(LibrarySwitch)ITKIOTransformBase-4.5 $(LibrarySwitch)ITKIOTransformHDF5-4.5 $(LibrarySwitch)ITKIOTransformInsightLegacy-4.5 $(LibrarySwitch)ITKIOTransformMatlab-4.5 $(LibrarySwitch)ITKKLMRegionGrowing-4.5 $(LibrarySwitch)ITKVTK-4.5 $(LibrarySwitch)ITKWatersheds-4.5 $(LibrarySwitch)ITKVideoCore-4.5 $(LibrarySwitch)ITKVideoIO-4.5 $(LibrarySwitch)itkgdcmIOD-4.5 $(LibrarySwitch)itkgdcmCommon-4.5 $(LibrarySwitch)itkgdcmjpeg8-4.5 $(LibrarySwitch)itkgdcmjpeg12-4.5 $(LibrarySwitch)itkgdcmjpeg16-4.5 $(LibrarySwitch)ITKVNLInstantiation-4.5 $(LibrarySwitch)itkv3p_lsqr-4.5 $(LibrarySwitch)itkvcl-4.5 
ArLibs                 :=  "libgsl.dylib" "libgslcblas.dylib" "libopencv_core.dylib" "libopencv_video.dylib" "libopencv_highgui.dylib" "libopencv_imgproc.dylib" "libitkdouble-conversion-4.5.a" "libitksys-4.5.a" "libitkvnl_algo-4.5.a" "libitkvnl-4.5.a" "libitkv3p_netlib-4.5.a" "libITKCommon-4.5.a" "libitkNetlibSlatec-4.5.a" "libITKStatistics-4.5.a" "libITKIOImageBase-4.5.a" "libITKIOBMP-4.5.a" "libITKIOBioRad-4.5.a" "libITKEXPAT-4.5.a" "libitkopenjpeg-4.5.a" "libitkzlib-4.5.a" "libitkgdcmDICT-4.5.a" "libitkgdcmMSFF-4.5.a" "libITKIOGDCM-4.5.a" "libITKIOGIPL-4.5.a" "libitkjpeg-4.5.a" "libITKIOJPEG-4.5.a" "libitktiff-4.5.a" "libITKIOTIFF-4.5.a" "libITKIOLSM-4.5.a" "libITKMetaIO-4.5.a" "libITKIOMeta-4.5.a" "libITKznz-4.5.a" "libITKniftiio-4.5.a" "libITKIONIFTI-4.5.a" "libITKNrrdIO-4.5.a" "libITKIONRRD-4.5.a" "libitkpng-4.5.a" "libITKIOPNG-4.5.a" "libITKIOStimulate-4.5.a" "libITKIOVTK-4.5.a" "libITKMesh-4.5.a" "libITKSpatialObjects-4.5.a" "libITKPath-4.5.a" "libITKLabelMap-4.5.a" "libITKQuadEdgeMesh-4.5.a" "libITKOptimizers-4.5.a" "libITKPolynomials-4.5.a" "libITKBiasCorrection-4.5.a" "libITKBioCell-4.5.a" "libITKDICOMParser-4.5.a" "libITKIOXML-4.5.a" "libITKIOSpatialObjects-4.5.a" "libITKFEM-4.5.a" "libITKgiftiio-4.5.a" "libITKIOMesh-4.5.a" "libitkhdf5_cpp-4.5.a" "libitkhdf5-4.5.a" "libITKIOCSV-4.5.a" "libITKIOIPL-4.5.a" "libITKIOGE-4.5.a" "libITKIOSiemens-4.5.a" "libITKIOHDF5-4.5.a" "libITKIOTransformBase-4.5.a" "libITKIOTransformHDF5-4.5.a" "libITKIOTransformInsightLegacy-4.5.a" "libITKIOTransformMatlab-4.5.a" "libITKKLMRegionGrowing-4.5.a" "libITKVTK-4.5.a" "libITKWatersheds-4.5.a" "libITKVideoCore-4.5.a" "libITKVideoIO-4.5.a" "libitkgdcmIOD-4.5.a" "libitkgdcmCommon-4.5.a" "libitkgdcmjpeg8-4.5.a" "libitkgdcmjpeg12-4.5.a" "libitkgdcmjpeg16-4.5.a" "libITKVNLInstantiation-4.5.a" "libitkv3p_lsqr-4.5.a" "libitkvcl-4.5.a" 
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)/usr/local/lib 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/ar rcu
CXX      := /usr/bin/g++ 
CC       := /usr/bin/gcc 
CXXFLAGS :=  -g -O0 -Wall $(shell /usr/local/bin/wx-config --cflags --debug)  -Wno-sign-compare $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/as 


##
## User defined environment variables
##
CodeLiteDir:=/Applications/codelite.app/Contents/SharedSupport/
Objects0=$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) $(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) $(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix) $(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) $(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) $(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/optical.cpp$(ObjectSuffix) 



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
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/main.cpp$(ObjectSuffix): main.cpp $(IntermediateDirectory)/main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(DependSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/main.cpp$(DependSuffix) -MM "main.cpp"

$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix) "main.cpp"

$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix): MainFrame.cpp $(IntermediateDirectory)/MainFrame.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/MainFrame.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MainFrame.cpp$(DependSuffix): MainFrame.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/MainFrame.cpp$(DependSuffix) -MM "MainFrame.cpp"

$(IntermediateDirectory)/MainFrame.cpp$(PreprocessSuffix): MainFrame.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MainFrame.cpp$(PreprocessSuffix) "MainFrame.cpp"

$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix): wxcrafter.cpp $(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/wxcrafter.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix): wxcrafter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix) -MM "wxcrafter.cpp"

$(IntermediateDirectory)/wxcrafter.cpp$(PreprocessSuffix): wxcrafter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/wxcrafter.cpp$(PreprocessSuffix) "wxcrafter.cpp"

$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix): wxcrafter_bitmaps.cpp $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/wxcrafter_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix): wxcrafter_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix) -MM "wxcrafter_bitmaps.cpp"

$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(PreprocessSuffix): wxcrafter_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(PreprocessSuffix) "wxcrafter_bitmaps.cpp"

$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix): scrolled_image_component.cpp $(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/scrolled_image_component.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix): scrolled_image_component.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix) -MM "scrolled_image_component.cpp"

$(IntermediateDirectory)/scrolled_image_component.cpp$(PreprocessSuffix): scrolled_image_component.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/scrolled_image_component.cpp$(PreprocessSuffix) "scrolled_image_component.cpp"

$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix): KDE.cpp $(IntermediateDirectory)/KDE.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/KDE.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/KDE.cpp$(DependSuffix): KDE.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/KDE.cpp$(DependSuffix) -MM "KDE.cpp"

$(IntermediateDirectory)/KDE.cpp$(PreprocessSuffix): KDE.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/KDE.cpp$(PreprocessSuffix) "KDE.cpp"

$(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix): LineDetector.cpp $(IntermediateDirectory)/LineDetector.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/LineDetector.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/LineDetector.cpp$(DependSuffix): LineDetector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/LineDetector.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/LineDetector.cpp$(DependSuffix) -MM "LineDetector.cpp"

$(IntermediateDirectory)/LineDetector.cpp$(PreprocessSuffix): LineDetector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/LineDetector.cpp$(PreprocessSuffix) "LineDetector.cpp"

$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix): MyUtil.cpp $(IntermediateDirectory)/MyUtil.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/MyUtil.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MyUtil.cpp$(DependSuffix): MyUtil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/MyUtil.cpp$(DependSuffix) -MM "MyUtil.cpp"

$(IntermediateDirectory)/MyUtil.cpp$(PreprocessSuffix): MyUtil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MyUtil.cpp$(PreprocessSuffix) "MyUtil.cpp"

$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix): Rat.cpp $(IntermediateDirectory)/Rat.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/Rat.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Rat.cpp$(DependSuffix): Rat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/Rat.cpp$(DependSuffix) -MM "Rat.cpp"

$(IntermediateDirectory)/Rat.cpp$(PreprocessSuffix): Rat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Rat.cpp$(PreprocessSuffix) "Rat.cpp"

$(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix): DlgOptical.cpp $(IntermediateDirectory)/DlgOptical.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/DlgOptical.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/DlgOptical.cpp$(DependSuffix): DlgOptical.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/DlgOptical.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/DlgOptical.cpp$(DependSuffix) -MM "DlgOptical.cpp"

$(IntermediateDirectory)/DlgOptical.cpp$(PreprocessSuffix): DlgOptical.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/DlgOptical.cpp$(PreprocessSuffix) "DlgOptical.cpp"

$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix): optical_macrat_01_bitmaps.cpp $(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/optical_macrat_01_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(DependSuffix): optical_macrat_01_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(DependSuffix) -MM "optical_macrat_01_bitmaps.cpp"

$(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(PreprocessSuffix): optical_macrat_01_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/optical_macrat_01_bitmaps.cpp$(PreprocessSuffix) "optical_macrat_01_bitmaps.cpp"

$(IntermediateDirectory)/optical.cpp$(ObjectSuffix): optical.cpp $(IntermediateDirectory)/optical.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/optical.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/optical.cpp$(ObjectSuffix) $(IncludePath)
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
	$(RM) ".build-debug/Vienna"


