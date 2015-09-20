##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=macRat
ConfigurationName      :=Debug
WorkspacePath          := "/Users/CCLee/Project/macRat"
ProjectPath            := "/Users/CCLee/Project/macRat"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=CCLee
Date                   :=20/09/2015
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
ObjectsFileList        :="macRat.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  $(shell /usr/local/bin/wx-config --libs --debug)
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). $(IncludeSwitch)/usr/local/include $(IncludeSwitch)/usr/local/include/ITK-4.8 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)gsl $(LibrarySwitch)gslcblas $(LibrarySwitch)opencv_core $(LibrarySwitch)opencv_video $(LibrarySwitch)opencv_highgui $(LibrarySwitch)opencv_imgproc $(LibrarySwitch)opencv_imgcodecs $(LibrarySwitch)itkdouble-conversion-4.8 $(LibrarySwitch)itksys-4.8 $(LibrarySwitch)itkvnl_algo-4.8 $(LibrarySwitch)itkvnl-4.8 $(LibrarySwitch)itkv3p_netlib-4.8 $(LibrarySwitch)ITKCommon-4.8 $(LibrarySwitch)itkNetlibSlatec-4.8 $(LibrarySwitch)ITKStatistics-4.8 $(LibrarySwitch)ITKIOImageBase-4.8 $(LibrarySwitch)ITKIOBMP-4.8 $(LibrarySwitch)ITKIOBioRad-4.8 $(LibrarySwitch)ITKEXPAT-4.8 $(LibrarySwitch)itkopenjpeg-4.8 $(LibrarySwitch)itkzlib-4.8 $(LibrarySwitch)itkgdcmDICT-4.8 $(LibrarySwitch)itkgdcmMSFF-4.8 $(LibrarySwitch)ITKIOGDCM-4.8 $(LibrarySwitch)ITKIOGIPL-4.8 $(LibrarySwitch)itkjpeg-4.8 $(LibrarySwitch)ITKIOJPEG-4.8 $(LibrarySwitch)itktiff-4.8 $(LibrarySwitch)ITKIOTIFF-4.8 $(LibrarySwitch)ITKIOLSM-4.8 $(LibrarySwitch)ITKMetaIO-4.8 $(LibrarySwitch)ITKIOMeta-4.8 $(LibrarySwitch)ITKznz-4.8 $(LibrarySwitch)ITKniftiio-4.8 $(LibrarySwitch)ITKIONIFTI-4.8 $(LibrarySwitch)ITKNrrdIO-4.8 $(LibrarySwitch)ITKIONRRD-4.8 $(LibrarySwitch)itkpng-4.8 $(LibrarySwitch)ITKIOPNG-4.8 $(LibrarySwitch)ITKIOStimulate-4.8 $(LibrarySwitch)ITKIOVTK-4.8 $(LibrarySwitch)ITKMesh-4.8 $(LibrarySwitch)ITKSpatialObjects-4.8 $(LibrarySwitch)ITKPath-4.8 $(LibrarySwitch)ITKLabelMap-4.8 $(LibrarySwitch)ITKQuadEdgeMesh-4.8 $(LibrarySwitch)ITKOptimizers-4.8 $(LibrarySwitch)ITKPolynomials-4.8 $(LibrarySwitch)ITKBiasCorrection-4.8 $(LibrarySwitch)ITKBioCell-4.8 $(LibrarySwitch)ITKDICOMParser-4.8 $(LibrarySwitch)ITKIOXML-4.8 $(LibrarySwitch)ITKIOSpatialObjects-4.8 $(LibrarySwitch)ITKFEM-4.8 $(LibrarySwitch)ITKgiftiio-4.8 $(LibrarySwitch)ITKIOMesh-4.8 $(LibrarySwitch)itkhdf5_cpp-4.8 $(LibrarySwitch)itkhdf5-4.8 $(LibrarySwitch)ITKIOCSV-4.8 $(LibrarySwitch)ITKIOIPL-4.8 $(LibrarySwitch)ITKIOGE-4.8 $(LibrarySwitch)ITKIOSiemens-4.8 $(LibrarySwitch)ITKIOHDF5-4.8 $(LibrarySwitch)ITKIOTransformBase-4.8 $(LibrarySwitch)ITKIOTransformHDF5-4.8 $(LibrarySwitch)ITKIOTransformInsightLegacy-4.8 $(LibrarySwitch)ITKIOTransformMatlab-4.8 $(LibrarySwitch)ITKKLMRegionGrowing-4.8 $(LibrarySwitch)ITKVTK-4.8 $(LibrarySwitch)ITKWatersheds-4.8 $(LibrarySwitch)ITKVideoCore-4.8 $(LibrarySwitch)ITKVideoIO-4.8 $(LibrarySwitch)itkgdcmIOD-4.8 $(LibrarySwitch)itkgdcmCommon-4.8 $(LibrarySwitch)itkgdcmjpeg8-4.8 $(LibrarySwitch)itkgdcmjpeg12-4.8 $(LibrarySwitch)itkgdcmjpeg16-4.8 $(LibrarySwitch)ITKVNLInstantiation-4.8 $(LibrarySwitch)itkv3p_lsqr-4.8 $(LibrarySwitch)itkvcl-4.8 
ArLibs                 :=  "libgsl.dylib" "libgslcblas.dylib" "libopencv_core.dylib" "libopencv_video.dylib" "libopencv_highgui.dylib" "libopencv_imgproc.dylib" "libopencv_imgcodecs.dylib" "libitkdouble-conversion-4.8.a" "libitksys-4.8.a" "libitkvnl_algo-4.8.a" "libitkvnl-4.8.a" "libitkv3p_netlib-4.8.a" "libITKCommon-4.8.a" "libitkNetlibSlatec-4.8.a" "libITKStatistics-4.8.a" "libITKIOImageBase-4.8.a" "libITKIOBMP-4.8.a" "libITKIOBioRad-4.8.a" "libITKEXPAT-4.8.a" "libitkopenjpeg-4.8.a" "libitkzlib-4.8.a" "libitkgdcmDICT-4.8.a" "libitkgdcmMSFF-4.8.a" "libITKIOGDCM-4.8.a" "libITKIOGIPL-4.8.a" "libitkjpeg-4.8.a" "libITKIOJPEG-4.8.a" "libitktiff-4.8.a" "libITKIOTIFF-4.8.a" "libITKIOLSM-4.8.a" "libITKMetaIO-4.8.a" "libITKIOMeta-4.8.a" "libITKznz-4.8.a" "libITKniftiio-4.8.a" "libITKIONIFTI-4.8.a" "libITKNrrdIO-4.8.a" "libITKIONRRD-4.8.a" "libitkpng-4.8.a" "libITKIOPNG-4.8.a" "libITKIOStimulate-4.8.a" "libITKIOVTK-4.8.a" "libITKMesh-4.8.a" "libITKSpatialObjects-4.8.a" "libITKPath-4.8.a" "libITKLabelMap-4.8.a" "libITKQuadEdgeMesh-4.8.a" "libITKOptimizers-4.8.a" "libITKPolynomials-4.8.a" "libITKBiasCorrection-4.8.a" "libITKBioCell-4.8.a" "libITKDICOMParser-4.8.a" "libITKIOXML-4.8.a" "libITKIOSpatialObjects-4.8.a" "libITKFEM-4.8.a" "libITKgiftiio-4.8.a" "libITKIOMesh-4.8.a" "libitkhdf5_cpp-4.8.a" "libitkhdf5-4.8.a" "libITKIOCSV-4.8.a" "libITKIOIPL-4.8.a" "libITKIOGE-4.8.a" "libITKIOSiemens-4.8.a" "libITKIOHDF5-4.8.a" "libITKIOTransformBase-4.8.a" "libITKIOTransformHDF5-4.8.a" "libITKIOTransformInsightLegacy-4.8.a" "libITKIOTransformMatlab-4.8.a" "libITKKLMRegionGrowing-4.8.a" "libITKVTK-4.8.a" "libITKWatersheds-4.8.a" "libITKVideoCore-4.8.a" "libITKVideoIO-4.8.a" "libitkgdcmIOD-4.8.a" "libitkgdcmCommon-4.8.a" "libitkgdcmjpeg8-4.8.a" "libitkgdcmjpeg12-4.8.a" "libitkgdcmjpeg16-4.8.a" "libITKVNLInstantiation-4.8.a" "libitkv3p_lsqr-4.8.a" "libitkvcl-4.8.a" 
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)/usr/local/lib 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/ar rcu
CXX      := /usr/bin/g++
CC       := /usr/bin/gcc
CXXFLAGS :=  -g $(shell /usr/local/bin/wx-config --cflags --debug) -Wno-sign-compare $(Preprocessors)
CFLAGS   :=  -g -Wall $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/as


##
## User defined environment variables
##
CodeLiteDir:=/Applications/codelite.app/Contents/SharedSupport/
Objects0=$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) $(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) $(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) $(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) $(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) $(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d ./Debug || $(MakeDirCommand) ./Debug


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

$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix): BendPoint.cpp $(IntermediateDirectory)/BendPoint.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/BendPoint.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/BendPoint.cpp$(DependSuffix): BendPoint.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/BendPoint.cpp$(DependSuffix) -MM "BendPoint.cpp"

$(IntermediateDirectory)/BendPoint.cpp$(PreprocessSuffix): BendPoint.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/BendPoint.cpp$(PreprocessSuffix) "BendPoint.cpp"

$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix): gnuplot_i.cpp $(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/gnuplot_i.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix): gnuplot_i.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix) -MM "gnuplot_i.cpp"

$(IntermediateDirectory)/gnuplot_i.cpp$(PreprocessSuffix): gnuplot_i.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/gnuplot_i.cpp$(PreprocessSuffix) "gnuplot_i.cpp"

$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix): DlgOpticalInput.cpp $(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/DlgOpticalInput.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix): DlgOpticalInput.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix) -MM "DlgOpticalInput.cpp"

$(IntermediateDirectory)/DlgOpticalInput.cpp$(PreprocessSuffix): DlgOpticalInput.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/DlgOpticalInput.cpp$(PreprocessSuffix) "DlgOpticalInput.cpp"

$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix): dlg_optical_macrat_bitmaps.cpp $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/dlg_optical_macrat_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix): dlg_optical_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix) -MM "dlg_optical_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(PreprocessSuffix): dlg_optical_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(PreprocessSuffix) "dlg_optical_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix): dlg_optical_input_base_macrat_bitmaps.cpp $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/dlg_optical_input_base_macrat_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix): dlg_optical_input_base_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix) -MM "dlg_optical_input_base_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(PreprocessSuffix): dlg_optical_input_base_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(PreprocessSuffix) "dlg_optical_input_base_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix): dlg_optical_input_base.cpp $(IntermediateDirectory)/dlg_optical_input_base.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/CCLee/Project/macRat/dlg_optical_input_base.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) $(IncludePath)
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


