##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=macRat
ConfigurationName      :=Debug
WorkspacePath          := "/Users/cclee/project/macRat"
ProjectPath            := "/Users/cclee/project/macRat"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=李建誠
Date                   :=29/10/2015
CodeLitePath           :="/Users/cclee/Library/Application Support/codelite"
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
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). $(IncludeSwitch)/usr/local/include 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)gsl $(LibrarySwitch)gslcblas $(LibrarySwitch)opencv_core $(LibrarySwitch)opencv_video $(LibrarySwitch)opencv_highgui $(LibrarySwitch)opencv_imgproc $(LibrarySwitch)opencv_imgcodecs 
ArLibs                 :=  "libgsl.dylib" "libgslcblas.dylib" "libopencv_core.dylib" "libopencv_video.dylib" "libopencv_highgui.dylib" "libopencv_imgproc.dylib" "libopencv_imgcodecs.dylib" 
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
	$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) $(IntermediateDirectory)/DlgSelectFolder.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) $(IntermediateDirectory)/dlg_select_folder.cpp$(ObjectSuffix) 



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
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(DependSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/main.cpp$(DependSuffix) -MM "main.cpp"

$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix) "main.cpp"

$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix): MainFrame.cpp $(IntermediateDirectory)/MainFrame.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/MainFrame.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MainFrame.cpp$(DependSuffix): MainFrame.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MainFrame.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/MainFrame.cpp$(DependSuffix) -MM "MainFrame.cpp"

$(IntermediateDirectory)/MainFrame.cpp$(PreprocessSuffix): MainFrame.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MainFrame.cpp$(PreprocessSuffix) "MainFrame.cpp"

$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix): wxcrafter.cpp $(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/wxcrafter.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix): wxcrafter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/wxcrafter.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/wxcrafter.cpp$(DependSuffix) -MM "wxcrafter.cpp"

$(IntermediateDirectory)/wxcrafter.cpp$(PreprocessSuffix): wxcrafter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/wxcrafter.cpp$(PreprocessSuffix) "wxcrafter.cpp"

$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix): wxcrafter_bitmaps.cpp $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/wxcrafter_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix): wxcrafter_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(DependSuffix) -MM "wxcrafter_bitmaps.cpp"

$(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(PreprocessSuffix): wxcrafter_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/wxcrafter_bitmaps.cpp$(PreprocessSuffix) "wxcrafter_bitmaps.cpp"

$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix): scrolled_image_component.cpp $(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/scrolled_image_component.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix): scrolled_image_component.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/scrolled_image_component.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/scrolled_image_component.cpp$(DependSuffix) -MM "scrolled_image_component.cpp"

$(IntermediateDirectory)/scrolled_image_component.cpp$(PreprocessSuffix): scrolled_image_component.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/scrolled_image_component.cpp$(PreprocessSuffix) "scrolled_image_component.cpp"

$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix): KDE.cpp $(IntermediateDirectory)/KDE.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/KDE.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/KDE.cpp$(DependSuffix): KDE.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/KDE.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/KDE.cpp$(DependSuffix) -MM "KDE.cpp"

$(IntermediateDirectory)/KDE.cpp$(PreprocessSuffix): KDE.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/KDE.cpp$(PreprocessSuffix) "KDE.cpp"

$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix): MyUtil.cpp $(IntermediateDirectory)/MyUtil.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/MyUtil.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MyUtil.cpp$(DependSuffix): MyUtil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MyUtil.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/MyUtil.cpp$(DependSuffix) -MM "MyUtil.cpp"

$(IntermediateDirectory)/MyUtil.cpp$(PreprocessSuffix): MyUtil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MyUtil.cpp$(PreprocessSuffix) "MyUtil.cpp"

$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix): Rat.cpp $(IntermediateDirectory)/Rat.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/Rat.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Rat.cpp$(DependSuffix): Rat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Rat.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/Rat.cpp$(DependSuffix) -MM "Rat.cpp"

$(IntermediateDirectory)/Rat.cpp$(PreprocessSuffix): Rat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Rat.cpp$(PreprocessSuffix) "Rat.cpp"

$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix): BendPoint.cpp $(IntermediateDirectory)/BendPoint.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/BendPoint.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/BendPoint.cpp$(DependSuffix): BendPoint.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/BendPoint.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/BendPoint.cpp$(DependSuffix) -MM "BendPoint.cpp"

$(IntermediateDirectory)/BendPoint.cpp$(PreprocessSuffix): BendPoint.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/BendPoint.cpp$(PreprocessSuffix) "BendPoint.cpp"

$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix): gnuplot_i.cpp $(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/gnuplot_i.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix): gnuplot_i.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/gnuplot_i.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/gnuplot_i.cpp$(DependSuffix) -MM "gnuplot_i.cpp"

$(IntermediateDirectory)/gnuplot_i.cpp$(PreprocessSuffix): gnuplot_i.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/gnuplot_i.cpp$(PreprocessSuffix) "gnuplot_i.cpp"

$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix): DlgOpticalInput.cpp $(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/DlgOpticalInput.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix): DlgOpticalInput.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/DlgOpticalInput.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/DlgOpticalInput.cpp$(DependSuffix) -MM "DlgOpticalInput.cpp"

$(IntermediateDirectory)/DlgOpticalInput.cpp$(PreprocessSuffix): DlgOpticalInput.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/DlgOpticalInput.cpp$(PreprocessSuffix) "DlgOpticalInput.cpp"

$(IntermediateDirectory)/DlgSelectFolder.cpp$(ObjectSuffix): DlgSelectFolder.cpp $(IntermediateDirectory)/DlgSelectFolder.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/DlgSelectFolder.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/DlgSelectFolder.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/DlgSelectFolder.cpp$(DependSuffix): DlgSelectFolder.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/DlgSelectFolder.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/DlgSelectFolder.cpp$(DependSuffix) -MM "DlgSelectFolder.cpp"

$(IntermediateDirectory)/DlgSelectFolder.cpp$(PreprocessSuffix): DlgSelectFolder.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/DlgSelectFolder.cpp$(PreprocessSuffix) "DlgSelectFolder.cpp"

$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix): dlg_optical_macrat_bitmaps.cpp $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/dlg_optical_macrat_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix): dlg_optical_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(DependSuffix) -MM "dlg_optical_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(PreprocessSuffix): dlg_optical_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_macrat_bitmaps.cpp$(PreprocessSuffix) "dlg_optical_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix): dlg_optical_input_base_macrat_bitmaps.cpp $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/dlg_optical_input_base_macrat_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix): dlg_optical_input_base_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(DependSuffix) -MM "dlg_optical_input_base_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(PreprocessSuffix): dlg_optical_input_base_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_input_base_macrat_bitmaps.cpp$(PreprocessSuffix) "dlg_optical_input_base_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(ObjectSuffix): dlg_select_folder_macrat_bitmaps.cpp $(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/dlg_select_folder_macrat_bitmaps.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(DependSuffix): dlg_select_folder_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(DependSuffix) -MM "dlg_select_folder_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(PreprocessSuffix): dlg_select_folder_macrat_bitmaps.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_select_folder_macrat_bitmaps.cpp$(PreprocessSuffix) "dlg_select_folder_macrat_bitmaps.cpp"

$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix): dlg_optical_input_base.cpp $(IntermediateDirectory)/dlg_optical_input_base.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/dlg_optical_input_base.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_optical_input_base.cpp$(DependSuffix): dlg_optical_input_base.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_optical_input_base.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_optical_input_base.cpp$(DependSuffix) -MM "dlg_optical_input_base.cpp"

$(IntermediateDirectory)/dlg_optical_input_base.cpp$(PreprocessSuffix): dlg_optical_input_base.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_optical_input_base.cpp$(PreprocessSuffix) "dlg_optical_input_base.cpp"

$(IntermediateDirectory)/dlg_select_folder.cpp$(ObjectSuffix): dlg_select_folder.cpp $(IntermediateDirectory)/dlg_select_folder.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/cclee/project/macRat/dlg_select_folder.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dlg_select_folder.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dlg_select_folder.cpp$(DependSuffix): dlg_select_folder.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dlg_select_folder.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dlg_select_folder.cpp$(DependSuffix) -MM "dlg_select_folder.cpp"

$(IntermediateDirectory)/dlg_select_folder.cpp$(PreprocessSuffix): dlg_select_folder.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dlg_select_folder.cpp$(PreprocessSuffix) "dlg_select_folder.cpp"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


