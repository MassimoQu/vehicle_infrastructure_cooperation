# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/massimo/vehicle_infrastructure_cooperation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/massimo/vehicle_infrastructure_cooperation/build

# Include any dependencies generated for this target.
include CMakeFiles/TEST.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TEST.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TEST.dir/flags.make

CMakeFiles/TEST.dir/test_true_boxing.cpp.o: CMakeFiles/TEST.dir/flags.make
CMakeFiles/TEST.dir/test_true_boxing.cpp.o: ../test_true_boxing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/massimo/vehicle_infrastructure_cooperation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TEST.dir/test_true_boxing.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TEST.dir/test_true_boxing.cpp.o -c /home/massimo/vehicle_infrastructure_cooperation/test_true_boxing.cpp

CMakeFiles/TEST.dir/test_true_boxing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TEST.dir/test_true_boxing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/massimo/vehicle_infrastructure_cooperation/test_true_boxing.cpp > CMakeFiles/TEST.dir/test_true_boxing.cpp.i

CMakeFiles/TEST.dir/test_true_boxing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TEST.dir/test_true_boxing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/massimo/vehicle_infrastructure_cooperation/test_true_boxing.cpp -o CMakeFiles/TEST.dir/test_true_boxing.cpp.s

# Object files for target TEST
TEST_OBJECTS = \
"CMakeFiles/TEST.dir/test_true_boxing.cpp.o"

# External object files for target TEST
TEST_EXTERNAL_OBJECTS =

TEST: CMakeFiles/TEST.dir/test_true_boxing.cpp.o
TEST: CMakeFiles/TEST.dir/build.make
TEST: /usr/local/lib/libpcl_surface.so
TEST: /usr/local/lib/libpcl_keypoints.so
TEST: /usr/local/lib/libpcl_tracking.so
TEST: /usr/local/lib/libpcl_recognition.so
TEST: /usr/local/lib/libpcl_stereo.so
TEST: /usr/local/lib/libpcl_outofcore.so
TEST: /usr/local/lib/libpcl_people.so
TEST: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
TEST: /usr/lib/libOpenNI.so
TEST: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
TEST: /usr/lib/libOpenNI2.so
TEST: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
TEST: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
TEST: /usr/lib/x86_64-linux-gnu/libqhull_r.so
TEST: /usr/local/lib/libpcl_registration.so
TEST: /usr/local/lib/libpcl_segmentation.so
TEST: /usr/local/lib/libpcl_features.so
TEST: /usr/local/lib/libpcl_filters.so
TEST: /usr/local/lib/libpcl_sample_consensus.so
TEST: /usr/local/lib/libpcl_ml.so
TEST: /usr/local/lib/libpcl_visualization.so
TEST: /usr/local/lib/libpcl_search.so
TEST: /usr/local/lib/libpcl_kdtree.so
TEST: /usr/local/lib/libpcl_io.so
TEST: /usr/local/lib/libpcl_octree.so
TEST: /usr/lib/libOpenNI.so
TEST: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
TEST: /usr/lib/libOpenNI2.so
TEST: /usr/local/lib/libvtkChartsCore-7.1.so.1
TEST: /usr/local/lib/libvtkInfovisCore-7.1.so.1
TEST: /usr/local/lib/libvtkInteractionImage-7.1.so.1
TEST: /usr/local/lib/libvtkIOGeometry-7.1.so.1
TEST: /usr/local/lib/libvtkIOLegacy-7.1.so.1
TEST: /usr/local/lib/libvtkIOPLY-7.1.so.1
TEST: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
TEST: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
TEST: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
TEST: /usr/local/lib/libvtkViewsCore-7.1.so.1
TEST: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
TEST: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
TEST: /usr/local/lib/libvtkImagingSources-7.1.so.1
TEST: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
TEST: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
TEST: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
TEST: /usr/local/lib/libvtkfreetype-7.1.so.1
TEST: /usr/local/lib/libvtkImagingColor-7.1.so.1
TEST: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
TEST: /usr/local/lib/libvtkIOXML-7.1.so.1
TEST: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
TEST: /usr/local/lib/libvtkIOCore-7.1.so.1
TEST: /usr/local/lib/libvtkexpat-7.1.so.1
TEST: /usr/local/lib/libvtkGUISupportQt-7.1.so.1
TEST: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
TEST: /usr/local/lib/libvtkImagingFourier-7.1.so.1
TEST: /usr/local/lib/libvtkalglib-7.1.so.1
TEST: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
TEST: /usr/local/lib/libvtkImagingCore-7.1.so.1
TEST: /usr/local/lib/libvtkRenderingCore-7.1.so.1
TEST: /usr/local/lib/libvtkCommonColor-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersSources-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
TEST: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
TEST: /usr/local/lib/libvtkFiltersCore-7.1.so.1
TEST: /usr/local/lib/libvtkIOImage-7.1.so.1
TEST: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
TEST: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
TEST: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
TEST: /usr/local/lib/libvtkCommonMisc-7.1.so.1
TEST: /usr/local/lib/libvtkCommonMath-7.1.so.1
TEST: /usr/local/lib/libvtkCommonSystem-7.1.so.1
TEST: /usr/local/lib/libvtkCommonCore-7.1.so.1
TEST: /usr/local/lib/libvtksys-7.1.so.1
TEST: /usr/local/lib/libvtkDICOMParser-7.1.so.1
TEST: /usr/local/lib/libvtkmetaio-7.1.so.1
TEST: /usr/local/lib/libvtkpng-7.1.so.1
TEST: /usr/local/lib/libvtktiff-7.1.so.1
TEST: /usr/local/lib/libvtkzlib-7.1.so.1
TEST: /usr/local/lib/libvtkjpeg-7.1.so.1
TEST: /usr/lib/x86_64-linux-gnu/libm.so
TEST: /usr/local/lib/libvtkglew-7.1.so.1
TEST: /usr/lib/x86_64-linux-gnu/libSM.so
TEST: /usr/lib/x86_64-linux-gnu/libICE.so
TEST: /usr/lib/x86_64-linux-gnu/libX11.so
TEST: /usr/lib/x86_64-linux-gnu/libXext.so
TEST: /usr/lib/x86_64-linux-gnu/libXt.so
TEST: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
TEST: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
TEST: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
TEST: /usr/local/lib/libpcl_common.so
TEST: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
TEST: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
TEST: CMakeFiles/TEST.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/massimo/vehicle_infrastructure_cooperation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable TEST"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TEST.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TEST.dir/build: TEST

.PHONY : CMakeFiles/TEST.dir/build

CMakeFiles/TEST.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TEST.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TEST.dir/clean

CMakeFiles/TEST.dir/depend:
	cd /home/massimo/vehicle_infrastructure_cooperation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/massimo/vehicle_infrastructure_cooperation /home/massimo/vehicle_infrastructure_cooperation /home/massimo/vehicle_infrastructure_cooperation/build /home/massimo/vehicle_infrastructure_cooperation/build /home/massimo/vehicle_infrastructure_cooperation/build/CMakeFiles/TEST.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TEST.dir/depend
