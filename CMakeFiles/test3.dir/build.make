# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/jozef/Documents/QT/3DScan

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jozef/Documents/QT/3DScan

# Include any dependencies generated for this target.
include CMakeFiles/test3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test3.dir/flags.make

CMakeFiles/test3.dir/main.cpp.o: CMakeFiles/test3.dir/flags.make
CMakeFiles/test3.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jozef/Documents/QT/3DScan/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test3.dir/main.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test3.dir/main.cpp.o -c /home/jozef/Documents/QT/3DScan/main.cpp

CMakeFiles/test3.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test3.dir/main.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jozef/Documents/QT/3DScan/main.cpp > CMakeFiles/test3.dir/main.cpp.i

CMakeFiles/test3.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test3.dir/main.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jozef/Documents/QT/3DScan/main.cpp -o CMakeFiles/test3.dir/main.cpp.s

CMakeFiles/test3.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/test3.dir/main.cpp.o.requires

CMakeFiles/test3.dir/main.cpp.o.provides: CMakeFiles/test3.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/test3.dir/build.make CMakeFiles/test3.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/test3.dir/main.cpp.o.provides

CMakeFiles/test3.dir/main.cpp.o.provides.build: CMakeFiles/test3.dir/main.cpp.o


CMakeFiles/test3.dir/kinect.cpp.o: CMakeFiles/test3.dir/flags.make
CMakeFiles/test3.dir/kinect.cpp.o: kinect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jozef/Documents/QT/3DScan/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test3.dir/kinect.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test3.dir/kinect.cpp.o -c /home/jozef/Documents/QT/3DScan/kinect.cpp

CMakeFiles/test3.dir/kinect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test3.dir/kinect.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jozef/Documents/QT/3DScan/kinect.cpp > CMakeFiles/test3.dir/kinect.cpp.i

CMakeFiles/test3.dir/kinect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test3.dir/kinect.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jozef/Documents/QT/3DScan/kinect.cpp -o CMakeFiles/test3.dir/kinect.cpp.s

CMakeFiles/test3.dir/kinect.cpp.o.requires:

.PHONY : CMakeFiles/test3.dir/kinect.cpp.o.requires

CMakeFiles/test3.dir/kinect.cpp.o.provides: CMakeFiles/test3.dir/kinect.cpp.o.requires
	$(MAKE) -f CMakeFiles/test3.dir/build.make CMakeFiles/test3.dir/kinect.cpp.o.provides.build
.PHONY : CMakeFiles/test3.dir/kinect.cpp.o.provides

CMakeFiles/test3.dir/kinect.cpp.o.provides.build: CMakeFiles/test3.dir/kinect.cpp.o


# Object files for target test3
test3_OBJECTS = \
"CMakeFiles/test3.dir/main.cpp.o" \
"CMakeFiles/test3.dir/kinect.cpp.o"

# External object files for target test3
test3_EXTERNAL_OBJECTS =

test3: CMakeFiles/test3.dir/main.cpp.o
test3: CMakeFiles/test3.dir/kinect.cpp.o
test3: CMakeFiles/test3.dir/build.make
test3: /usr/lib/x86_64-linux-gnu/libboost_system.so
test3: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
test3: /usr/lib/x86_64-linux-gnu/libboost_thread.so
test3: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
test3: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
test3: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
test3: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
test3: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
test3: /usr/lib/x86_64-linux-gnu/libboost_regex.so
test3: /usr/lib/x86_64-linux-gnu/libpthread.so
test3: /usr/local/lib/libpcl_common.so
test3: /usr/local/lib/libpcl_octree.so
test3: /usr/lib/libOpenNI.so
test3: /usr/lib/libOpenNI2.so
test3: /usr/local/lib/libpcl_io.so
test3: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
test3: /usr/local/lib/libpcl_kdtree.so
test3: /usr/local/lib/libpcl_search.so
test3: /usr/local/lib/libpcl_visualization.so
test3: /usr/lib/x86_64-linux-gnu/libboost_system.so
test3: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
test3: /usr/lib/x86_64-linux-gnu/libboost_thread.so
test3: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
test3: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
test3: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
test3: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
test3: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
test3: /usr/lib/x86_64-linux-gnu/libboost_regex.so
test3: /usr/lib/x86_64-linux-gnu/libpthread.so
test3: /usr/lib/libOpenNI.so
test3: /usr/lib/libOpenNI2.so
test3: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
test3: /usr/local/lib/libvtkIOInfovis-8.1.so.1
test3: /usr/local/lib/libvtkRenderingContextOpenGL2-8.1.so.1
test3: /usr/local/lib/libvtkTestingRendering-8.1.so.1
test3: /usr/local/lib/libvtkViewsContext2D-8.1.so.1
test3: /usr/local/lib/libvtkFiltersGeneric-8.1.so.1
test3: /usr/local/lib/libvtkTestingGenericBridge-8.1.so.1
test3: /usr/local/lib/libvtkDomainsChemistryOpenGL2-8.1.so.1
test3: /usr/local/lib/libvtkIOAMR-8.1.so.1
test3: /usr/local/lib/libvtkIOExodus-8.1.so.1
test3: /usr/local/lib/libvtkRenderingVolumeOpenGL2-8.1.so.1
test3: /usr/local/lib/libvtkFiltersFlowPaths-8.1.so.1
test3: /usr/local/lib/libvtkFiltersHyperTree-8.1.so.1
test3: /usr/local/lib/libvtkImagingStencil-8.1.so.1
test3: /usr/local/lib/libvtkFiltersParallelDIY2-8.1.so.1
test3: /usr/local/lib/libvtkFiltersParallelGeometry-8.1.so.1
test3: /usr/local/lib/libvtkIOParallelXML-8.1.so.1
test3: /usr/local/lib/libvtkFiltersParallelImaging-8.1.so.1
test3: /usr/local/lib/libvtkFiltersParallelMPI-8.1.so.1
test3: /usr/local/lib/libvtkFiltersParallelVerdict-8.1.so.1
test3: /usr/local/lib/libvtkFiltersPoints-8.1.so.1
test3: /usr/local/lib/libvtkFiltersProgrammable-8.1.so.1
test3: /usr/local/lib/libvtkFiltersSMP-8.1.so.1
test3: /usr/local/lib/libvtkFiltersSelection-8.1.so.1
test3: /usr/local/lib/libvtkFiltersTopology-8.1.so.1
test3: /usr/local/lib/libvtkGUISupportQtSQL-8.1.so.1
test3: /usr/local/lib/libvtkGeovisCore-8.1.so.1
test3: /usr/local/lib/libvtkIOEnSight-8.1.so.1
test3: /usr/local/lib/libvtkIOExportOpenGL2-8.1.so.1
test3: /usr/local/lib/libvtkInteractionImage-8.1.so.1
test3: /usr/local/lib/libvtkIOImport-8.1.so.1
test3: /usr/local/lib/libvtkIOLSDyna-8.1.so.1
test3: /usr/local/lib/libvtkIOMINC-8.1.so.1
test3: /usr/local/lib/libvtkIOMPIImage-8.1.so.1
test3: /usr/local/lib/libvtkIOMPIParallel-8.1.so.1
test3: /usr/local/lib/libvtkIOMovie-8.1.so.1
test3: /usr/local/lib/libvtkIOParallelNetCDF-8.1.so.1
test3: /usr/local/lib/libvtkTestingIOSQL-8.1.so.1
test3: /usr/local/lib/libvtkIOTecplotTable-8.1.so.1
test3: /usr/local/lib/libvtkIOVideo-8.1.so.1
test3: /usr/local/lib/libvtkImagingStatistics-8.1.so.1
test3: /usr/local/lib/libvtkRenderingImage-8.1.so.1
test3: /usr/local/lib/libvtkImagingMorphological-8.1.so.1
test3: /usr/local/lib/libvtkRenderingQt-8.1.so.1
test3: /usr/local/lib/libvtkViewsQt-8.1.so.1
test3: /usr/local/lib/libopencv_stitching.so.3.4.2
test3: /usr/local/lib/libopencv_viz.so.3.4.2
test3: /usr/local/lib/libopencv_videostab.so.3.4.2
test3: /usr/local/lib/libopencv_superres.so.3.4.2
test3: /usr/local/lib/libopencv_ml.so.3.4.2
test3: /usr/local/lib/libopencv_photo.so.3.4.2
test3: /usr/local/lib/libopencv_dnn.so.3.4.2
test3: /usr/local/lib/libopencv_shape.so.3.4.2
test3: /usr/local/lib/libopencv_video.so.3.4.2
test3: /usr/local/lib/libopencv_objdetect.so.3.4.2
test3: /usr/local/lib/libopencv_calib3d.so.3.4.2
test3: /usr/local/lib/libfreenect2.so
test3: /usr/local/lib/libpcl_common.so
test3: /usr/local/lib/libpcl_octree.so
test3: /usr/local/lib/libpcl_io.so
test3: /usr/local/lib/libpcl_kdtree.so
test3: /usr/local/lib/libpcl_search.so
test3: /usr/local/lib/libpcl_visualization.so
test3: /usr/local/lib/libfreenect2.so
test3: /usr/local/lib/libvtklibxml2-8.1.so.1
test3: /usr/local/lib/libvtkDomainsChemistry-8.1.so.1
test3: /usr/local/lib/libvtkFiltersAMR-8.1.so.1
test3: /usr/local/lib/libvtkImagingMath-8.1.so.1
test3: /usr/local/lib/libvtkFiltersVerdict-8.1.so.1
test3: /usr/local/lib/libvtkverdict-8.1.so.1
test3: /usr/local/lib/libvtkIOSQL-8.1.so.1
test3: /usr/local/lib/libvtksqlite-8.1.so.1
test3: /usr/lib/x86_64-linux-gnu/libQt5Sql.so.5.5.1
test3: /usr/local/lib/libvtkproj4-8.1.so.1
test3: /usr/local/lib/libvtkIOParallel-8.1.so.1
test3: /usr/local/lib/libvtkexoIIc-8.1.so.1
test3: /usr/local/lib/libvtkFiltersParallel-8.1.so.1
test3: /usr/local/lib/libvtkIONetCDF-8.1.so.1
test3: /usr/local/lib/libvtknetcdfcpp-8.1.so.1
test3: /usr/local/lib/libvtkjsoncpp-8.1.so.1
test3: /usr/local/lib/libvtkoggtheora-8.1.so.1
test3: /usr/local/lib/libvtkNetCDF-8.1.so.1
test3: /usr/local/lib/libvtkhdf5_hl-8.1.so.1
test3: /usr/local/lib/libvtkhdf5-8.1.so.1
test3: /usr/local/lib/libvtkParallelMPI-8.1.so.1
test3: /usr/local/lib/libvtkParallelCore-8.1.so.1
test3: /usr/local/lib/libvtkGUISupportQt-8.1.so.1
test3: /usr/local/lib/libvtkViewsInfovis-8.1.so.1
test3: /usr/local/lib/libvtkChartsCore-8.1.so.1
test3: /usr/local/lib/libvtkViewsCore-8.1.so.1
test3: /usr/local/lib/libvtkInteractionWidgets-8.1.so.1
test3: /usr/local/lib/libvtkFiltersHybrid-8.1.so.1
test3: /usr/local/lib/libvtkRenderingAnnotation-8.1.so.1
test3: /usr/local/lib/libvtkImagingColor-8.1.so.1
test3: /usr/local/lib/libvtkRenderingVolume-8.1.so.1
test3: /usr/local/lib/libvtkIOXML-8.1.so.1
test3: /usr/local/lib/libvtkIOXMLParser-8.1.so.1
test3: /usr/local/lib/libvtkexpat-8.1.so.1
test3: /usr/local/lib/libvtkFiltersImaging-8.1.so.1
test3: /usr/local/lib/libvtkImagingGeneral-8.1.so.1
test3: /usr/local/lib/libvtkImagingSources-8.1.so.1
test3: /usr/local/lib/libvtkRenderingLabel-8.1.so.1
test3: /usr/local/lib/libvtkInfovisLayout-8.1.so.1
test3: /usr/local/lib/libvtkInfovisCore-8.1.so.1
test3: /usr/local/lib/libvtkImagingHybrid-8.1.so.1
test3: /usr/local/lib/libvtkInteractionStyle-8.1.so.1
test3: /usr/local/lib/libvtkFiltersExtraction-8.1.so.1
test3: /usr/local/lib/libvtkFiltersStatistics-8.1.so.1
test3: /usr/local/lib/libvtkImagingFourier-8.1.so.1
test3: /usr/local/lib/libvtkalglib-8.1.so.1
test3: /usr/local/lib/libvtkIOGeometry-8.1.so.1
test3: /usr/local/lib/libvtkIOLegacy-8.1.so.1
test3: /usr/local/lib/libvtkFiltersTexture-8.1.so.1
test3: /usr/local/lib/libvtkIOExport-8.1.so.1
test3: /usr/local/lib/libvtkImagingCore-8.1.so.1
test3: /usr/local/lib/libvtkRenderingContext2D-8.1.so.1
test3: /usr/local/lib/libvtkRenderingFreeType-8.1.so.1
test3: /usr/local/lib/libvtkfreetype-8.1.so.1
test3: /usr/local/lib/libvtkIOImage-8.1.so.1
test3: /usr/local/lib/libvtkDICOMParser-8.1.so.1
test3: /usr/local/lib/libvtkmetaio-8.1.so.1
test3: /usr/local/lib/libvtkpng-8.1.so.1
test3: /usr/local/lib/libvtktiff-8.1.so.1
test3: /usr/local/lib/libvtkjpeg-8.1.so.1
test3: /usr/lib/x86_64-linux-gnu/libm.so
test3: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-8.1.so.1
test3: /usr/local/lib/libvtkRenderingOpenGL2-8.1.so.1
test3: /usr/local/lib/libvtkglew-8.1.so.1
test3: /usr/lib/x86_64-linux-gnu/libSM.so
test3: /usr/lib/x86_64-linux-gnu/libICE.so
test3: /usr/lib/x86_64-linux-gnu/libX11.so
test3: /usr/lib/x86_64-linux-gnu/libXext.so
test3: /usr/lib/x86_64-linux-gnu/libXt.so
test3: /usr/local/lib/libvtkgl2ps-8.1.so.1
test3: /usr/local/lib/libvtklibharu-8.1.so.1
test3: /usr/local/lib/libvtkIOPLY-8.1.so.1
test3: /usr/local/lib/libvtkIOCore-8.1.so.1
test3: /usr/local/lib/libvtkzlib-8.1.so.1
test3: /usr/local/lib/libvtklz4-8.1.so.1
test3: /usr/local/lib/libvtkRenderingLOD-8.1.so.1
test3: /usr/local/lib/libvtkRenderingCore-8.1.so.1
test3: /usr/local/lib/libvtkCommonColor-8.1.so.1
test3: /usr/local/lib/libvtkFiltersGeometry-8.1.so.1
test3: /usr/local/lib/libvtkFiltersModeling-8.1.so.1
test3: /usr/local/lib/libvtkFiltersSources-8.1.so.1
test3: /usr/local/lib/libvtkFiltersGeneral-8.1.so.1
test3: /usr/local/lib/libvtkCommonComputationalGeometry-8.1.so.1
test3: /usr/local/lib/libvtkFiltersCore-8.1.so.1
test3: /usr/local/lib/libvtkCommonExecutionModel-8.1.so.1
test3: /usr/local/lib/libvtkCommonDataModel-8.1.so.1
test3: /usr/local/lib/libvtkCommonMisc-8.1.so.1
test3: /usr/local/lib/libvtkCommonSystem-8.1.so.1
test3: /usr/local/lib/libvtksys-8.1.so.1
test3: /usr/local/lib/libvtkCommonTransforms-8.1.so.1
test3: /usr/local/lib/libvtkCommonMath-8.1.so.1
test3: /usr/local/lib/libvtkCommonCore-8.1.so.1
test3: /usr/local/lib/libopencv_features2d.so.3.4.2
test3: /usr/local/lib/libopencv_highgui.so.3.4.2
test3: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
test3: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
test3: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
test3: /usr/local/lib/libopencv_flann.so.3.4.2
test3: /usr/local/lib/libopencv_videoio.so.3.4.2
test3: /usr/local/lib/libopencv_imgcodecs.so.3.4.2
test3: /usr/local/lib/libopencv_imgproc.so.3.4.2
test3: /usr/local/lib/libopencv_core.so.3.4.2
test3: CMakeFiles/test3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jozef/Documents/QT/3DScan/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test3.dir/build: test3

.PHONY : CMakeFiles/test3.dir/build

CMakeFiles/test3.dir/requires: CMakeFiles/test3.dir/main.cpp.o.requires
CMakeFiles/test3.dir/requires: CMakeFiles/test3.dir/kinect.cpp.o.requires

.PHONY : CMakeFiles/test3.dir/requires

CMakeFiles/test3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test3.dir/clean

CMakeFiles/test3.dir/depend:
	cd /home/jozef/Documents/QT/3DScan && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jozef/Documents/QT/3DScan /home/jozef/Documents/QT/3DScan /home/jozef/Documents/QT/3DScan /home/jozef/Documents/QT/3DScan /home/jozef/Documents/QT/3DScan/CMakeFiles/test3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test3.dir/depend
