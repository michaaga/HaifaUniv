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
CMAKE_SOURCE_DIR = /home/michaaga/openvslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/michaaga/openvslam/build

# Include any dependencies generated for this target.
include example/CMakeFiles/run_euroc_slam.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/run_euroc_slam.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/run_euroc_slam.dir/flags.make

example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o: example/CMakeFiles/run_euroc_slam.dir/flags.make
example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o: ../example/run_euroc_slam.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michaaga/openvslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o"
	cd /home/michaaga/openvslam/build/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o -c /home/michaaga/openvslam/example/run_euroc_slam.cc

example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.i"
	cd /home/michaaga/openvslam/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michaaga/openvslam/example/run_euroc_slam.cc > CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.i

example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.s"
	cd /home/michaaga/openvslam/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michaaga/openvslam/example/run_euroc_slam.cc -o CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.s

example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.requires:

.PHONY : example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.requires

example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.provides: example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.requires
	$(MAKE) -f example/CMakeFiles/run_euroc_slam.dir/build.make example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.provides.build
.PHONY : example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.provides

example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.provides.build: example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o


example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o: example/CMakeFiles/run_euroc_slam.dir/flags.make
example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o: ../example/util/euroc_util.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michaaga/openvslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o"
	cd /home/michaaga/openvslam/build/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o -c /home/michaaga/openvslam/example/util/euroc_util.cc

example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.i"
	cd /home/michaaga/openvslam/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michaaga/openvslam/example/util/euroc_util.cc > CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.i

example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.s"
	cd /home/michaaga/openvslam/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michaaga/openvslam/example/util/euroc_util.cc -o CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.s

example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.requires:

.PHONY : example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.requires

example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.provides: example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.requires
	$(MAKE) -f example/CMakeFiles/run_euroc_slam.dir/build.make example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.provides.build
.PHONY : example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.provides

example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.provides.build: example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o


# Object files for target run_euroc_slam
run_euroc_slam_OBJECTS = \
"CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o" \
"CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o"

# External object files for target run_euroc_slam
run_euroc_slam_EXTERNAL_OBJECTS =

run_euroc_slam: example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o
run_euroc_slam: example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o
run_euroc_slam: example/CMakeFiles/run_euroc_slam.dir/build.make
run_euroc_slam: lib/libsocket_publisher.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libglog.so
run_euroc_slam: lib/libopenvslam.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.5.2
run_euroc_slam: /usr/local/lib/libopencv_calib3d.so.3.4.0
run_euroc_slam: /usr/local/lib/libopencv_features2d.so.3.4.0
run_euroc_slam: /usr/local/lib/libopencv_flann.so.3.4.0
run_euroc_slam: /usr/local/lib/libopencv_highgui.so.3.4.0
run_euroc_slam: /usr/local/lib/libopencv_videoio.so.3.4.0
run_euroc_slam: /usr/local/lib/libg2o_types_sim3.so
run_euroc_slam: /usr/local/lib/libg2o_types_sba.so
run_euroc_slam: /usr/local/lib/libg2o_types_slam3d.so
run_euroc_slam: /usr/local/lib/libg2o_solver_dense.so
run_euroc_slam: /usr/local/lib/libg2o_solver_eigen.so
run_euroc_slam: /usr/local/lib/libg2o_solver_csparse.so
run_euroc_slam: /usr/local/lib/libg2o_core.so
run_euroc_slam: /usr/local/lib/libg2o_stuff.so
run_euroc_slam: /usr/local/lib/libg2o_csparse_extension.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libcxsparse.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libccolamd.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libcamd.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libcolamd.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libamd.so
run_euroc_slam: /usr/lib/liblapack.so
run_euroc_slam: /usr/lib/libf77blas.so
run_euroc_slam: /usr/lib/libatlas.so
run_euroc_slam: /usr/lib/libf77blas.so
run_euroc_slam: /usr/lib/libatlas.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
run_euroc_slam: /usr/lib/x86_64-linux-gnu/librt.so
run_euroc_slam: /usr/local/lib/libdbow2.so
run_euroc_slam: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
run_euroc_slam: /usr/local/lib/libopencv_imgproc.so.3.4.0
run_euroc_slam: /usr/local/lib/libopencv_core.so.3.4.0
run_euroc_slam: /usr/local/lib/libsioclient.so
run_euroc_slam: /usr/local/lib/libprotobuf.so
run_euroc_slam: example/CMakeFiles/run_euroc_slam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/michaaga/openvslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../run_euroc_slam"
	cd /home/michaaga/openvslam/build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_euroc_slam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/run_euroc_slam.dir/build: run_euroc_slam

.PHONY : example/CMakeFiles/run_euroc_slam.dir/build

example/CMakeFiles/run_euroc_slam.dir/requires: example/CMakeFiles/run_euroc_slam.dir/run_euroc_slam.cc.o.requires
example/CMakeFiles/run_euroc_slam.dir/requires: example/CMakeFiles/run_euroc_slam.dir/util/euroc_util.cc.o.requires

.PHONY : example/CMakeFiles/run_euroc_slam.dir/requires

example/CMakeFiles/run_euroc_slam.dir/clean:
	cd /home/michaaga/openvslam/build/example && $(CMAKE_COMMAND) -P CMakeFiles/run_euroc_slam.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/run_euroc_slam.dir/clean

example/CMakeFiles/run_euroc_slam.dir/depend:
	cd /home/michaaga/openvslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/michaaga/openvslam /home/michaaga/openvslam/example /home/michaaga/openvslam/build /home/michaaga/openvslam/build/example /home/michaaga/openvslam/build/example/CMakeFiles/run_euroc_slam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/run_euroc_slam.dir/depend

