# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/leo/git/mapper_px4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/git/mapper_px4/build

# Include any dependencies generated for this target.
include src/CMakeFiles/onboard.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/onboard.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/onboard.dir/flags.make

src/CMakeFiles/onboard.dir/onboard.cpp.o: src/CMakeFiles/onboard.dir/flags.make
src/CMakeFiles/onboard.dir/onboard.cpp.o: ../src/onboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/git/mapper_px4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/onboard.dir/onboard.cpp.o"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/onboard.cpp.o -c /home/leo/git/mapper_px4/src/onboard.cpp

src/CMakeFiles/onboard.dir/onboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/onboard.cpp.i"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/git/mapper_px4/src/onboard.cpp > CMakeFiles/onboard.dir/onboard.cpp.i

src/CMakeFiles/onboard.dir/onboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/onboard.cpp.s"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/git/mapper_px4/src/onboard.cpp -o CMakeFiles/onboard.dir/onboard.cpp.s

src/CMakeFiles/onboard.dir/onboard.cpp.o.requires:

.PHONY : src/CMakeFiles/onboard.dir/onboard.cpp.o.requires

src/CMakeFiles/onboard.dir/onboard.cpp.o.provides: src/CMakeFiles/onboard.dir/onboard.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/onboard.dir/build.make src/CMakeFiles/onboard.dir/onboard.cpp.o.provides.build
.PHONY : src/CMakeFiles/onboard.dir/onboard.cpp.o.provides

src/CMakeFiles/onboard.dir/onboard.cpp.o.provides.build: src/CMakeFiles/onboard.dir/onboard.cpp.o


src/CMakeFiles/onboard.dir/connection.cpp.o: src/CMakeFiles/onboard.dir/flags.make
src/CMakeFiles/onboard.dir/connection.cpp.o: ../src/connection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/git/mapper_px4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/onboard.dir/connection.cpp.o"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/connection.cpp.o -c /home/leo/git/mapper_px4/src/connection.cpp

src/CMakeFiles/onboard.dir/connection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/connection.cpp.i"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/git/mapper_px4/src/connection.cpp > CMakeFiles/onboard.dir/connection.cpp.i

src/CMakeFiles/onboard.dir/connection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/connection.cpp.s"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/git/mapper_px4/src/connection.cpp -o CMakeFiles/onboard.dir/connection.cpp.s

src/CMakeFiles/onboard.dir/connection.cpp.o.requires:

.PHONY : src/CMakeFiles/onboard.dir/connection.cpp.o.requires

src/CMakeFiles/onboard.dir/connection.cpp.o.provides: src/CMakeFiles/onboard.dir/connection.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/onboard.dir/build.make src/CMakeFiles/onboard.dir/connection.cpp.o.provides.build
.PHONY : src/CMakeFiles/onboard.dir/connection.cpp.o.provides

src/CMakeFiles/onboard.dir/connection.cpp.o.provides.build: src/CMakeFiles/onboard.dir/connection.cpp.o


src/CMakeFiles/onboard.dir/glad.c.o: src/CMakeFiles/onboard.dir/flags.make
src/CMakeFiles/onboard.dir/glad.c.o: ../src/glad.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/git/mapper_px4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object src/CMakeFiles/onboard.dir/glad.c.o"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/onboard.dir/glad.c.o   -c /home/leo/git/mapper_px4/src/glad.c

src/CMakeFiles/onboard.dir/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/onboard.dir/glad.c.i"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leo/git/mapper_px4/src/glad.c > CMakeFiles/onboard.dir/glad.c.i

src/CMakeFiles/onboard.dir/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/onboard.dir/glad.c.s"
	cd /home/leo/git/mapper_px4/build/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leo/git/mapper_px4/src/glad.c -o CMakeFiles/onboard.dir/glad.c.s

src/CMakeFiles/onboard.dir/glad.c.o.requires:

.PHONY : src/CMakeFiles/onboard.dir/glad.c.o.requires

src/CMakeFiles/onboard.dir/glad.c.o.provides: src/CMakeFiles/onboard.dir/glad.c.o.requires
	$(MAKE) -f src/CMakeFiles/onboard.dir/build.make src/CMakeFiles/onboard.dir/glad.c.o.provides.build
.PHONY : src/CMakeFiles/onboard.dir/glad.c.o.provides

src/CMakeFiles/onboard.dir/glad.c.o.provides.build: src/CMakeFiles/onboard.dir/glad.c.o


# Object files for target onboard
onboard_OBJECTS = \
"CMakeFiles/onboard.dir/onboard.cpp.o" \
"CMakeFiles/onboard.dir/connection.cpp.o" \
"CMakeFiles/onboard.dir/glad.c.o"

# External object files for target onboard
onboard_EXTERNAL_OBJECTS =

src/onboard: src/CMakeFiles/onboard.dir/onboard.cpp.o
src/onboard: src/CMakeFiles/onboard.dir/connection.cpp.o
src/onboard: src/CMakeFiles/onboard.dir/glad.c.o
src/onboard: src/CMakeFiles/onboard.dir/build.make
src/onboard: /usr/local/lib/libopencv_dnn.so.3.4.5
src/onboard: /usr/local/lib/libopencv_ml.so.3.4.5
src/onboard: /usr/local/lib/libopencv_objdetect.so.3.4.5
src/onboard: /usr/local/lib/libopencv_shape.so.3.4.5
src/onboard: /usr/local/lib/libopencv_stitching.so.3.4.5
src/onboard: /usr/local/lib/libopencv_superres.so.3.4.5
src/onboard: /usr/local/lib/libopencv_videostab.so.3.4.5
src/onboard: /usr/local/lib/librealsense2.so
src/onboard: /usr/lib/x86_64-linux-gnu/libGL.so
src/onboard: /usr/lib/x86_64-linux-gnu/libGLU.so
src/onboard: /usr/lib/x86_64-linux-gnu/libcurses.so
src/onboard: /usr/lib/x86_64-linux-gnu/libform.so
src/onboard: /usr/lib/x86_64-linux-gnu/libcurses.so
src/onboard: /usr/lib/x86_64-linux-gnu/libform.so
src/onboard: /usr/local/lib/libopencv_calib3d.so.3.4.5
src/onboard: /usr/local/lib/libopencv_features2d.so.3.4.5
src/onboard: /usr/local/lib/libopencv_flann.so.3.4.5
src/onboard: /usr/local/lib/libopencv_highgui.so.3.4.5
src/onboard: /usr/local/lib/libopencv_photo.so.3.4.5
src/onboard: /usr/local/lib/libopencv_video.so.3.4.5
src/onboard: /usr/local/lib/libopencv_videoio.so.3.4.5
src/onboard: /usr/local/lib/libopencv_imgcodecs.so.3.4.5
src/onboard: /usr/local/lib/libopencv_imgproc.so.3.4.5
src/onboard: /usr/local/lib/libopencv_core.so.3.4.5
src/onboard: src/CMakeFiles/onboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/git/mapper_px4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable onboard"
	cd /home/leo/git/mapper_px4/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/onboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/onboard.dir/build: src/onboard

.PHONY : src/CMakeFiles/onboard.dir/build

src/CMakeFiles/onboard.dir/requires: src/CMakeFiles/onboard.dir/onboard.cpp.o.requires
src/CMakeFiles/onboard.dir/requires: src/CMakeFiles/onboard.dir/connection.cpp.o.requires
src/CMakeFiles/onboard.dir/requires: src/CMakeFiles/onboard.dir/glad.c.o.requires

.PHONY : src/CMakeFiles/onboard.dir/requires

src/CMakeFiles/onboard.dir/clean:
	cd /home/leo/git/mapper_px4/build/src && $(CMAKE_COMMAND) -P CMakeFiles/onboard.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/onboard.dir/clean

src/CMakeFiles/onboard.dir/depend:
	cd /home/leo/git/mapper_px4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/git/mapper_px4 /home/leo/git/mapper_px4/src /home/leo/git/mapper_px4/build /home/leo/git/mapper_px4/build/src /home/leo/git/mapper_px4/build/src/CMakeFiles/onboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/onboard.dir/depend

