# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/MKGL/quadro_ws/src/dgz

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/MKGL/quadro_ws/build/dgz

# Include any dependencies generated for this target.
include CMakeFiles/dgz_frame_lib.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/dgz_frame_lib.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/dgz_frame_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dgz_frame_lib.dir/flags.make

CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o: CMakeFiles/dgz_frame_lib.dir/flags.make
CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o: /home/MKGL/quadro_ws/src/dgz/src/dgz_frame.cpp
CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o: CMakeFiles/dgz_frame_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/MKGL/quadro_ws/build/dgz/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o -MF CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o.d -o CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o -c /home/MKGL/quadro_ws/src/dgz/src/dgz_frame.cpp

CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MKGL/quadro_ws/src/dgz/src/dgz_frame.cpp > CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.i

CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MKGL/quadro_ws/src/dgz/src/dgz_frame.cpp -o CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.s

# Object files for target dgz_frame_lib
dgz_frame_lib_OBJECTS = \
"CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o"

# External object files for target dgz_frame_lib
dgz_frame_lib_EXTERNAL_OBJECTS =

libdgz_frame_lib.so: CMakeFiles/dgz_frame_lib.dir/src/dgz_frame.cpp.o
libdgz_frame_lib.so: CMakeFiles/dgz_frame_lib.dir/build.make
libdgz_frame_lib.so: CMakeFiles/dgz_frame_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/MKGL/quadro_ws/build/dgz/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libdgz_frame_lib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dgz_frame_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dgz_frame_lib.dir/build: libdgz_frame_lib.so
.PHONY : CMakeFiles/dgz_frame_lib.dir/build

CMakeFiles/dgz_frame_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dgz_frame_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dgz_frame_lib.dir/clean

CMakeFiles/dgz_frame_lib.dir/depend:
	cd /home/MKGL/quadro_ws/build/dgz && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/MKGL/quadro_ws/src/dgz /home/MKGL/quadro_ws/src/dgz /home/MKGL/quadro_ws/build/dgz /home/MKGL/quadro_ws/build/dgz /home/MKGL/quadro_ws/build/dgz/CMakeFiles/dgz_frame_lib.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/dgz_frame_lib.dir/depend

