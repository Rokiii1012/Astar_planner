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
CMAKE_SOURCE_DIR = /home/fyk/eigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fyk/eigen/build

# Include any dependencies generated for this target.
include test/CMakeFiles/visitor_3.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/visitor_3.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/visitor_3.dir/flags.make

test/CMakeFiles/visitor_3.dir/visitor.cpp.o: test/CMakeFiles/visitor_3.dir/flags.make
test/CMakeFiles/visitor_3.dir/visitor.cpp.o: ../test/visitor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fyk/eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/visitor_3.dir/visitor.cpp.o"
	cd /home/fyk/eigen/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/visitor_3.dir/visitor.cpp.o -c /home/fyk/eigen/test/visitor.cpp

test/CMakeFiles/visitor_3.dir/visitor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/visitor_3.dir/visitor.cpp.i"
	cd /home/fyk/eigen/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fyk/eigen/test/visitor.cpp > CMakeFiles/visitor_3.dir/visitor.cpp.i

test/CMakeFiles/visitor_3.dir/visitor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/visitor_3.dir/visitor.cpp.s"
	cd /home/fyk/eigen/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fyk/eigen/test/visitor.cpp -o CMakeFiles/visitor_3.dir/visitor.cpp.s

test/CMakeFiles/visitor_3.dir/visitor.cpp.o.requires:

.PHONY : test/CMakeFiles/visitor_3.dir/visitor.cpp.o.requires

test/CMakeFiles/visitor_3.dir/visitor.cpp.o.provides: test/CMakeFiles/visitor_3.dir/visitor.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/visitor_3.dir/build.make test/CMakeFiles/visitor_3.dir/visitor.cpp.o.provides.build
.PHONY : test/CMakeFiles/visitor_3.dir/visitor.cpp.o.provides

test/CMakeFiles/visitor_3.dir/visitor.cpp.o.provides.build: test/CMakeFiles/visitor_3.dir/visitor.cpp.o


# Object files for target visitor_3
visitor_3_OBJECTS = \
"CMakeFiles/visitor_3.dir/visitor.cpp.o"

# External object files for target visitor_3
visitor_3_EXTERNAL_OBJECTS =

test/visitor_3: test/CMakeFiles/visitor_3.dir/visitor.cpp.o
test/visitor_3: test/CMakeFiles/visitor_3.dir/build.make
test/visitor_3: test/CMakeFiles/visitor_3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fyk/eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable visitor_3"
	cd /home/fyk/eigen/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/visitor_3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/visitor_3.dir/build: test/visitor_3

.PHONY : test/CMakeFiles/visitor_3.dir/build

test/CMakeFiles/visitor_3.dir/requires: test/CMakeFiles/visitor_3.dir/visitor.cpp.o.requires

.PHONY : test/CMakeFiles/visitor_3.dir/requires

test/CMakeFiles/visitor_3.dir/clean:
	cd /home/fyk/eigen/build/test && $(CMAKE_COMMAND) -P CMakeFiles/visitor_3.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/visitor_3.dir/clean

test/CMakeFiles/visitor_3.dir/depend:
	cd /home/fyk/eigen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fyk/eigen /home/fyk/eigen/test /home/fyk/eigen/build /home/fyk/eigen/build/test /home/fyk/eigen/build/test/CMakeFiles/visitor_3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/visitor_3.dir/depend

