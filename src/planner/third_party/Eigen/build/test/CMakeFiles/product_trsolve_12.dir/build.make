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
include test/CMakeFiles/product_trsolve_12.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/product_trsolve_12.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/product_trsolve_12.dir/flags.make

test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o: test/CMakeFiles/product_trsolve_12.dir/flags.make
test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o: ../test/product_trsolve.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fyk/eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o"
	cd /home/fyk/eigen/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o -c /home/fyk/eigen/test/product_trsolve.cpp

test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.i"
	cd /home/fyk/eigen/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fyk/eigen/test/product_trsolve.cpp > CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.i

test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.s"
	cd /home/fyk/eigen/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fyk/eigen/test/product_trsolve.cpp -o CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.s

test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.requires:

.PHONY : test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.requires

test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.provides: test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/product_trsolve_12.dir/build.make test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.provides.build
.PHONY : test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.provides

test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.provides.build: test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o


# Object files for target product_trsolve_12
product_trsolve_12_OBJECTS = \
"CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o"

# External object files for target product_trsolve_12
product_trsolve_12_EXTERNAL_OBJECTS =

test/product_trsolve_12: test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o
test/product_trsolve_12: test/CMakeFiles/product_trsolve_12.dir/build.make
test/product_trsolve_12: test/CMakeFiles/product_trsolve_12.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fyk/eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable product_trsolve_12"
	cd /home/fyk/eigen/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/product_trsolve_12.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/product_trsolve_12.dir/build: test/product_trsolve_12

.PHONY : test/CMakeFiles/product_trsolve_12.dir/build

test/CMakeFiles/product_trsolve_12.dir/requires: test/CMakeFiles/product_trsolve_12.dir/product_trsolve.cpp.o.requires

.PHONY : test/CMakeFiles/product_trsolve_12.dir/requires

test/CMakeFiles/product_trsolve_12.dir/clean:
	cd /home/fyk/eigen/build/test && $(CMAKE_COMMAND) -P CMakeFiles/product_trsolve_12.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/product_trsolve_12.dir/clean

test/CMakeFiles/product_trsolve_12.dir/depend:
	cd /home/fyk/eigen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fyk/eigen /home/fyk/eigen/test /home/fyk/eigen/build /home/fyk/eigen/build/test /home/fyk/eigen/build/test/CMakeFiles/product_trsolve_12.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/product_trsolve_12.dir/depend

