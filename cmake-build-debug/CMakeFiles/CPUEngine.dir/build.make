# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/forbes/CLionProjects/CPUEngine

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/forbes/CLionProjects/CPUEngine/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/CPUEngine.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CPUEngine.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CPUEngine.dir/flags.make

CMakeFiles/CPUEngine.dir/main.cpp.o: CMakeFiles/CPUEngine.dir/flags.make
CMakeFiles/CPUEngine.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/forbes/CLionProjects/CPUEngine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CPUEngine.dir/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CPUEngine.dir/main.cpp.o -c /Users/forbes/CLionProjects/CPUEngine/main.cpp

CMakeFiles/CPUEngine.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CPUEngine.dir/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/forbes/CLionProjects/CPUEngine/main.cpp > CMakeFiles/CPUEngine.dir/main.cpp.i

CMakeFiles/CPUEngine.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CPUEngine.dir/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/forbes/CLionProjects/CPUEngine/main.cpp -o CMakeFiles/CPUEngine.dir/main.cpp.s

# Object files for target CPUEngine
CPUEngine_OBJECTS = \
"CMakeFiles/CPUEngine.dir/main.cpp.o"

# External object files for target CPUEngine
CPUEngine_EXTERNAL_OBJECTS =

CPUEngine: CMakeFiles/CPUEngine.dir/main.cpp.o
CPUEngine: CMakeFiles/CPUEngine.dir/build.make
CPUEngine: CMakeFiles/CPUEngine.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/forbes/CLionProjects/CPUEngine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable CPUEngine"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CPUEngine.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CPUEngine.dir/build: CPUEngine

.PHONY : CMakeFiles/CPUEngine.dir/build

CMakeFiles/CPUEngine.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CPUEngine.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CPUEngine.dir/clean

CMakeFiles/CPUEngine.dir/depend:
	cd /Users/forbes/CLionProjects/CPUEngine/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/forbes/CLionProjects/CPUEngine /Users/forbes/CLionProjects/CPUEngine /Users/forbes/CLionProjects/CPUEngine/cmake-build-debug /Users/forbes/CLionProjects/CPUEngine/cmake-build-debug /Users/forbes/CLionProjects/CPUEngine/cmake-build-debug/CMakeFiles/CPUEngine.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CPUEngine.dir/depend

