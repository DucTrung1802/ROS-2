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
CMAKE_SOURCE_DIR = "/home/ductrung1802/GIT/ROS 2/python_call_cpp"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/ductrung1802/GIT/ROS 2/python_call_cpp"

# Include any dependencies generated for this target.
include CMakeFiles/fuzzy_logic.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fuzzy_logic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fuzzy_logic.dir/flags.make

CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.o: CMakeFiles/fuzzy_logic.dir/flags.make
CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.o: fuzzy_logic_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/ductrung1802/GIT/ROS 2/python_call_cpp/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.o -c "/home/ductrung1802/GIT/ROS 2/python_call_cpp/fuzzy_logic_test.cpp"

CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ductrung1802/GIT/ROS 2/python_call_cpp/fuzzy_logic_test.cpp" > CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.i

CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ductrung1802/GIT/ROS 2/python_call_cpp/fuzzy_logic_test.cpp" -o CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.s

# Object files for target fuzzy_logic
fuzzy_logic_OBJECTS = \
"CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.o"

# External object files for target fuzzy_logic
fuzzy_logic_EXTERNAL_OBJECTS =

fuzzy_logic.cpython-38-x86_64-linux-gnu.so: CMakeFiles/fuzzy_logic.dir/fuzzy_logic_test.cpp.o
fuzzy_logic.cpython-38-x86_64-linux-gnu.so: CMakeFiles/fuzzy_logic.dir/build.make
fuzzy_logic.cpython-38-x86_64-linux-gnu.so: CMakeFiles/fuzzy_logic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/ductrung1802/GIT/ROS 2/python_call_cpp/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module fuzzy_logic.cpython-38-x86_64-linux-gnu.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fuzzy_logic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fuzzy_logic.dir/build: fuzzy_logic.cpython-38-x86_64-linux-gnu.so

.PHONY : CMakeFiles/fuzzy_logic.dir/build

CMakeFiles/fuzzy_logic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fuzzy_logic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fuzzy_logic.dir/clean

CMakeFiles/fuzzy_logic.dir/depend:
	cd "/home/ductrung1802/GIT/ROS 2/python_call_cpp" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ductrung1802/GIT/ROS 2/python_call_cpp" "/home/ductrung1802/GIT/ROS 2/python_call_cpp" "/home/ductrung1802/GIT/ROS 2/python_call_cpp" "/home/ductrung1802/GIT/ROS 2/python_call_cpp" "/home/ductrung1802/GIT/ROS 2/python_call_cpp/CMakeFiles/fuzzy_logic.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/fuzzy_logic.dir/depend

