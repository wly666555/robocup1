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
CMAKE_SOURCE_DIR = /home/unitree/robocup1-master/roboCup_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/unitree/robocup1-master/roboCup_sdk/build

# Include any dependencies generated for this target.
include test/CMakeFiles/test_findAndTrackBall.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/test_findAndTrackBall.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/test_findAndTrackBall.dir/flags.make

test/CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.o: test/CMakeFiles/test_findAndTrackBall.dir/flags.make
test/CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.o: ../test/findAndTrackBall.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/robocup1-master/roboCup_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.o"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.o -c /home/unitree/robocup1-master/roboCup_sdk/test/findAndTrackBall.cpp

test/CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.i"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/robocup1-master/roboCup_sdk/test/findAndTrackBall.cpp > CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.i

test/CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.s"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/robocup1-master/roboCup_sdk/test/findAndTrackBall.cpp -o CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.s

test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.o: test/CMakeFiles/test_findAndTrackBall.dir/flags.make
test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.o: ../src/common/DetectionModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/robocup1-master/roboCup_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.o"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.o -c /home/unitree/robocup1-master/roboCup_sdk/src/common/DetectionModule.cpp

test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.i"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/robocup1-master/roboCup_sdk/src/common/DetectionModule.cpp > CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.i

test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.s"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/robocup1-master/roboCup_sdk/src/common/DetectionModule.cpp -o CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.s

test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.o: test/CMakeFiles/test_findAndTrackBall.dir/flags.make
test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.o: ../src/common/LocationModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/robocup1-master/roboCup_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.o"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.o -c /home/unitree/robocup1-master/roboCup_sdk/src/common/LocationModule.cpp

test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.i"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/robocup1-master/roboCup_sdk/src/common/LocationModule.cpp > CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.i

test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.s"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/robocup1-master/roboCup_sdk/src/common/LocationModule.cpp -o CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.s

test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.o: test/CMakeFiles/test_findAndTrackBall.dir/flags.make
test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.o: ../src/control/interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/robocup1-master/roboCup_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.o"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.o -c /home/unitree/robocup1-master/roboCup_sdk/src/control/interface.cpp

test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.i"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/robocup1-master/roboCup_sdk/src/control/interface.cpp > CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.i

test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.s"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/robocup1-master/roboCup_sdk/src/control/interface.cpp -o CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.s

test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.o: test/CMakeFiles/test_findAndTrackBall.dir/flags.make
test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.o: ../src/control/node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/robocup1-master/roboCup_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.o"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.o -c /home/unitree/robocup1-master/roboCup_sdk/src/control/node.cpp

test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.i"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/robocup1-master/roboCup_sdk/src/control/node.cpp > CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.i

test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.s"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/robocup1-master/roboCup_sdk/src/control/node.cpp -o CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.s

# Object files for target test_findAndTrackBall
test_findAndTrackBall_OBJECTS = \
"CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.o" \
"CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.o" \
"CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.o" \
"CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.o" \
"CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.o"

# External object files for target test_findAndTrackBall
test_findAndTrackBall_EXTERNAL_OBJECTS =

test/test_findAndTrackBall: test/CMakeFiles/test_findAndTrackBall.dir/findAndTrackBall.cpp.o
test/test_findAndTrackBall: test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/DetectionModule.cpp.o
test/test_findAndTrackBall: test/CMakeFiles/test_findAndTrackBall.dir/__/src/common/LocationModule.cpp.o
test/test_findAndTrackBall: test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/interface.cpp.o
test/test_findAndTrackBall: test/CMakeFiles/test_findAndTrackBall.dir/__/src/control/node.cpp.o
test/test_findAndTrackBall: test/CMakeFiles/test_findAndTrackBall.dir/build.make
test/test_findAndTrackBall: /usr/local/lib/libunitree_sdk2.a
test/test_findAndTrackBall: /usr/local/lib/libddsc.so
test/test_findAndTrackBall: /usr/local/lib/libddscxx.so
test/test_findAndTrackBall: test/CMakeFiles/test_findAndTrackBall.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/robocup1-master/roboCup_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable test_findAndTrackBall"
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_findAndTrackBall.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/test_findAndTrackBall.dir/build: test/test_findAndTrackBall

.PHONY : test/CMakeFiles/test_findAndTrackBall.dir/build

test/CMakeFiles/test_findAndTrackBall.dir/clean:
	cd /home/unitree/robocup1-master/roboCup_sdk/build/test && $(CMAKE_COMMAND) -P CMakeFiles/test_findAndTrackBall.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/test_findAndTrackBall.dir/clean

test/CMakeFiles/test_findAndTrackBall.dir/depend:
	cd /home/unitree/robocup1-master/roboCup_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/robocup1-master/roboCup_sdk /home/unitree/robocup1-master/roboCup_sdk/test /home/unitree/robocup1-master/roboCup_sdk/build /home/unitree/robocup1-master/roboCup_sdk/build/test /home/unitree/robocup1-master/roboCup_sdk/build/test/CMakeFiles/test_findAndTrackBall.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/test_findAndTrackBall.dir/depend

