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
CMAKE_SOURCE_DIR = /home/unitree/Videos/football_detectcpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/unitree/Videos/football_detectcpp/build

# Include any dependencies generated for this target.
include CMakeFiles/football_detect.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/football_detect.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/football_detect.dir/flags.make

CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o: CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o.depend
CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o: CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o.cmake
CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o: ../src/preprocess.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/unitree/Videos/football_detectcpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building NVCC (Device) object CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o"
	cd /home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/src && /usr/bin/cmake -E make_directory /home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/src/.
	cd /home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/src && /usr/bin/cmake -D verbose:BOOL=$(VERBOSE) -D build_configuration:STRING= -D generated_file:STRING=/home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/src/./football_detect_generated_preprocess.cu.o -D generated_cubin_file:STRING=/home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/src/./football_detect_generated_preprocess.cu.o.cubin.txt -P /home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o.cmake

CMakeFiles/football_detect.dir/main.cpp.o: CMakeFiles/football_detect.dir/flags.make
CMakeFiles/football_detect.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/Videos/football_detectcpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/football_detect.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/football_detect.dir/main.cpp.o -c /home/unitree/Videos/football_detectcpp/main.cpp

CMakeFiles/football_detect.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/football_detect.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/Videos/football_detectcpp/main.cpp > CMakeFiles/football_detect.dir/main.cpp.i

CMakeFiles/football_detect.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/football_detect.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/Videos/football_detectcpp/main.cpp -o CMakeFiles/football_detect.dir/main.cpp.s

CMakeFiles/football_detect.dir/src/YOLO.cpp.o: CMakeFiles/football_detect.dir/flags.make
CMakeFiles/football_detect.dir/src/YOLO.cpp.o: ../src/YOLO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/Videos/football_detectcpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/football_detect.dir/src/YOLO.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/football_detect.dir/src/YOLO.cpp.o -c /home/unitree/Videos/football_detectcpp/src/YOLO.cpp

CMakeFiles/football_detect.dir/src/YOLO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/football_detect.dir/src/YOLO.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/Videos/football_detectcpp/src/YOLO.cpp > CMakeFiles/football_detect.dir/src/YOLO.cpp.i

CMakeFiles/football_detect.dir/src/YOLO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/football_detect.dir/src/YOLO.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/Videos/football_detectcpp/src/YOLO.cpp -o CMakeFiles/football_detect.dir/src/YOLO.cpp.s

CMakeFiles/football_detect.dir/src/DetectionModule.cpp.o: CMakeFiles/football_detect.dir/flags.make
CMakeFiles/football_detect.dir/src/DetectionModule.cpp.o: ../src/DetectionModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/Videos/football_detectcpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/football_detect.dir/src/DetectionModule.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/football_detect.dir/src/DetectionModule.cpp.o -c /home/unitree/Videos/football_detectcpp/src/DetectionModule.cpp

CMakeFiles/football_detect.dir/src/DetectionModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/football_detect.dir/src/DetectionModule.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/Videos/football_detectcpp/src/DetectionModule.cpp > CMakeFiles/football_detect.dir/src/DetectionModule.cpp.i

CMakeFiles/football_detect.dir/src/DetectionModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/football_detect.dir/src/DetectionModule.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/Videos/football_detectcpp/src/DetectionModule.cpp -o CMakeFiles/football_detect.dir/src/DetectionModule.cpp.s

# Object files for target football_detect
football_detect_OBJECTS = \
"CMakeFiles/football_detect.dir/main.cpp.o" \
"CMakeFiles/football_detect.dir/src/YOLO.cpp.o" \
"CMakeFiles/football_detect.dir/src/DetectionModule.cpp.o"

# External object files for target football_detect
football_detect_EXTERNAL_OBJECTS = \
"/home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o"

football_detect: CMakeFiles/football_detect.dir/main.cpp.o
football_detect: CMakeFiles/football_detect.dir/src/YOLO.cpp.o
football_detect: CMakeFiles/football_detect.dir/src/DetectionModule.cpp.o
football_detect: CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o
football_detect: CMakeFiles/football_detect.dir/build.make
football_detect: /usr/local/cuda-11.4/lib64/libcudart_static.a
football_detect: /usr/lib/aarch64-linux-gnu/librt.so
football_detect: /usr/local/lib/libopencv_gapi.so.4.11.0
football_detect: /usr/local/lib/libopencv_highgui.so.4.11.0
football_detect: /usr/local/lib/libopencv_ml.so.4.11.0
football_detect: /usr/local/lib/libopencv_objdetect.so.4.11.0
football_detect: /usr/local/lib/libopencv_photo.so.4.11.0
football_detect: /usr/local/lib/libopencv_stitching.so.4.11.0
football_detect: /usr/local/lib/libopencv_video.so.4.11.0
football_detect: /usr/local/lib/libopencv_videoio.so.4.11.0
football_detect: /usr/local/cuda-11.4/lib64/libcudart_static.a
football_detect: /usr/lib/aarch64-linux-gnu/librt.so
football_detect: /usr/local/lib/librealsense2.so.2.53.1
football_detect: /usr/local/lib/libunitree_sdk2.a
football_detect: /usr/local/lib/libopencv_imgcodecs.so.4.11.0
football_detect: /usr/local/lib/libopencv_dnn.so.4.11.0
football_detect: /usr/local/lib/libopencv_calib3d.so.4.11.0
football_detect: /usr/local/lib/libopencv_features2d.so.4.11.0
football_detect: /usr/local/lib/libopencv_flann.so.4.11.0
football_detect: /usr/local/lib/libopencv_imgproc.so.4.11.0
football_detect: /usr/local/lib/libopencv_core.so.4.11.0
football_detect: /usr/local/lib/libddsc.so
football_detect: /usr/local/lib/libddscxx.so
football_detect: CMakeFiles/football_detect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/Videos/football_detectcpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable football_detect"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/football_detect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/football_detect.dir/build: football_detect

.PHONY : CMakeFiles/football_detect.dir/build

CMakeFiles/football_detect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/football_detect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/football_detect.dir/clean

CMakeFiles/football_detect.dir/depend: CMakeFiles/football_detect.dir/src/football_detect_generated_preprocess.cu.o
	cd /home/unitree/Videos/football_detectcpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/Videos/football_detectcpp /home/unitree/Videos/football_detectcpp /home/unitree/Videos/football_detectcpp/build /home/unitree/Videos/football_detectcpp/build /home/unitree/Videos/football_detectcpp/build/CMakeFiles/football_detect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/football_detect.dir/depend

