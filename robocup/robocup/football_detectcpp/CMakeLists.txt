cmake_minimum_required(VERSION 3.12)
project(football_detect)

find_package(CUDA REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
add_definitions(-DAPI_EXPORTS)

# Include CUDA directories
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(src/)

# Add source files
set(SOURCES
    main.cpp
    src/YOLO.cpp
    src/DetectionModule.cpp
)

# Add CUDA source files
set(CUDA_SOURCES src/preprocess.cu)

# Add headers
set(HEADERS
    src/YOLO.h
    src/macros.h
    src/logging.h
    src/cuda_utils.h
    src/preprocess.h
    src/common.h
    src/DetectionModule.hpp
)






# Find and include OpenCV
#set(OpenCV_DIR "your OpenCV build directory path")
find_package(OpenCV REQUIRED)
include_directories(
${OpenCV_INCLUDE_DIRS}
)


find_package(realsense2 REQUIRED)

find_package(unitree_sdk2 REQUIRED)

# Include TensorRT
# Set TensorRT path if not set in environment variables
# set(TENSORRT_DIR "/home/unitree/Toots/TensorRT-10.9.0.34/")
# include_directories(${TENSORRT_DIR}/include)
# link_directories(${TENSORRT_DIR}/lib)

set(TENSORRT_LIBS nvinfer nvinfer_plugin  nvonnxparser)

# Create an executable
cuda_add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${CUDA_SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIBS}
    realsense2::realsense2
    unitree_sdk2
)

cuda_add_executable(sub sub.cpp src/DetectionModule.cpp src/DetectionModule.hpp )

target_link_libraries(sub
    unitree_sdk2
)