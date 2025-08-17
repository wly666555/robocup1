cmake_minimum_required(VERSION 3.12)
project(football_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)  # 依赖消息包
# OpenCV
find_package(OpenCV REQUIRED)
# RealSense2
find_package(realsense2 REQUIRED)
# CUDA
find_package(CUDA REQUIRED)

# TensorRT
find_library(TENSORRT_LIB nvinfer HINTS /usr/lib/x86_64-linux-gnu/ /usr/lib/)
find_library(NVONNXPARSER_LIB nvonnxparser HINTS /usr/lib/x86_64-linux-gnu/ /usr/lib/)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS /usr/lib/x86_64-linux-gnu/ /usr/lib/)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17")
cuda_add_library(preprocess_cuda src/preprocess.cu)

set(SRC_FILES
  src/main.cpp
  src/YOLO.cpp
  src/cameracontroller.cpp
  src/safe_angle.cpp
  src/version_node.cpp
)

add_executable(football_detect_node ${SRC_FILES})
target_link_libraries(football_detect_node
  preprocess_cuda
  ${OpenCV_LIBS}
  ${TENSORRT_LIB}
  ${NVONNXPARSER_LIB}
  ${NVINFER_PLUGIN_LIB}
  cuda
  realsense2::realsense2
)

ament_target_dependencies(football_detect_node
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  realsense2
  robot_interfaces  # 依赖消息包
)

install(TARGETS preprocess_cuda DESTINATION lib)
install(TARGETS football_detect_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)
install(DIRECTORY weight/ DESTINATION share/${PROJECT_NAME}/weight)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
