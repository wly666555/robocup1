cmake_minimum_required(VERSION 3.10)
project(g1_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_api REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)



add_executable(g1_brain 
    src/main.cpp 
    src/brain.cpp 
    src/locator.cpp 
    src/brain_tree.cpp
    src/brain_config.cpp
    src/brain_data.cpp
    src/client.cpp
    src/pose.cpp
)

target_include_directories(g1_brain
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${behaviortree_cpp_v3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../roboCup_sdk/include
    ${CMAKE_SOURCE_DIR}/../roboCup_sdk/include/common
    ${CMAKE_SOURCE_DIR}/../roboCup_sdk/include/control
    ${CMAKE_SOURCE_DIR}/../roboCup_sdk/library/unitree_sdk2/include
    ${CMAKE_SOURCE_DIR}/../roboCup_sdk/library/unitree_sdk2/include/unitree
)

link_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/lib/x86_64)
link_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/lib/aarch64)

ament_target_dependencies(g1_brain
  rclcpp
  robot_interfaces
  nav_msgs
  geometry_msgs
  unitree_sdk2
  Eigen3
  unitree_api
  tf2
  tf2_ros
  sensor_msgs
  OpenCV
  yaml-cpp
  unitree_go
)



target_link_libraries(g1_brain
  /opt/ros/foxy/lib/libbehaviortree_cpp_v3.so
  unitree_sdk2
  rt
  pthread
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS
  g1_brain
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
