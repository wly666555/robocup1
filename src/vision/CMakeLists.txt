cmake_minimum_required(VERSION 3.10)
project(vision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4.5 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(PCL REQUIRED)
# find ros2 package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vision_interface REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src)
# add_subdirectory(test)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
