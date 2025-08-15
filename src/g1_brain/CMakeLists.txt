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
find_package(unitree_go REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_api REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)


# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Kick.msg"
#   DEPENDENCIES std_msgs # Add packages that above messages depend on
# )
# ament_export_dependencies(rosidl_default_runtime)

file(GLOB SOURCE_FILES src/*.cpp)
add_executable(g1_brain_node ${SOURCE_FILES})

# Find the typesupport target
# rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Link the typesupport target to your project
# target_link_libraries(g1_brain_node ${typesupport_target})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wno-unused-parameter -Wno-sign-compare -Wno-missing-field-initializers -Wno-unused-variable")


target_include_directories(g1_brain_node
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${behaviortree_cpp_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    /usr/local/include/ddscxx
    /usr/local/include/iceoryx/v2.0.2
)


ament_target_dependencies(g1_brain_node
  rclcpp
  robot_interfaces
  unitree_go
  geometry_msgs
  Eigen3
  unitree_api
  tf2
  tf2_ros
  sensor_msgs
  OpenCV
  yaml-cpp
  behaviortree_cpp
  unitree_sdk2
)



target_link_libraries(g1_brain_node
  rt
  pthread
  ${OpenCV_LIBS}
  yaml-cpp
  unitree_sdk2
)

install(TARGETS
  g1_brain_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME})


ament_package()
