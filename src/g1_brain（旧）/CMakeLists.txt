cmake_minimum_required(VERSION 3.10)
project(g1_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 包含头文件目录
include_directories(include)

# 包含宇树SDK头文件目录
include_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/include)
include_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/include/common)
include_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/include/control)
include_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/library/unitree_sdk2/include)
include_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/library/unitree_sdk2/include/unitree)

# 添加roboCup_sdk库路径
link_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/lib/x86_64)
link_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/lib/aarch64)

# 创建可执行文件
add_executable(g1_brain 
    src/main.cpp 
    src/brain.cpp 
    src/locator.cpp 
    src/brain_tree.cpp
)

# 链接依赖库
ament_target_dependencies(g1_brain
  rclcpp
  robot_interfaces
  nav_msgs
  geometry_msgs
)

# 链接roboCup_sdk库
target_link_libraries(g1_brain
  behaviortree_cpp
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
)

# 安装可执行文件
install(TARGETS
  g1_brain_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 