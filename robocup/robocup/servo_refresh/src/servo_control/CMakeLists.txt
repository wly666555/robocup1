cmake_minimum_required(VERSION 3.8)
project(servo_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# 查找PkgConfig
find_package(PkgConfig REQUIRED)

# 查找yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# 查找spdlog
find_package(spdlog REQUIRED)

# 包含目录
include_directories(
    include
    ${YAML_CPP_INCLUDE_DIRS}
    ${spdlog_INCLUDE_DIRS}
    ${dynamixel_sdk_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${robot_interfaces_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(servo_control_node
    src/main.cpp
    src/servo_control.cpp
    src/dxl_controller.cpp
    src/utilities.cpp
)

# 链接库
target_link_libraries(servo_control_node
    ${dynamixel_sdk_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${spdlog_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    Eigen3::Eigen
)

# 使用ament_target_dependencies链接ROS2依赖项
ament_target_dependencies(servo_control_node
    rclcpp
    std_msgs
    robot_interfaces
)

# 安装目标
install(TARGETS servo_control_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
    rclcpp
    std_msgs
    robot_interfaces
    dynamixel_sdk
    eigen
    yaml-cpp
    spdlog
)

ament_package() 