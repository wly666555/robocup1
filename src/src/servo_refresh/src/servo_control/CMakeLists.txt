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
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# 查找PkgConfig
find_package(PkgConfig REQUIRED)

# 查找yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# 查找spdlog
find_package(spdlog REQUIRED)

# 设置DynamixelSDK源码路径
set(DYNAMIXEL_SDK_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/DynamixelSDK/c++)


# 包含目录
include_directories(
    include
    ${YAML_CPP_INCLUDE_DIRS}
    ${spdlog_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${robot_interfaces_INCLUDE_DIRS}
    ${DYNAMIXEL_SDK_SOURCE_DIR}/include
    ${DYNAMIXEL_SDK_SOURCE_DIR}/include/dynamixel_sdk
)

# DynamixelSDK源文件
set(DYNAMIXEL_SDK_SOURCES
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/port_handler.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/port_handler_linux.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/packet_handler.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/protocol1_packet_handler.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/protocol2_packet_handler.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/group_handler.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/group_sync_read.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/group_sync_write.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/group_bulk_read.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/group_bulk_write.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/group_fast_sync_read.cpp
    ${DYNAMIXEL_SDK_SOURCE_DIR}/src/dynamixel_sdk/group_fast_bulk_read.cpp
)

# 创建可执行文件
add_executable(servo_control_node
    src/main.cpp
    src/servo_control.cpp
    src/dxl_controller.cpp
    src/utilities.cpp
    ${DYNAMIXEL_SDK_SOURCES}
)

# 链接库
target_link_libraries(servo_control_node
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
    eigen
    yaml-cpp
    spdlog
)

ament_package() 