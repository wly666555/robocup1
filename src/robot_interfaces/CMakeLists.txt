cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # 用于 Time 类型
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/DetectionResult.msg"
  "msg/DetectionResults.msg"
  "msg/MotorCmds.msg" 
  "msg/MotorCmd.msg"         # 新增 MotorCmd.msg
  "msg/MotorStates.msg"      # 修改后的 MotorStates.msg
  "msg/MotorState.msg"       # 基础消息类型
  "msg/IMUState.msg"
  "msg/LowState.msg"
  "msg/LocationResult.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs  # 依赖 builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()