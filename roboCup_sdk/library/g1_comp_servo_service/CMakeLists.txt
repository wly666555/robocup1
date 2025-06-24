cmake_minimum_required(VERSION 3.10)
project(g1_comp_servo_service)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  thirdparty/DynamixelSDK/c++/include
)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread) 
link_libraries(libboost_program_options.a yaml-cpp) 

link_directories(lib/)
link_libraries(dxl_x64_cpp fmt)

# test
add_executable(test test.cpp)
add_executable(test2 test2.cpp)
add_executable(test_servo_control test/test_servo_control.cpp)
add_executable(test_read_servo_postion test/test_read_servo_postion.cpp)
add_executable(test_read_yaml test/test_read_yaml.cpp)
add_executable(test_calibration test/test_calibration.cpp)
add_executable(test_joint0_map test/test_joint0_map.cpp)
add_executable(test_joint1_map test/test_joint1_map.cpp)
add_executable(test_servo_control2 test/test_servo_control2.cpp)
add_executable(test_servo_control3 test/test_servo_control3.cpp)
add_executable(test_read_angle test/test_read_angle.cpp)