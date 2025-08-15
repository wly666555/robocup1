cmake_minimum_required(VERSION 3.12)
project(robocup_locator)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)                                                                                                                                                                
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)

find_package(unitree_sdk2 REQUIRED)

include_directories("include")

file(GLOB SOURCES "src/*.cpp")

add_executable(test_location ${SOURCES} test_location.cpp)
target_link_libraries(test_location 
                    unitree_sdk2
                    Eigen3::Eigen 
                    ${OpenCV_LIBS}
                    yaml-cpp
                    )

add_executable(test_subscribe_location src/LocationModule.cpp test_subscribe_location.cpp)
target_link_libraries(test_subscribe_location unitree_sdk2 )
