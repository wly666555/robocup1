cmake_minimum_required(VERSION 3.8)
project(roboCup_sdk_test)

include_directories(
    ../include
    /usr/local/include/iceoryx/v2.0.2
    ../library/BehaviorTree.CPP/include
    ../library/g1_comp_servo_service/include
)

message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

link_directories(/usr/local/lib) 

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/${CMAKE_SYSTEM_PROCESSOR})

file(GLOB_RECURSE SRC_LIST
    "../src/*/*.cpp"
)

add_executable(test_camToPosition node/camToPosition.cpp ${SRC_LIST})
target_link_libraries(test_camToPosition unitree_sdk2 ddsc ddscxx rt pthread behaviortree_cpp)

add_executable(test_camTrackBall node/camTrackBall.cpp ${SRC_LIST})
target_link_libraries(test_camTrackBall unitree_sdk2 ddsc ddscxx rt pthread behaviortree_cpp)

add_executable(test_camFindBall node/camFindBall.cpp ${SRC_LIST})
target_link_libraries(test_camFindBall unitree_sdk2 ddsc ddscxx rt pthread behaviortree_cpp)

add_executable(test_findAndTrackBall findAndTrackBall.cpp ${SRC_LIST})
target_link_libraries(test_findAndTrackBall unitree_sdk2 ddsc ddscxx rt pthread behaviortree_cpp)

add_executable(halfFieldKickExample halfFieldKickExample.cpp ${SRC_LIST})
target_link_libraries(halfFieldKickExample unitree_sdk2 ddsc ddscxx rt pthread behaviortree_cpp)

add_executable(Back_test Back_test.cpp ${SRC_LIST})
target_link_libraries(Back_test unitree_sdk2 ddsc ddscxx rt pthread behaviortree_cpp)



