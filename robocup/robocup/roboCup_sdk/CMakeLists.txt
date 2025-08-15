cmake_minimum_required(VERSION 3.8)
project(roboCup_sdk)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(test)

find_package(unitree_sdk2)  # todo

include_directories(
    include
    /usr/local/include/ddscxx
    /usr/local/include/iceoryx/v2.0.2
    library/BehaviorTree.CPP/include
    library/BehaviorTree.CPP/sample_nodes
    library/g1_comp_servo_service/include
)

# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

# file(GLOB_RECURSE SRC_LIST
#     "src/*/*.cpp"
#     "library/BehaviorTree.CPP/sample_nodes/*.cpp"
# )

# add_executable(junior_ctrl src/main.cpp ${SRC_LIST})
# target_compile_definitions(junior_ctrl PRIVATE "MANUAL_STATIC_LINKING")
# target_link_libraries(junior_ctrl behaviortree_cpp unitree_sdk2)
# target_link_libraries(junior_ctrl ddsc ddscxx rt pthread)
