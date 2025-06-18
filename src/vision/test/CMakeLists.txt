
add_executable(test_yolov8 model/test_yolov8.cpp)
target_link_libraries(test_yolov8 yolov8_model pthread
                                  /usr/local/lib/libgtest.so
                                  /usr/local/lib/libgtest_main.so
) 

add_executable(test_utils base/test_utils.cpp)
target_link_libraries(test_utils booster_vision_base pthread
                                 /usr/local/lib/libgtest.so
                                 /usr/local/lib/libgtest_main.so
)

add_executable(test_intrin base/test_intrin.cpp)
target_link_libraries(test_intrin booster_vision_base pthread
                                  /usr/local/lib/libgtest.so
                                  /usr/local/lib/libgtest_main.so
)