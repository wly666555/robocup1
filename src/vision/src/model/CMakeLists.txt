# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("embed_platform on")
  include_directories(/usr/local/cuda/targets/aarch64-linux/include)
  link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
  message("embed_platform off")
  # cuda
  include_directories(/usr/local/cuda/include)
  link_directories(/usr/local/cuda/lib64)

  # tensorrt
  include_directories(/usr/local/TensorRT-8.6.1.6/include)
  link_directories(/usr/local/TensorRT-8.6.1.6/lib)
endif()

add_subdirectory(trt)

add_library(yolov8_model SHARED detector.cc)

target_link_libraries(yolov8_model PUBLIC yolov8_trt ${YAML_CPP_LIBRARIES})

install(TARGETS 
  yolov8_model
  yolov8_trt
  DESTINATION lib)