#!/bin/bash

# 设置编译输出目录
BUILD_DIR="./build"
# 设置模型文件路径
WEIGHT_FILE="../weight/weight.engine"
# 设置图像显示器参数 # 1 表示使用图像显示器，0表示不使用
IMAGE_SHOWER="1" 
# 检查构建目录是否存在
if [ ! -d "$BUILD_DIR" ]; then
    echo "Error: Build directory does not exist. Please compile the project first using the following commands:"
    echo "chmod +x football_build.sh"
    echo "./football_build.sh"
    echo "Then run this script again."
    exit 1
fi
# 进入构建目录
cd "$BUILD_DIR"

# 检查模型文件是否存在
if [ ! -f "$WEIGHT_FILE" ]; then
    echo "Error: Model file '$WEIGHT_FILE' not found. Please convert the model first using the following commands:"
    echo "chmod +x football_onnx2trt.sh"
    echo "./football_onnx2trt.sh"
    exit 1
fi



# 检查可执行文件是否存在
EXECUTABLE="./football_detect"
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable file '$EXECUTABLE' not found. Please ensure the project is compiled correctly."
    exit 1
fi

# 执行程序
echo "Running the program..."
$EXECUTABLE $WEIGHT_FILE $IMAGE_SHOWER