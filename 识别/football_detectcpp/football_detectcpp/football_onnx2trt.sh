#!/bin/bash
# filepath: /home/unitree/Videos/football_detect_cpp/run.sh

# 设置编译输出目录
BUILD_DIR="./build"
# 设置模型文件路径
WEIGHT_FILE="../weight/weight.onnx"
# 创建构建目录
if [ ! -d "$BUILD_DIR" ]; then
    echo "Error: Build directory does not exist. Please compile the project first using the following commands:"
    echo "chmod +x football_build.sh"
    echo "./football_build.sh"
    echo "Then run this script again."
    exit 1
fi

# 进入构建目录
cd "$BUILD_DIR"

# 检查可执行文件是否存在
EXECUTABLE="./football_detect"
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable file '$EXECUTABLE' not found. Please ensure the project is compiled correctly."
    exit 1
fi

# 执行程序
echo "Running the program..."
$EXECUTABLE $WEIGHT_FILE

