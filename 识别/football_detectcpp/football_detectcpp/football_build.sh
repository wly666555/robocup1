#!/bin/bash
# filepath: /home/unitree/Videos/football_detect_cpp/run.sh

# 设置编译输出目录
BUILD_DIR="./build"

# 创建构建目录
if [ ! -d "$BUILD_DIR" ]; then
    mkdir "$BUILD_DIR"
fi

# 进入构建目录
cd "$BUILD_DIR"

# 编译项目
echo "Compiling the project..."
cmake .. && make -j$(nproc)
if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi


