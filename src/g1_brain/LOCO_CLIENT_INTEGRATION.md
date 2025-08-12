# LocoClient 集成文档

## 概述

本文档描述了在g1_brain包中集成roboCup_sdk的LocoClient进行本体控制的修改。

## 主要修改

### 1. 头文件修改 (brain.hpp)

- 添加了roboCup_sdk的依赖：
  ```cpp
  #include <unitree/robot/g1/loco/g1_loco_client.hpp>
  using namespace unitree::robot::g1;
  ```

- 在RobotClient类中添加了LocoClient成员：
  ```cpp
  std::unique_ptr<LocoClient> locoClient_;
  ```

### 2. 实现文件修改 (brain.cpp)

#### RobotClient::init()
- 初始化LocoClient：
  ```cpp
  locoClient_ = std::make_unique<LocoClient>();
  locoClient_->Init();
  locoClient_->SetTimeout(10.0f);
  ```

#### RobotClient::setVelocity()
- 使用LocoClient的Move方法进行本体控制：
  ```cpp
  // 限制速度范围
  vx = std::clamp(vx, -1.0, 1.0);
  vy = std::clamp(vy, -1.0, 1.0);
  omega = std::clamp(omega, -1.0, 1.0);
  
  // 使用LocoClient的Move方法
  locoClient_->Move(vx, vy, omega);
  ```

### 3. CMakeLists.txt修改

- 添加了unitree_sdk2的find_package：
  ```cmake
  find_package(unitree_sdk2 REQUIRED)
  ```

- 添加了库路径：
  ```cmake
  link_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/lib/x86_64)
  link_directories(${CMAKE_SOURCE_DIR}/../roboCup_sdk/lib/aarch64)
  ```

- 链接了必要的库：
  ```cmake
  target_link_libraries(g1_brain_node
    behaviortree_cpp
    unitree_sdk2
    ddsc
    ddscxx
    rt
    pthread
  )
  ```

## 使用方法

### 基本用法

```cpp
// 在RobotClient中
void RobotClient::setVelocity(double vx, double vy, double omega) {
    // 限制速度范围
    vx = std::clamp(vx, -1.0, 1.0);
    vy = std::clamp(vy, -1.0, 1.0);
    omega = std::clamp(omega, -1.0, 1.0);
    
    // 发送移动命令
    locoClient_->Move(vx, vy, omega);
}
```

### 参考roboCup_sdk中的用法

参考`roboCup_sdk/src/control/node.cpp`中的实现：

```cpp
// 基本移动
_interface->locoClient.Move(vx, vy, vyaw);

// 停止
_interface->locoClient.Move(0, 0, 0);

// 旋转搜索
_interface->locoClient.Move(0, 0, 1);  // 缓慢旋转
```

## 测试

创建了测试程序`test_loco_client.cpp`来验证LocoClient的使用：

```bash
# 编译
./build.sh

# 运行测试（需要在实际机器人上）
./install/g1_brain/lib/g1_brain/test_loco_client
```

## 注意事项

1. **速度限制**：所有速度参数都被限制在[-1.0, 1.0]范围内
2. **超时设置**：LocoClient设置了10秒的超时时间
3. **错误处理**：建议在实际使用时添加异常处理
4. **依赖关系**：需要正确链接unitree_sdk2和相关库

## 与原有代码的兼容性

- 保持了原有的头部控制接口（moveHead）
- 修改了本体控制接口（setVelocity），现在使用LocoClient而不是舵机控制
- 保持了ROS2消息接口的兼容性

## 下一步

1. 在实际机器人上测试LocoClient的功能
2. 根据实际测试结果调整速度参数和限制
3. 添加更完善的错误处理和状态监控
4. 考虑添加运动学模型的集成 