# G1 Brain Package

宇树机器人足球系统的核心大脑节点，负责处理舵机状态和视觉检测数据，执行定位和决策逻辑。

## 概述

G1 Brain是宇树机器人足球系统的中央处理单元，实现了类似于加速进化机器人足球赛brain节点的功能，但针对宇树机器人进行了优化。该系统接收舵机状态和视觉检测数据，通过定位模块确定机器人位置，通过行为树进行决策，并发送控制命令。

## 功能特性

- **舵机状态处理**: 接收和处理舵机状态信息，包括头部舵机角度
- **视觉检测处理**: 处理来自视觉系统的检测结果，包括球、球门柱、人、对手、场地标记等
- **机器人定位**: 基于场地标记进行机器人定位（占位符实现）
- **球检测和记忆**: 检测球位置并维护记忆，处理球丢失逻辑
- **行为树决策**: 执行决策逻辑（占位符实现）
- **控制命令发送**: 发送机器人控制命令到舵机系统
- **坐标转换**: 实现场地坐标系和机器人坐标系之间的转换
- **参数配置**: 支持运行时参数配置和调整

## 系统架构

```
Vision节点 → 检测结果 → G1Brain节点 → Locate模块（定位）
                                    ↓
Servo节点 ← cmd ← G1Brain节点 ← BT模块（决策）
                                    ↑
Servo节点 → servo state → G1Brain节点
```

### 数据流

1. **输入数据**:
   - 舵机状态数据 (`/servo/motor_states`)
   - 视觉检测结果 (`/vision/detections`)

2. **处理流程**:
   - 数据接收和预处理
   - 坐标转换和定位
   - 球检测和记忆更新
   - 行为树决策执行

3. **输出数据**:
   - 舵机控制命令 (`/servo/motor_cmd`)

## 话题接口

### 订阅话题
- `/servo/motor_states` (robot_interfaces/msg/MotorStates): 舵机状态信息
- `/vision/detections` (robot_interfaces/msg/DetectionResults): 视觉检测结果

### 发布话题
- `/servo/motor_cmd` (robot_interfaces/msg/MotorCmd): 舵机控制命令

## 参数配置

### 游戏相关参数
- `game.team_id`: 队伍ID (默认: 0)
- `game.player_id`: 球员ID (默认: 29)
- `game.player_role`: 球员角色 (默认: "striker")
- `game.field_type`: 场地类型 (默认: "")
- `game.player_start_pos`: 起始位置 (默认: "")

### 机器人相关参数
- `robot.robot_height`: 机器人高度 (默认: 1.0)
- `robot.odom_factor`: 里程计因子 (默认: 1.0)
- `robot.vx_factor`: 前进速度因子 (默认: 0.95)
- `robot.yaw_offset`: 偏航偏移 (默认: 0.1)

### 记忆相关参数
- `memory.ball_memory_length`: 球位置记忆时间（秒）(默认: 5.0)

## 安装和编译

### 环境要求
- ROS2 Foxy
- C++17 编译器
- robot_interfaces 包

### 编译步骤

1. **编译robot_interfaces包**:
```bash
cd ~/Desktop/unitree_football/ROS2_node/robot_interfaces
source /opt/ros/foxy/setup.bash
colcon build
```

2. **编译g1_brain包**:
```bash
cd ~/Desktop/unitree_football/ROS2_node/g1_brain
source /opt/ros/foxy/setup.bash
source ../robot_interfaces/install/setup.bash
colcon build
```

## 使用方法

### 启动节点

1. **使用launch文件启动**:
```bash
cd ~/Desktop/unitree_football/ROS2_node/g1_brain
source install/setup.bash
ros2 launch g1_brain g1_brain_launch.py
```

2. **直接启动节点**:
```bash
cd ~/Desktop/unitree_football/ROS2_node/g1_brain
source install/setup.bash
ros2 run g1_brain g1_brain_node
```

### 参数配置

1. **启动时指定参数**:
```bash
ros2 launch g1_brain g1_brain_launch.py team_id:=1 player_id:=30 player_role:=goal_keeper
```

2. **运行时修改参数**:
```bash
ros2 param set /g1_brain_node game.team_id 1
ros2 param set /g1_brain_node game.player_role goal_keeper
```

3. **查看当前参数**:
```bash
ros2 param list /g1_brain_node
ros2 param get /g1_brain_node game.team_id
```

### 话题监控

1. **查看话题列表**:
```bash
ros2 topic list
```

2. **监控话题数据**:
```bash
# 监控舵机状态
ros2 topic echo /servo/motor_states

# 监控视觉检测结果
ros2 topic echo /vision/detections

# 监控控制命令
ros2 topic echo /servo/motor_cmd
```

## 依赖包

### 必需依赖
- `rclcpp`: ROS2 C++客户端库
- `robot_interfaces`: 机器人接口消息

### 可选依赖
- `ament_cmake`: 构建系统
- `ament_lint_auto`: 代码检查工具

## 文件结构

```
g1_brain/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # 包定义
├── README.md                   # 说明文档
├── include/
│   └── g1_brain/
│       └── g1_brain.hpp       # 头文件（类定义和数据结构）
├── src/
│   └── g1_brain_node.cpp      # 主实现文件
├── launch/
│   └── g1_brain_launch.py     # 启动文件
└── config/
    └── g1_brain_config.yaml   # 配置文件
```

## 核心组件

### G1Brain类
主要的节点类，负责：
- 参数管理和配置加载
- 话题订阅和发布
- 系统初始化和主循环
- 数据流协调

### Locator类（占位符）
负责机器人定位功能：
- 基于场地标记的定位算法
- 坐标转换和姿态估计
- 定位精度优化

### BehaviorTree类（占位符）
负责决策逻辑：
- 行为树执行引擎
- 状态管理和转换
- 决策优先级处理

### RobotClient类
负责控制命令发送：
- 舵机命令生成
- 速度控制接口
- 头部控制接口

## 开发说明

### 扩展定位模块
1. 实现Locator类的具体算法
2. 添加定位精度评估
3. 集成多种定位方法

### 扩展决策模块
1. 实现BehaviorTree类的决策逻辑
2. 添加状态机管理
3. 集成多种行为策略

### 扩展控制模块
1. 实现RobotClient类的具体控制逻辑
2. 添加安全检查和限制
3. 优化控制响应

## 调试和测试

### 日志级别设置
```bash
# 设置日志级别
ros2 run g1_brain g1_brain_node --ros-args --log-level debug

# 查看节点日志
ros2 run g1_brain g1_brain_node --ros-args --log-level info
```

### 单元测试
```bash
# 编译测试
colcon build --packages-select g1_brain --cmake-args -DBUILD_TESTING=ON

# 运行测试
colcon test --packages-select g1_brain
```

## 故障排除

### 常见问题

1. **编译错误**:
   - 确保ROS2环境已正确设置
   - 检查robot_interfaces包是否已编译
   - 确认C++17支持

2. **运行时错误**:
   - 检查话题名称是否正确
   - 确认依赖节点是否运行
   - 验证参数配置

3. **性能问题**:
   - 调整tick频率（默认100Hz）
   - 优化检测处理算法
   - 减少不必要的计算

## 注意事项

1. **系统要求**:
   - 确保舵机系统和视觉系统正常运行
   - 检查话题名称是否正确映射
   - 根据实际机器人配置调整参数

2. **开发建议**:
   - 定位和决策模块需要根据具体需求实现
   - 建议添加单元测试和集成测试
   - 考虑添加配置验证和错误处理

3. **性能优化**:
   - 优化坐标转换算法
   - 减少内存分配和拷贝
   - 使用高效的数据结构

## 版本历史

- **v0.0.1**: 初始版本，实现基本框架和占位符模块
- 支持舵机状态和视觉检测数据处理
- 实现坐标转换和球检测逻辑
- 提供参数配置和话题接口

## 贡献指南

1. Fork项目
2. 创建功能分支
3. 提交更改
4. 创建Pull Request

## 许可证

MIT License

## 联系方式

如有问题或建议，请联系开发团队。 