/**
 * @file main.cpp
 * @brief 宇树机器人足球系统的核心大脑节点主程序
 * 
 * 该文件是g1_brain包的主入口，负责启动和运行整个机器人足球系统的大脑节点。
 * 采用多线程架构，分离ROS2消息处理和逻辑执行。
 * 
 * 系统架构：
 * Vision节点 → 检测结果 → G1Brain节点 → Locate模块（定位）
 *                                    ↓
 * g1_comp_servo_service ← 头部控制 ← G1Brain节点 ← BT模块（决策）
 *                                    ↑
 * g1_comp_servo_service → 舵机状态 → G1Brain节点
 * 
 * 本体控制：
 * G1Brain节点 → LocoClient → 宇树SDK → 机器人本体
 * 
 * 话题通信：
 * - 视觉检测：detection_results
 * - 头部控制：rt/g1_comp_servo/cmd
 * - 舵机状态：rt/g1_comp_servo/state (RobotClient)
 * - 舵机状态：servo/motor_states (G1Brain)
 */
#include "g1_brain/brain.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<G1Brain>();
    node->init();
    
    // 创建定时器，以10Hz的频率执行tick
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),  // 100ms = 10Hz
        [node]() {
            node->tick();
        });
    
    RCLCPP_INFO(node->get_logger(), "G1Brain node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 