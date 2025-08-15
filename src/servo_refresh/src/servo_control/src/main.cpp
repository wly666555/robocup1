#include "servo_control/servo_control.hpp"
#include <signal.h>
#include <iostream>

// ROS2支持
#include "rclcpp/rclcpp.hpp"

static bool running = true;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

int main(int argc, char** argv) {
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // 初始化ROS2
        rclcpp::init(argc, argv);
        
        // 创建舵机控制对象
        servo_control::ServoControl servo_control;
        
        // 初始化
        if (!servo_control.initialize()) {
            std::cerr << "Failed to initialize servo control" << std::endl;
            rclcpp::shutdown();
            return -1;
        }
        
        // 启动控制
        servo_control.run();
        
        std::cout << "Servo control started. Press Ctrl+C to stop." << std::endl;
        
        // 使用ROS2的spin机制处理节点
        auto node = servo_control.getRos2Node();
        
        // 主循环 - 使用ROS2的spin_some机制
        while (running && rclcpp::ok()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // 停止控制
        servo_control.stop();
        
        std::cout << "Servo control stopped." << std::endl;
        
        // 关闭ROS2
        rclcpp::shutdown();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return -1;
    }
    
    return 0;
} 