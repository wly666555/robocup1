#ifndef SERVO_CONTROL_HPP
#define SERVO_CONTROL_HPP

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <spdlog/spdlog.h>

// ROS2 消息支持
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"

// Unitree SDK2 标准API
#ifdef UNITREE_SDK_AVAILABLE
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/thread/thread.hpp>
#endif

// DXL includes
#include "servo_control/dxl_controller.hpp"
#include "servo_control/utilities.hpp"

namespace servo_control {

class ServoControl {
public:
    ServoControl();
    ~ServoControl();

    // 初始化函数
    bool initialize();
    void run();
    void stop();
    
    // 获取ROS2节点
    rclcpp::Node::SharedPtr getRos2Node() const { return ros2_node_; }

private:
    // ROS2 通信
    void initRos2Communication();
    void motorCmdsCallback(const robot_interfaces::msg::MotorCmd::SharedPtr msg);
    void publishMotorStates();
    
    // DDS 通信 - 使用Unitree SDK2标准API
    void initDdsCommunication();
    #ifdef UNITREE_SDK_AVAILABLE
    void motorCmdsCallbackDds(const unitree_go::msg::dds_::MotorCmds_& msg);
    void publishMotorStatesDds();
    #endif

    // 舵机控制
    void initDxlController();
    void updateServoPositions();
    void checkMotorEnable();
    
    // 控制循环
    void controlLoop();

    // 配置参数
    void loadConfig();

    // ROS2 发布者和订阅者
    rclcpp::Node::SharedPtr ros2_node_;
    rclcpp::Publisher<robot_interfaces::msg::MotorStates>::SharedPtr motor_state_pub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorCmd>::SharedPtr motor_cmd_sub_;
    
    // DDS 发布者和订阅者 - 使用Unitree SDK2标准API
    #ifdef UNITREE_SDK_AVAILABLE
    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorStates_>> motor_state_pub_dds_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorCmds_>> motor_cmd_sub_dds_;
    unitree::common::RecurrentThreadPtr control_thread_;
    #endif

    // DXL 控制器
    std::unique_ptr<DxlController> dxl_controller_;

    // 状态变量
    bool is_initialized_;
    bool is_running_;
    bool joint_enable_;
    bool simulation_mode_;  // 添加模拟模式标志

    // 舵机参数 - 参考g1_comp_servo_service
    float servo0_calibration_;
    float servo1_calibration_;
    Eigen::VectorXf joint0_limitation_;
    Eigen::VectorXf joint1_limitation_;
    float servo0_limit_encoder_;
    float servo1_limit_encoder_;
    Eigen::VectorXf direction_;
    Eigen::VectorXf servo_angle_;

    Eigen::VectorXf target_angle_command_;
    Eigen::VectorXf target_encoder_command_;
};

} // namespace servo_control

#endif // SERVO_CONTROL_HPP 