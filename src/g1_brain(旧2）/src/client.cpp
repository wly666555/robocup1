#include "client.hpp"
#include <algorithm>

RobotClient::RobotClient(rclcpp::Node* node)
    : node_(node), currentHeadYaw_(0.0), currentHeadPitch_(0.0) {
    RCLCPP_INFO(node_->get_logger(), "RobotClient created");
}

void RobotClient::init2() {
    // locoClient_ = std::make_unique<LocoClient>();
    // locoClient_->Init();
    // locoClient_->SetTimeout(10.0f);
    chassis_cmd_pub_ = node_->create_publisher<unitree_go::msg::PathPoint>("chassis_cmd", 10);
    RCLCPP_INFO(node_->get_logger(), "RobotClient initialized with LocoClient for body control");
}

void RobotClient::setVelocity(double vx, double vy, double omega) {
    unitree_go::msg::PathPoint cmd;
    cmd.t_from_start = 0.0f; // 或者用当前时间戳
    cmd.x = 0.0f;            // 目标位置（如果只控制速度可以不填）
    cmd.y = 0.0f;
    cmd.yaw = 0.0f;
    cmd.vx = static_cast<float>(vx);
    cmd.vy = static_cast<float>(vy);
    cmd.vyaw = static_cast<float>(omega);

    chassis_cmd_pub_->publish(cmd);
}

void RobotClient::moveHead(double yaw, double pitch) {
    RCLCPP_DEBUG(node_->get_logger(), "Move head: yaw=%.2f, pitch=%.2f", yaw, pitch);

    robot_interfaces::msg::MotorCmds motor_cmds;
    motor_cmds.count = 2;
    motor_cmds.states.resize(2);

    // 假设通道 0 为 yaw，通道 1 为 pitch（根据你的伺服映射调整）
    robot_interfaces::msg::MotorState cmd_yaw;
    cmd_yaw.mode = 1;
    cmd_yaw.q = static_cast<float>(yaw);
    cmd_yaw.dq = 0.0f;
    cmd_yaw.ddq = 0.0f;
    // 用 kp/kd 在 MotorState 中无字段，若需要增益请在驱动侧或拓展消息定义
    cmd_yaw.tau_est = 0.0f;
    cmd_yaw.temperature = 0;
    cmd_yaw.lost = 0;

    robot_interfaces::msg::MotorState cmd_pitch = cmd_yaw;
    cmd_pitch.q = static_cast<float>(pitch);

    motor_cmds.states[0] = cmd_yaw;
    motor_cmds.states[1] = cmd_pitch;

    motor_cmd_pub_->publish(motor_cmds);
}


