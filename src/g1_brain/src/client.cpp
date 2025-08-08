#include "g1_brain/client.hpp"
#include <algorithm>

RobotClient::RobotClient(rclcpp::Node* node)
    : node_(node), currentHeadYaw_(0.0), currentHeadPitch_(0.0) {
    RCLCPP_INFO(node_->get_logger(), "RobotClient created");
}

void RobotClient::init() {
    locoClient_ = std::make_unique<LocoClient>();
    locoClient_->Init();
    locoClient_->SetTimeout(10.0f);

    motor_cmd_pub_ = node_->create_publisher<robot_interfaces::msg::MotorCmds>(
        "rt/g1_comp_servo/cmd", 10);

    motor_states_sub_ = node_->create_subscription<robot_interfaces::msg::MotorStates>(
        "rt/g1_comp_servo/state", 10,
        std::bind(&RobotClient::motorStatesCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "RobotClient initialized with LocoClient for body control");
}

void RobotClient::setVelocity(double vx, double vy, double omega) {
    RCLCPP_DEBUG(node_->get_logger(), "Set velocity: vx=%.2f, vy=%.2f, omega=%.2f", vx, vy, omega);

    vx = std::clamp(vx, -1.0, 1.0);
    vy = std::clamp(vy, -1.0, 1.0);
    omega = std::clamp(omega, -1.0, 1.0);

    locoClient_->Move(vx, vy, omega);
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

void RobotClient::motorStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg) {
    if (!msg->states.empty()) {
        currentHeadYaw_ = msg->states[0].q;
        currentHeadPitch_ = 0.0;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Head servo states: yaw=%.2f, pitch=%.2f",
                 currentHeadYaw_, currentHeadPitch_);
}


