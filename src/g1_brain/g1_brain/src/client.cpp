#include "client.hpp"
#include <algorithm>

RobotClient::RobotClient(rclcpp::Node* node)
    : node_(node), currentHeadYaw_(0.0), currentHeadPitch_(0.0) {
    RCLCPP_INFO(node_->get_logger(), "RobotClient created");
}

void RobotClient::init() {

    req_puber_ = brain->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
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

void RobotClient::StandUp() {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
    req_puber_->publish(req);
}

void RobotClient::Move(float vx, float vy,
                       float vyaw) {
    unitree_api::msg::Request req;
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy;
    js["z"] = vyaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
    req_puber_->publish(req);
}

