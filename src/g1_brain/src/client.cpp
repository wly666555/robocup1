
#include <algorithm>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <iostream>
#include <map>
#include "common/ut_errror.hpp"

#include "brain.h"
#include "robot_client.h"
#include "rclcpp/rclcpp.hpp"

RobotClient::RobotClient(G1Brain* argBrain)
    : brain(argBrain),
      node_(std::make_shared<rclcpp::Node>("base_client_node")),  // 创建辅助节点
    base_client_(node_.get(), "/api/sport/request", "/api/sport/response") {}

void RobotClient::init() 
{
    cmd_puber_ = brain->create_publisher<robot_interfaces::msg::MotorCmds>("/servo/motor_cmd", 10);
}


// pitch yaw (rad)
void RobotClient::moveHead(double pitch, double yaw) {

    robot_interfaces::msg::MotorCmds motor_cmds;
    motor_cmds.count = 2;
    motor_cmds.states.resize(2);

    // 假设通道 0 为 yaw，通道 1 为 pitch（根据你的伺服映射调整）
    robot_interfaces::msg::MotorState cmd_pitch;
    cmd_pitch.mode = 1;
    cmd_pitch.q = rad2deg(pitch);
    cmd_pitch.dq = 0.0f;
    cmd_pitch.ddq = 0.0f;
    // 用 kp/kd 在 MotorState 中无字段，若需要增益请在驱动侧或拓展消息定义
    cmd_pitch.tau_est = 0.0f;
    cmd_pitch.temperature = 0;
    cmd_pitch.lost = 0;

    robot_interfaces::msg::MotorState cmd_yaw;
    cmd_yaw.mode = 1;
    cmd_yaw.q = rad2deg(yaw);
    cmd_yaw.dq = 0.0f;
    cmd_yaw.ddq = 0.0f;
    // 用 kp/kd 在 MotorState 中无字段，若需要增益请在驱动侧或拓展消息定义
    cmd_yaw.tau_est = 0.0f;
    cmd_yaw.temperature = 0;
    cmd_yaw.lost = 0;

    motor_cmds.states[0] = cmd_yaw;
    motor_cmds.states[1] = cmd_pitch;

    cmd_puber_->publish(motor_cmds);
}



void RobotClient::moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance)
{
    Pose2D target_f, target_r; // 移动目标在 field 和 robot 坐标系中的 Pose
    target_f.x = tx;
    target_f.y = ty;
    target_f.theta = ttheta;
    target_r = brain->data->field2robot(target_f);
    double targetAngle = atan2(target_r.y, target_r.x);
    double targetDist = norm(target_r.x, target_r.y);

    double vx, vy, vtheta;
    // 已经到达目标?
    if (
        (fabs(brain->data->robotPoseToField.x - target_f.x) < xTolerance) && (fabs(brain->data->robotPoseToField.y - target_f.y) < yTolerance) && (fabs(toPInPI(brain->data->robotPoseToField.theta - target_f.theta)) < thetaTolerance))
    {
        SetVelocity(0, 0, 0,1.F);
    }

    static double breakOscillate = 0.0;
    if (targetDist > longRangeThreshold - breakOscillate)
    {
        breakOscillate = 0.5;

        // 角度较大, 先转向目标点
        if (fabs(targetAngle) > turnThreshold)
        {
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
            SetVelocity(0, 0, vtheta,84000.F);
        }

        // else

        vx = cap(target_r.x, vxLimit, -vxLimit);
        vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        SetVelocity(vx, 0, vtheta,84000.F);
    }

    // else 比较近了
    breakOscillate = 0.0;
    vx = cap(target_r.x, vxLimit, -vxLimit);
    vy = cap(target_r.y, vyLimit, -vyLimit);
    vtheta = cap(target_r.theta, vthetaLimit, -vthetaLimit);
    SetVelocity(vx, vy, vtheta,84000.F);
}


void RobotClient::SetVelocity(float vx, float vy, float omega, float duration) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;

    // 构造 JSON 参数
    nlohmann::json js;
    std::vector<float> velocity = {vx, vy, omega};
    js["velocity"] = velocity;
    js["duration"] = duration;
    req.parameter = js.dump();

    // 调用 BaseClient::Call 并检查结果
    nlohmann::json response_data;
    int32_t result = base_client_.Call(req, response_data);

    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotClient"), "SetVelocity failed, error code: %d", result);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RobotClient"), "SetVelocity response: %s", response_data.dump().c_str());
    }
}


void RobotClient::SetFsmId(int fsm_id) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;

    // 构造 JSON 参数
    nlohmann::json js;
    js["data"] = fsm_id;
    req.parameter = js.dump();

    // 调用 BaseClient::Call 并检查结果
    nlohmann::json response_data;
    int32_t result = base_client_.Call(req, response_data);

    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotClient"), "SetFsmId failed, error code: %d", result);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RobotClient"), "SetFsmId response: %s", response_data.dump().c_str());
    }
}

// void RobotClient::StandUp() {
//     unitree_api::msg::Request req;
//     req.header.identity.api_id = ROBOT_API_ID_LOCO_STAND_UP;

//     // 调用 BaseClient::Call 并检查结果
//     nlohmann::json response_data;
//     int32_t result = base_client_.Call(req, response_data);

//     if (result != 0) {
//         RCLCPP_ERROR(rclcpp::get_logger("RobotClient"), "StandUp failed, error code: %d", result);
//     } else {
//         RCLCPP_INFO(rclcpp::get_logger("RobotClient"), "StandUp response: %s", response_data.dump().c_str());
//     }
// }
