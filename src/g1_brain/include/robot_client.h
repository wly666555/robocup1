#pragma once

#include <cstdint>
#include <limits>
#include <rclcpp/node.hpp>

#include <memory>
#include <iostream>
#include <string>
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"
#include "robot_interfaces/msg/motor_state.hpp"
#include "robot_interfaces/msg/motor_cmds.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_api/msg/request.hpp"
#include "locator.h"
#include <behaviortree_cpp/contrib/json.hpp>

#include "common/base_client.hpp"
#include "common/ut_errror.hpp"
#include "common/patch.hpp"



using namespace std;

class G1Brain; // 类相互依赖，向前声明



class RobotClient {
public:
    RobotClient(G1Brain* argBrain);
    std::shared_ptr<rclcpp::Node> node_; 

    void init();
    void moveHead(double pitch, double yaw);
    void moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);
    // void Move(float vx, float vy, float vyaw);
    void SetFsmId(int fsm_id);
    void SetVelocity(float vx, float vy, float omega, float duration = 1.F);
    // void StandUp();

private:
    G1Brain *brain;  // 指向父节点
    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr cmd_puber_;
    BaseClient base_client_;  // 用于请求-响应处理
};

