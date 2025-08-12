#pragma once

#include <memory>
#include <iostream>
#include <string>
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"
#include "robot_interfaces/msg/motor_cmds.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_api/msg/request.hpp"
#include "locator.h"
#include "nlohmann/json.hpp"



using namespace std;

class G1Brain; // 类相互依赖，向前声明


class RobotClient {
public:
    RobotClient(G1Brain* argBrain) : brain(argBrain) {}
    void Move(float vx, float vy, float vyaw);
    void init();
    void moveHead(double pitch, double yaw);
    void StandUp();
    rclcpp::Node::SharedPtr node_;

private:

    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr motor_states_sub_;

    
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;

    G1Brain *brain;



};



