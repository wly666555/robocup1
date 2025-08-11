#ifndef G1_BRAIN_CLIENT_HPP
#define G1_BRAIN_CLIENT_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"
#include "robot_interfaces/msg/motor_cmds.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_api/msg/request.hpp"
#include "locator.h"
#include "nlohmann/json.hpp"

class RobotClient {
public:
    explicit RobotClient(rclcpp::Node* node){}
    ~RobotClient() = default;
    double currentHeadYaw_;
    double currentHeadPitch_;

    void init();
    void moveHead(double yaw, double pitch);
    void StandUp();
    double getCurrentHeadYaw() const { return currentHeadYaw_; }
    double getCurrentHeadPitch() const { return currentHeadPitch_; }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr motor_states_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;

    // std::unique_ptr<LocoClient> locoClient_;



};

#endif // G1_BRAIN_CLIENT_HPP


