#ifndef G1_BRAIN_CLIENT_HPP
#define G1_BRAIN_CLIENT_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"
#include "robot_interfaces/msg/motor_cmds.hpp"
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

using namespace unitree::robot::g1;

class RobotClient {
public:
    explicit RobotClient(rclcpp::Node* node);
    ~RobotClient() = default;

    void init();
    void setVelocity(double vx, double vy, double omega);
    void moveHead(double yaw, double pitch);

    double getCurrentHeadYaw() const { return currentHeadYaw_; }
    double getCurrentHeadPitch() const { return currentHeadPitch_; }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr motor_states_sub_;

    std::unique_ptr<LocoClient> locoClient_;

    double currentHeadYaw_;
    double currentHeadPitch_;

};

#endif // G1_BRAIN_CLIENT_HPP


