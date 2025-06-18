#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vision_interface/msg/detections.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include <booster/robot/b1/b1_api_const.hpp>

#include "booster_interface/msg/odometer.hpp"
#include "booster_interface/msg/low_state.hpp"

#include "brain_config.h"
#include "brain_data.h"
#include "brain_log.h"
#include "brain_tree.h"
#include "locator.h"
#include "robot_client.h"

using namespace std;

/**
 * The Brain class, which inherits from rlccpp::Node.
 */
class Brain : public rclcpp::Node
{
public:
    // The BrainConfig object, which mainly contains the configuration values (static) required at runtime.
    std::shared_ptr<BrainConfig> config;
    // The BrainLog object, which encapsulates the operations related to rerun logs.
    std::shared_ptr<BrainLog> log;
    // The BrainData object, where all the runtime values of the Brain are stored.
    std::shared_ptr<BrainData> data;
    // The RobotClient object, which contains all the operations on the robot.
    std::shared_ptr<RobotClient> client;
    // The locator object.
    std::shared_ptr<Locator> locator;
    // The BrainTree object, which contains the operations related to the BehaviorTree.
    std::shared_ptr<BrainTree> tree;

    Brain();

    // Initialization operation. It only needs to be called once in the main function.
    // If the initialization process does not meet expectations, an exception can be thrown to directly abort the program.
    void init();

    // Call it within the Ros2 loop
    void tick();

    /**
     * @brief Calculate the vector angles from the current ball to the two goalposts of the opposing side in the coordinate system of the pitch.
     * @param margin double. When calculating the angles, the return value will be shifted inward by this distance compared to the actual position of the goalposts, because the ball will be blocked by the goalposts at these angles. The larger this value is, the higher the shooting accuracy will be, but the longer it will take to adjust the angles.
     * @return vector<double> In the return value, vec[0] represents the left goalpost, and vec[1] represents the right goalpost. Note that left and right are determined with the direction of the opposing side as the front.
     */
    vector<double> getGoalPostAngles(const double margin = 0.3);

    // Calibrate odom according to the ground truth. The parameters x, y, and theta represent the ground truth of the robot's Pose in the coordinate system of the pitch.
    void calibrateOdom(double x, double y, double theta);

    double msecsSince(rclcpp::Time time);

private:
    void loadConfig();

    void updateBallMemory();

    void updateMemory();

    // ------------------------------------------------------ SUB CALLBACKS ------------------------------------------------------
    void joystickCallback(const sensor_msgs::msg::Joy &msg);
    void gameControlCallback(const game_controller_interface::msg::GameControlData &msg);
    void detectionsCallback(const vision_interface::msg::Detections &msg);
    void imageCallback(const sensor_msgs::msg::Image &msg);
    void odometerCallback(const booster_interface::msg::Odometer &msg);
    void lowStateCallback(const booster_interface::msg::LowState &msg);
    void headPoseCallback(const geometry_msgs::msg::Pose &msg);
    vector<GameObject> getGameObjects(const vision_interface::msg::Detections &msg);
    void detectProcessBalls(const vector<GameObject> &ballObjs);
    void detectProcessMarkings(const vector<GameObject> &markingObjs);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<game_controller_interface::msg::GameControlData>::SharedPtr gameControlSubscription;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometerSubscription;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr headPoseSubscription;
};
