#ifndef G1_BRAIN_HPP
#define G1_BRAIN_HPP

#include <memory>
#include <vector>
#include <string>
#include <any>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "robot_interfaces/msg/low_state.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
#include "locate/types.h"
#include "locator.h"
#include "locate/pose.h"
#include "locate/yaml_parser.h"
// 添加roboCup_sdk依赖
#include "brain_config.h"
#include "client.hpp"
#include <cmath>
#include <sstream>
#include "locate/math_utils.h"
#include "brain_data.h"

class BrainTree;
using namespace std::placeholders;

class SelfLocate;



/**
 * @brief 宇树机器人足球系统核心大脑节点
 * 
 * 主要功能：
 * - 接收舵机状态和视觉检测结果
 * - 机器人定位
 * - 行为树决策
 * - 发送控制命令
 */
class G1Brain : public rclcpp::Node {
public:
    void publishMotorCmds() {
        motor_cmd_pub_->publish(motor_cmds);
    }
    std::shared_ptr<BrainData> getData() const { return data; }
    const robot_interfaces::msg::MotorStates& getMotorStates() const { return motor_states; }
    robot_interfaces::msg::MotorCmds& getMotorCmds() { return motor_cmds; }
    Locator locator;
    G1Brain();
    ~G1Brain() = default;
    Pose p_eye2base;
    FieldDimensions fd;
    void init();
    void tick();
    std::shared_ptr<BrainConfig> getConfig() const { return config; }
    void calibrateOdom(double x, double y, double theta);
    
private:
    // 配置和数据
    std::shared_ptr<BrainConfig> config;
    std::shared_ptr<BrainData> data;
    // std::shared_ptr<Locator> locator;
    std::shared_ptr<BrainTree> tree;
    std::shared_ptr<SelfLocate> selflocate;
    std::shared_ptr<RobotClient> client;
    
    // std::shared_ptr<BrainLog> log;
    
    // 成员变量
    double odometry_factor_{1.0};
    double servo_pitch_compensation_{0.0};
    double servo_yaw_compensation_{0.0};
    double servo_height_{0.0};

    // 参数声明
    void loadConfig();
    
    // 记忆更新
    void updateMemory();
    void updateBallMemory();
    
    // 回调函数
    void motorStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg);
    void detectionsCallback(const robot_interfaces::msg::DetectionResults::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void lowstateCallback(const robot_interfaces::msg::LowState::SharedPtr msg);
    void mainLoop();
    
    // 检测处理
    nav_msgs::msg::Odometry last_odom_;
    robot_interfaces::msg::LowState low_state;
    robot_interfaces::msg::MotorStates motor_states;
    robot_interfaces::msg::MotorState motor_state;
    robot_interfaces::msg::LowState last_lowstate_;
    robot_interfaces::msg::MotorCmds motor_cmds;
    std::vector<robot_interfaces::msg::DetectionResult> last_detections_;
    std::vector<GameObject> getGameObjects(
        const std::vector<robot_interfaces::msg::DetectionResult_<std::allocator<void>>>& detections,
        const Pose& robot_pose,
        const Pose2D& map_pose);
    void detectProcessBalls(const std::vector<GameObject>& ballObjs);
    void detectProcessMarkings(const std::vector<GameObject>& markingObjs);

    
    // 订阅者
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr motorStatesSubscription;
    rclcpp::Subscription<robot_interfaces::msg::DetectionResults>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<robot_interfaces::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr motor_states_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr motor_cmd_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;   
};

#endif // G1_BRAIN_HPP 