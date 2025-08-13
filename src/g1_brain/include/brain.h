#pragma once


#include <memory>
#include <vector>
#include <string>
#include <any>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
#include "robot_interfaces/msg/low_state.hpp"
#include "locate/yaml_parser.h"
#include "locate/pose.h"
// 添加roboCup_sdk依赖
#include "brain_config.h"
#include "brain_data.h"
#include "brain_tree.h"

#include "robot_client.h"
#include "locator.h"


class BrainTree;
using namespace std::placeholders;




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

    // BrainConfig 对象，主要包含运行时需要的配置值（静态）
    std::shared_ptr<BrainConfig> config;
    // BrainData 对象，Brain 所有运行时的值都放在这里
    std::shared_ptr<BrainData> data;
    // RobotClient 对象，包含所有对机器人的操作
    std::shared_ptr<RobotClient> client;
    // locator 对象
    std::shared_ptr<Locator> locator;
    // BrainTree 对象，里面包含 BehaviorTree 相关的操作
    std::shared_ptr<BrainTree> tree;

    // 构造函数，接受 nodeName 创建 ros2 结点
    G1Brain();

    void init();

    void tick();

    vector<double> getGoalPostAngles(const double margin = 0.3);

    void calibrateOdom(double x, double y, double theta);

    double msecsSince(rclcpp::Time time);



    void publishMotorCmds() {
        motor_cmd_pub_->publish(motor_cmds);}
    const robot_interfaces::msg::MotorStates& getMotorStates() const { return motor_states; }
    robot_interfaces::msg::MotorCmds& getMotorCmds() { return motor_cmds; }

    Pose p_eye2base;
    
private:
    
    // 成员变量
    double odometry_factor_{1.0};
    double servo_pitch_compensation_{0.0};
    double servo_yaw_compensation_{0.0};
    double servo_height_{0.0};


    void loadConfig();
    
    // 记忆更新
    void updateMemory();
    // 看不见球时, 可以利用记忆中球在 Field 中的位置以及机器人 Odom 信息更新球的相对位置
    void updateBallMemory();
    

    
    //---- 回调函数----//

    // 处理舵机消息
    void servoStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg);
    // 处理视觉识别消息
    void detectionsCallback(const robot_interfaces::msg::DetectionResults::SharedPtr msg);
    // 处理里程计消息
    void odomCallback(const nav_msgs::msg::Odometry &msg);
    //处理底层状态信息
    void lowstateCallback(const robot_interfaces::msg::LowState::SharedPtr msg);
    

    // 检测处理
    nav_msgs::msg::Odometry last_odom_;   //!!修改！！
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
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr servoStatesSubscription;
    rclcpp::Subscription<robot_interfaces::msg::DetectionResults>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<robot_interfaces::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr servo_states_sub_;

    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr motor_cmd_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;   
};
