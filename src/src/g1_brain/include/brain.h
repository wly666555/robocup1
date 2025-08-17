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
#include "unitree_go/msg/sport_mode_state.hpp"
#include "robot_interfaces/msg/low_state.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "locate/yaml_parser.h"
#include "locate/pose.h"

// 添加roboCup_sdk依赖
#include "brain_config.h"
#include "brain_data.h"
#include "brain_tree.h"

#include "robot_client.h"
#include "locator.h"



using namespace std;
using namespace std::placeholders;


class BrainTree;


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


    Pose p_eye2base;
    
private:


    void loadConfig();
    
    // 记忆更新
    void updateMemory();
    // 看不见球时, 可以利用记忆中球在 Field 中的位置以及机器人 Odom 信息更新球的相对位置
    void updateBallMemory();
    

    
    //---- 回调函数----//

    void servoStatesCallback(const std::shared_ptr<robot_interfaces::msg::MotorStates> msg);
    void detectionsCallback(const std::shared_ptr<robot_interfaces::msg::DetectionResults> msg);
    void odomCallback(const std::shared_ptr<unitree_go::msg::SportModeState> msg);
    void lowstateCallback(const std::shared_ptr<robot_interfaces::msg::LowState> msg);
    void joystickCallback(const std::shared_ptr<unitree_go::msg::WirelessController> msg);

    // 检测处理 
    robot_interfaces::msg::MotorState motor_state;

    
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
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr odom_sub_;
    rclcpp::Subscription<robot_interfaces::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr servo_states_sub_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr joystick_sub_;

    // 发布者
    // rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;   
};
