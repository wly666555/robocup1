#ifndef G1_BRAIN_HPP
#define G1_BRAIN_HPP

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <any>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"
#include "robot_interfaces/msg/detection_result.hpp"
#include "robot_interfaces/msg/detection_results.hpp"
#include "robot_interfaces/msg/location_result.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "types.h"
#include "locator.h"
#include "pose.h"
#include "yaml_parser.h"
// 添加roboCup_sdk依赖
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include "g1_brain/tree.hpp"
#include <cmath>
#include <sstream>
#include "math_utils.h"

using namespace std::placeholders;
using namespace unitree::robot::g1;

/**
 * @brief 配置类
 * 存储系统配置参数
 */
struct BrainConfig {
    // 游戏相关参数

    std::string field_size;
    std::string location_mode;
    std::string playerStartPos;
    
    // 机器人相关参数
    double pitch_compensation;
    double yaw_compensation;
    double height;
    double scale_factor;
    
    // 记忆相关参数
    double memoryLength;  // 球位置记忆时间（秒）
    
    void handle() {
        // 参数处理逻辑
    }
    
    void print(std::ostringstream& oss) const {
        oss << "field_size=" << field_size 
            << ", location_mode=" << location_mode 
            << ", playerStartPos=" << playerStartPos;
    }
};

/**
 * @brief 数据存储类
 * 存储系统运行时的数据
 */
struct BrainData {
    // 机器人状态
    RobotPose robotPoseToField;
    RobotPose robotPoseToOdom;
    double headYaw = 0.0;
    double headPitch = 0.0;
    
    // 球信息
    Ball ball;
    bool ballDetected = false;
    double robotBallAngleToField = 0.0;
    
    // 检测对象
    std::vector<GameObject> markings;
    std::vector<GameObject> opponents;
    std::vector<GameObject> goalposts;
    
    // 时间相关
    rclcpp::Time lastSuccessfulLocalizeTime;
};

/**
 * @brief 机器人客户端类
 * 负责发送控制命令
 */
class RobotClient {
public:
    RobotClient(rclcpp::Node* node);
    ~RobotClient() = default;
    
    void init();
    void setVelocity(double vx, double vy, double omega);
    void moveHead(double yaw, double pitch);
    
    // 获取当前头部位置
    double getCurrentHeadYaw() const { return currentHeadYaw_; }
    double getCurrentHeadPitch() const { return currentHeadPitch_; }
    
private:
    rclcpp::Node* node_;
    rclcpp::Publisher<robot_interfaces::msg::MotorCmd>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr motor_states_sub_;
    
    // 添加LocoClient用于本体控制
    std::unique_ptr<LocoClient> locoClient_;
    
    double currentHeadYaw_;
    double currentHeadPitch_;
    
    void motorStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg);
};

/**
 * @brief 日志类（简化版）
 */
class BrainLog {
public:
    BrainLog(rclcpp::Node* node);
    
    void prepare();
    bool isEnabled() const { return enabled_; }
    void setTimeNow();
    void setTimeSeconds(double time);
    
    template<typename T>
    void log(const std::string& path, const T& data) {
        // 占位符实现
    }
    
private:
    rclcpp::Node* node_;
    bool enabled_;
};

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
    G1Brain();
    ~G1Brain() = default;
    
    void init();
    void tick();
    
private:
    // 配置和数据
    std::shared_ptr<BrainConfig> config;
    std::shared_ptr<BrainData> data;
    std::shared_ptr<Locator> locator;
    std::shared_ptr<BehaviorTree> tree;
    std::shared_ptr<RobotClient> client;
    std::shared_ptr<BrainLog> log;
    
    // 成员变量
    double odometry_factor_{1.0};
    double servo_pitch_compensation_{0.0};
    double servo_yaw_compensation_{0.0};
    double servo_height_{0.0};

    // 参数声明
    void declareParameters();
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
    robot_interfaces::msg::LowState last_lowstate_;
    std::vector<robot_interfaces::msg::DetectionResult> last_detections_;
    std::vector<GameObject> getGameObjects(const robot_interfaces::msg::DetectionResults& detections);
    void detectProcessBalls(const std::vector<GameObject>& ballObjs);
    void detectProcessMarkings(const std::vector<GameObject>& markingObjs);
    void calibrateOdom(double x, double y, double theta);
    
    // 订阅者
    rclcpp::Subscription<robot_interfaces::msg::MotorStates>::SharedPtr motorStatesSubscription;
    rclcpp::Subscription<robot_interfaces::msg::DetectionResults>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<robot_interfaces::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;   
};

#endif // G1_BRAIN_HPP 