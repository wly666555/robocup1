#include <iostream>

#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/thread/thread.hpp>
#include <math.h>

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include "DetectionModule.hpp"
#include "LocationModule.hpp"
#include "pose.h"
#include "Locator.h"
#include "misc.h"


int main(int argc, char *argv[])
{
    // 检查命令行参数，确保提供了网络接口和配置文件路径
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " [networkInterface] [config_file] [is_display]" << std::endl;
        return -1;
    }
    // 加载配置文件
    std::string env_cfg_path = "../config.yaml";
    if (argc >= 3) {
        env_cfg_path = argv[2];// 从命令行参数中获取配置文件路径
    } 
    // 是否显示调试信息
    // 0: 不显示，1: 显示
    bool is_display = false;
    if (argc == 4 && strcmp(argv[3], "1") == 0) {
        is_display = true;
    }

    
    // 初始化DDS通信系统
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    //订阅
    // Create servo Subscriber：servostate是一个订阅者对象，用于接收来自"rt/g1_comp_servo/state"主题的MotorStates消息。
    // 该主题包含了机器人的伺服电机状态信息(例如位置、速度)，servostate对象通过订阅该主题来获取这些信息。
    // 这里使用了unitree::robot::SubscriptionBase模板类来创建订阅者对象，模板参数指定了消息类型为unitree_go::msg::dds_::MotorStates_。
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>> servoState;
    servoState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/g1_comp_servo/state");
    servoState->msg_.states().resize(2); //存储两个电机的状态信息

    // Create wrist Subscriber：lowState是一个订阅者对象，用于接收来自"rt/lowstate"主题的LowState消息。机器人底层状态：imu姿态，电机状态等
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>> lowState;
    lowState = std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");

    // Create Odometry Subscriber ：odomState是一个订阅者对象，用于接收来自"rt/lf/odommodestate"主题的SportModeState消息。
    // 该主题包含了机器人的里程计状态信息(例如位置、姿态、速度)，odomState对象通过订阅该主题来获取这些信息。
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>> odomState;
    odomState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>>("rt/lf/odommodestate");

    // Create Detection Subscriber ：detectionSub是一个订阅者对象，用于接收来自"detectionresults"主题的DetectionResults消息。
    // 该主题包含了机器人的检测结果信息(例如目标位置、目标类型)，detectionSub对象通过订阅该主题来获取这些信息。
    dds::domain::DomainParticipant participant(0);// 创建一个DDS域参与者，域ID为0。
    dds::topic::Topic<DetectionModule::DetectionResults> topic(participant, "detectionresults");// 创建一个DDS主题，主题名称为"detectionresults"。
    dds::sub::Subscriber subscriber(participant); // 创建一个DDS订阅者，关联到之前创建的域参与者。
    dds::sub::DataReader<DetectionModule::DetectionResults> reader(subscriber, topic); // 创建一个DDS数据读取器，关联到之前创建的主题。

    //发布
    // Create Location Publisher ：posePub是一个发布者对象，用于向"rt/locationresults"主题发布位置信息。
    // 该主题用于发布机器人的位置信息(例如位置、姿态)，posePub对象通过发布该主题来向其他系统提供位置信息。
    // 这里使用了unitree::robot::RealTimePublisher模板类来创建发布
    std::unique_ptr<unitree::robot::RealTimePublisher<LocationModule::LocationResult>> posePub; 
    posePub = std::make_unique<unitree::robot::RealTimePublisher<LocationModule::LocationResult>>("rt/locationresults");


    // Load config file ：config是一个YamlParser对象，用于解析配置文件。
    // 该配置文件包含了机器人的各种参数设置(例如比例系数、补偿值)，config对象通过解析该文件来获取这些参数。
    YamlParser config;
    try {
        config.setup(env_cfg_path.c_str()); // 调用setup方法加载配置文件，传入配置文件路径env_cfg_path。
    }
    catch(const std::exception& e)
    {
        std::cout << "[ERROR] Cannot find config file: " << env_cfg_path << std::endl; // 如果找不到配置文件，则输出错误信息并退出程序
        return 0;
    }

    Locator locator; // 创建一个Locator对象，用于处理机器人的定位任务。
    locator.init(config); // 初始化Locator对象，传入配置文件对象config。

    double odometry_factor = config.ReadFloatFromYaml("odometry", "scale_factor"); // 从配置文件中读取里程计的缩放因子，用于将里程计数据转换为实际的位置信息。
    double servo_pitch_compensation = config.ReadFloatFromYaml("servo", "pitch_compensation"); // 从配置文件中读取伺服电机的俯仰补偿值，用于调整伺服电机的姿态。
    double servo_yaw_compensation = config.ReadFloatFromYaml("servo", "yaw_compensation"); // 从配置文件中读取伺服电机的偏航补偿值，用于调整伺服电机的姿态。
    double servo_height = config.ReadFloatFromYaml("servo", "height"); // 从配置文件中读取伺服电机的高度，用于计算伺服电机相对于机器人的位置。

    while (true) {// 主循环，持续运行定位任务

        // Read odometry data
        locator.robotPoseToOdom.x = odomState->msg_.position()[0] * odometry_factor;// 从里程计状态消息中读取机器人的位置数据，并乘以缩放因子以获取实际位置。
        locator.robotPoseToOdom.y = odomState->msg_.position()[1] * odometry_factor;
        locator.robotPoseToOdom.theta = odomState->msg_.imu_state().rpy()[2]; // 读取偏航角，rpy()[2]表示roll、pitch、yaw三个角度中的偏航角。

        // 输出机器人的位置和姿态信息
        std::cout << "Odometer information: (" 
                    << locator.robotPoseToOdom.x << ", "
                    << locator.robotPoseToOdom.y << ", "
                    << locator.robotPoseToOdom.theta << ")" << std::endl;

        // Read servo data
        double wrist_yaw_angle = rad2deg(lowState->msg_.motor_state()[JointIndex::kWaistYaw].q()); // 从lowstate中读取腰部偏航角度，并转换为度数。
        double servo_yaw_angle = servoState -> msg_.states()[0].q(); // 从servoState中读取伺服电机的偏航角度。
        double servo_pitch_angle = servoState -> msg_.states()[1].q() + servo_pitch_compensation; // 从servoState读取伺服电机的俯仰角度，并加上补偿值。
	    Pose p_eye2base = Pose(0, -servo_height, 0, deg2rad(servo_pitch_angle), -deg2rad(wrist_yaw_angle) - deg2rad(servo_yaw_angle), 0) ; 
        // 创建一个Pose对象p_eye2base，表示伺服电机相对于机器人的位置和姿态。这里的参数包括位置坐标和姿态角度。
	
        // 输出伺服电机的角度信息
        std::cout << "Servo information: "
                 << "wrist_yaw_angle(" << wrist_yaw_angle << "), "
                 << "servo_yaw_angle(" << servo_yaw_angle << "), "
                 << "servo_pitch_angle(" << servo_pitch_angle << ")" << std::endl;

        // Read detection data 从检测结果消息中读取检测到的目标信息。
        auto samples = reader.take(); // 从订阅者对象reader中获取检测结果样本。
        for (const auto &sample : samples) { // 遍历每个检测结果样本
            if (sample.info().valid()) { // 检测结果样本是有效的
                const auto& det_results = sample.data().results(); // 获取检测结果中的目标信息
                std::cout << "===== Received " << det_results.size() << " detection results =====\n";// 输出检测结果的数量
                if (det_results.size() > 0) { // 如果检测结果中有目标信息
                    locator.processDetections(det_results, p_eye2base); // 调用Locator对象的processDetections方法，处理检测结果和伺服电机相对于机器人的位置和姿态。
                    locator.selfLocate(); // 调用Locator对象的selfLocate方法，进行自我定位。
                }   
            }
        }
        
        // Compute and send location results 计算并发送定位结果
        if (locator.odomCalibrated) { // 如果里程计已经校准

            transCoord(// 将机器人的位置从里程计坐标系转换到场地坐标系
                locator.robotPoseToOdom.x, locator.robotPoseToOdom.y, locator.robotPoseToOdom.theta,// 转换前：里程计坐标系的机器人的位置和姿态
                locator.odomToField.x, locator.odomToField.y, locator.odomToField.theta,// 转换操作：里程计到场地坐标系的转换
                locator.robotPoseToField.x, locator.robotPoseToField.y, locator.robotPoseToField.theta);// 转换后：转换后的机器人的位置和姿态

            std::cout << "== Final RobotToFiled: (" 
                            << locator.robotPoseToField.x << ", " 
                            << locator.robotPoseToField.y << ", " 
                            << locator.robotPoseToField.theta << ")" << std::endl;

            posePub -> msg_.robot2field_x() = locator.robotPoseToField.x;// 将机器人的位置和姿态信息填充到发布的消息中
            posePub -> msg_.robot2field_y() = locator.robotPoseToField.y;
            posePub -> msg_.robot2field_theta() = locator.robotPoseToField.theta;
            posePub -> unlockAndPublish();// 解锁并发布消息

            if (is_display) {// 如果需要显示调试信息
                locator.display_board -> displayRobotPose(locator.robotPoseToField.x, locator.robotPoseToField.y, locator.robotPoseToField.theta);
            // 在显示板上显示机器人的位置和姿态信息
            }
            
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
