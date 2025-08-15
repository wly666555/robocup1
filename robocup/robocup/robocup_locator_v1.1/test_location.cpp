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

    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " [networkInterface] [config_file] [is_display]" << std::endl;
        return -1;
    }

    std::string env_cfg_path = "../config.yaml";
    if (argc >= 3) {
        env_cfg_path = argv[2];
    } 

    bool is_display = false;
    if (argc == 4 && strcmp(argv[3], "1") == 0) {
        is_display = true;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    // Create servo Subscriber
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>> servoState;
    servoState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/g1_comp_servo/state");
    servoState->msg_.states().resize(2);

    // Create wrist Subscriber
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>> lowState;
    lowState = std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");

    // Create Odometry Subscriber
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>> odomState;
    odomState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>>("rt/lf/odommodestate");

    // Create Detection Subscriber
    dds::domain::DomainParticipant participant(0);
    dds::topic::Topic<DetectionModule::DetectionResults> topic(participant, "detectionresults");
    dds::sub::Subscriber subscriber(participant);
    dds::sub::DataReader<DetectionModule::DetectionResults> reader(subscriber, topic);

    // Create Location Publisher
    std::unique_ptr<unitree::robot::RealTimePublisher<LocationModule::LocationResult>> posePub; 
    posePub = std::make_unique<unitree::robot::RealTimePublisher<LocationModule::LocationResult>>("rt/locationresults");

    // Load config file
    YamlParser config;
    try {
        config.setup(env_cfg_path.c_str());
    }
    catch(const std::exception& e)
    {
        std::cout << "[ERROR] Cannot find config file: " << env_cfg_path << std::endl;
        return 0;
    }

    Locator locator;
    locator.init(config);


    double odometry_factor = config.ReadFloatFromYaml("odometry", "scale_factor");
    double servo_pitch_compensation = config.ReadFloatFromYaml("servo", "pitch_compensation");
    double servo_yaw_compensation = config.ReadFloatFromYaml("servo", "yaw_compensation");
    double servo_height = config.ReadFloatFromYaml("servo", "height");

    while (true) {

        // Read odometry data
        locator.robotPoseToOdom.x = odomState->msg_.position()[0] * odometry_factor;
        locator.robotPoseToOdom.y = odomState->msg_.position()[1] * odometry_factor;
        locator.robotPoseToOdom.theta = odomState->msg_.imu_state().rpy()[2];

        std::cout << "Odometer information: (" 
                    << locator.robotPoseToOdom.x << ", "
                    << locator.robotPoseToOdom.y << ", "
                    << locator.robotPoseToOdom.theta << ")" << std::endl;

        // Read servo data
        double wrist_yaw_angle = rad2deg(lowState->msg_.motor_state()[JointIndex::kWaistYaw].q());
        double servo_yaw_angle = servoState -> msg_.states()[0].q();
        double servo_pitch_angle = servoState -> msg_.states()[1].q() + servo_pitch_compensation;
	    Pose p_eye2base = Pose(0, -servo_height, 0, deg2rad(servo_pitch_angle), -deg2rad(wrist_yaw_angle) - deg2rad(servo_yaw_angle), 0) ;
	
        std::cout << "Servo information: "
                 << "wrist_yaw_angle(" << wrist_yaw_angle << "), "
                 << "servo_yaw_angle(" << servo_yaw_angle << "), "
                 << "servo_pitch_angle(" << servo_pitch_angle << ")" << std::endl;

        // Read detection data
        auto samples = reader.take();
        for (const auto &sample : samples) {
            if (sample.info().valid()) {
                const auto& det_results = sample.data().results();
                std::cout << "===== Received " << det_results.size() << " detection results =====\n";
                if (det_results.size() > 0) {
                    locator.processDetections(det_results, p_eye2base);
                    locator.selfLocate();
                }   
            }
        }
        
        // Compute and send location results
        if (locator.odomCalibrated) {

            transCoord(
                locator.robotPoseToOdom.x, locator.robotPoseToOdom.y, locator.robotPoseToOdom.theta,
                locator.odomToField.x, locator.odomToField.y, locator.odomToField.theta,
                locator.robotPoseToField.x, locator.robotPoseToField.y, locator.robotPoseToField.theta);

            std::cout << "== Final RobotToFiled: (" 
                            << locator.robotPoseToField.x << ", " 
                            << locator.robotPoseToField.y << ", " 
                            << locator.robotPoseToField.theta << ")" << std::endl;

            posePub -> msg_.robot2field_x() = locator.robotPoseToField.x;
            posePub -> msg_.robot2field_y() = locator.robotPoseToField.y;
            posePub -> msg_.robot2field_theta() = locator.robotPoseToField.theta;
            posePub -> unlockAndPublish();

            if (is_display) {
                locator.display_board -> displayRobotPose(locator.robotPoseToField.x, locator.robotPoseToField.y, locator.robotPoseToField.theta);
            }
            
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
