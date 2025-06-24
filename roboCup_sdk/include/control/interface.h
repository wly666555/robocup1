#ifndef INTERFACE_H
#define INTERFACE_H

#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/thread/thread.hpp>
#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include "common/types.h"
#include "common/mathTools.h"
#include "common/DetectionModule.hpp"
#include "common/LocationModule.hpp"
#include <cmath> 

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::g1;

enum JointIndex {
    // Left leg
    kLeftHipPitch = 0,
    kLeftHipRoll = 1,
    kLeftHipYaw = 2,
    kLeftKnee = 3,
    kLeftAnkle  = 4,
    kLeftAnkleRoll = 5,

    // Right leg
    kRightHipPitch = 6,
    kRightHipRoll = 7,
    kRightHipYaw = 8,
    kRightKnee = 9,
    kRightAnkle = 10,
    kRightAnkleRoll = 11,

    kWaistYaw = 12,
    kWaistRoll = 13,
    kWaistPitch = 14,

    // Left arm
    kLeftShoulderPitch = 15,
    kLeftShoulderRoll = 16,
    kLeftShoulderYaw = 17,
    kLeftElbowPitch = 18,
    kLeftElbowRoll = 19,
    
    // Right arm
    kRightShoulderPitch = 22,
    kRightShoulderRoll = 23,
    kRightShoulderYaw = 24,
    kRightElbowPitch = 25,
    kRightElbowRoll = 26,

    kNotUsedJoint = 29,
    kNotUsedJoint1 = 30,
    kNotUsedJoint2 = 31,
    kNotUsedJoint3 = 32,
    kNotUsedJoint4 = 33,
    kNotUsedJoint5 = 34
};


class PIDController {
public:
    PIDController(double Kp,double Kd)
        : Kp(Kp), Kd(Kd), prev_error(0.0){}

    double compute(double error, double dt) 
    {
        double P = Kp * error;
        double derivative = (dt > 0) ? (error - prev_error) / dt : 0;
        double D = Kd * derivative;

        prev_error = error;
        return P + D;
    }

private:
    double Kp, Kd;
    double prev_error;
};

class Interface
{
public:
    Interface();
    ~Interface();

    LocoClient locoClient;
    AudioClient audioClient;
    xRockerBtnDataStruct keyData;

    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>> servoCmd;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>> servoState;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>> lowState;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::IMUState_>> torsoImu;
    std::shared_ptr<unitree::robot::SubscriptionBase<DetectionModule::DetectionResults>> detection;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>> odomState;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>> armState;
    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::LowCmd_>> armCmd;
    std::shared_ptr<unitree::robot::SubscriptionBase<LocationModule::LocationResult>> locateResult;

    RotMat<double> rotMatPelvisToGlobal, rotMatGlobalToPelvis;
    bool ballDetected;
    double ballYawToPelvis;
    Vec2<double> ball_offset_fov;
    Vec2<double> ball_offset;
    double ball_range_selected;

    Vec2<double> ballPositionInPelvis;
    Vec2<double> ballPositionInField;

    Vec3<double> debug_rpy;

    // for test;
    HomoMat2<double> homoMatPelvisToField;
 
    bool calibration = false;

    bool trackDone = false;
    bool adjustDone = false;


private:
    void init();
    void lowStateHandle();
    ThreadPtr lowStateThreadPtr;
    double yaw_angle_add;
    double pitch_angle_add;
    
    void compute_ball_position(RotMat<double>rotMatPelvisToGlobal,double waist_yaw_q, double servo0_q, double servo1_q, Vec3<double> ball_position_in_cam);

    double ballDetected_counter;
};

#endif