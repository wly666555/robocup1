#pragma once

#include <string>
#include <mutex>
#include <vector>
#include "locator.h"
#include "robot_interfaces/msg/imu_state.hpp"

class BrainData
{
public:
    // Robot position & velocity commands
    Pose2D robotPoseToOdom;
    Pose2D odomToField;
    Pose2D robotPoseToField;

    double headPitch;
    double headYaw;
    robot_interfaces::msg::IMUState cur_imu;

    // Ball
    bool ballDetected = false;
    GameObject ball;
    std::vector<GameObject> opponents;
    std::vector<GameObject> goalposts;
    std::vector<GameObject> markings;

    double ballYawToPelvis;
    Vec2<double> ballPositionInPelvis;
    Vec2<double> ballPositionInField;
    RotMat<double> rotMatPelvisToGlobal, rotMatGlobalToPelvis;
    double ball_range_selected;
    double robotBallAngleToField;
    HomoMat2<double> homoMatPelvisToField;

    Vec3<double> computeBallPosition(const RotMat<double>& rotMatPelvisToGlobal,
                                     double waist_yaw_q,
                                     double servo0_q,
                                     double servo1_q,
                                     const Vec3<double>& ball_position_in_cam);
};
