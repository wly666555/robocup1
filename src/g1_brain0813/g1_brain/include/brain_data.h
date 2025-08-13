#pragma once

#include <string>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "locator.h"
#include "robot_interfaces/msg/imu_state.hpp"

#include "locate/math_types.h"


/**
 * BrainData 类，记录 Brain 在决策中需要用到的所在数据
 */

class BrainData
{
public:

    rclcpp::Time lastSuccessfulLocalizeTime;
    Locator locator;
    // Robot position & velocity commands
    Pose2D robotPoseToOdom;         // 机器人在 Odom 坐标系中的 Pose, 通过 odomCallback 更新数据
    Pose2D odomToField;             // Odom 坐标系原点在 Field 坐标系中的位置和方向.  可通过已知位置进行校准, 例如上场时根据上场点校准
    Pose2D robotPoseToField;        // 机器人当前在球场坐标系中的位置和方向. 球场中心为原点, x 轴指向对方球门(前方), y 轴指向左方. 逆时针为 theta 正方向.
    
    bool odomCalibrated = false;

    // 头部位置 通过 lowStateCallback 更新数据
    double headPitch;
    double headYaw;

    //IMU 数据
    robot_interfaces::msg::IMUState cur_imu;

    // 足球
    bool ballDetected = false;
    GameObject ball;
    double robotBallAngleToField;       // 机器人到球的向量, 在球场坐标系中与 X 轴的夹角, (-PI,PI]

    // 场上其它对象
    std::vector<GameObject> opponents;
    std::vector<GameObject> goalposts;
    std::vector<GameObject> markings;



    RotMat<double> rotMatPelvisToGlobal, rotMatGlobalToPelvis;

    HomoMat2<double> homoMatPelvisToField; 
    HomoMat<double> homoMatPelvisToWorldAligned;
    HomoMat<double> homoMatTorsoToPelvis;
    HomoMat<double> homoMatHeadServoToTorso;
    RotMat<double> rotMatXl330ToHeadServo;
    HomoMat<double> homoMatXl330ToHeadServo;
    HomoMat<double> homoMatD455ToXl330;
    RotMat<double> rotMatCamToD455;
    HomoMat<double> homoMatCamToD455;
    HomoMat<double> homoMatBallToCam;
    HomoMat<double> homoMatBallToWorldAligned;

    Vec3<double> computeBallPosition(const RotMat<double>& rotMatPelvisToGlobal,
                                    double waist_yaw_q,
                                    double servo0_q,
                                    double servo1_q,
                                    const Vec3<double>& ball_position_in_cam);

    vector<FieldMarker> getMarkers();   
    // 将一个 Pose 从 robot 坐标系转到 field 坐标系
    Pose2D robot2field(const Pose2D &poseToRobot);
    // 将一个 Pose 从 field 坐标系转到 robot 坐标系
    Pose2D field2robot(const Pose2D &poseToField);
};
