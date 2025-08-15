#include "brain_data.h"
#include <cmath>
#include "locate/math_utils.h"
#include "locate/pose.h"




std::vector<FieldMarker> BrainData::getMarkers()
{
    std::vector<FieldMarker> res;
    for (size_t i = 0; i < markings.size(); i++){
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;
        std::cout << "[DEBUG] i=" << i << std::endl;
        std::cout << "[DEBUG] label: " << markings[i].label << std::endl;
        std::cout << "[DEBUG] posToRobot: x=" << markings[i].posToRobot.x << " y=" << markings[i].posToRobot.y << std::endl;
        std::cout << "[DEBUG] confidence: " << markings[i].confidence << std::endl; 
        char markerType = ' ';
        if (label == "L")
            markerType = 'L';
        else if (label == "T")
            markerType = 'T';
        else if (label == "X")
            markerType = 'X';
        else if (label == "P")
            markerType = 'P';
            
        res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    return res;
}


Vec3<double> BrainData::computeBallPosition(
    const RotMat<double>& rotMatPelvisToGlobal,double waist_yaw_q,double headYaw,double headPitch,const Vec3<double>& ball_position_in_cam)
{
    // 从IMU四元数获取旋转矩阵(如果未传入)
    RotMat<double> actualRotMat = rotMatPelvisToGlobal;
    if (rotMatPelvisToGlobal.isZero()) {
        Quat<double> quat;
        quat << cur_imu.quaternion[0], cur_imu.quaternion[1],
                cur_imu.quaternion[2], cur_imu.quaternion[3];
        actualRotMat = quatToRotMat(quat);
    }

    // 分解RPY角并去除偏航分量
    Vec3<double> _B2G_rpy = rotMatToRPY(actualRotMat);
    RotMat<double> rotMatPelvisToGlobal_no_yaw = rpyToRotMat(_B2G_rpy(0), _B2G_rpy(1), 0);

    // 构建完整的坐标变换链
    // 1）pelvis到全局坐标变换，T形，其实是场地定位
    homoMatPelvisToWorldAligned = homoMatrix(Vec3<double>(0.0, 0.0, 0.0), rotMatPelvisToGlobal_no_yaw);
    // 2）torso到pelvis，实际是腰部关节变换，带yaw旋转
    homoMatTorsoToPelvis = homoMatrix(Vec3<double>(-0.0039635, 0.0, 0.044), rotz(waist_yaw_q));
    // 3）头舵机到torso，连接搭建，通常这个固定
    homoMatHeadServoToTorso = homoMatrix(Vec3<double>(0.0039635, 0.0, -0.047), RotMat<double>::Identity());
    // 4）Xl330舵机到head servo（脖子旋转+mechanical偏置），连乘pitch/yaw
    rotMatXl330ToHeadServo = roty(0.039968) * rotz(headYaw);
    homoMatXl330ToHeadServo = homoMatrix(Vec3<double>(0.030518, 0.0, 0.52486), rotMatXl330ToHeadServo);
    // 5）D455摄像头到Xl330舵机，考虑摄像头上下转动舵机角
    homoMatD455ToXl330 = homoMatrix(Vec3<double>(0.0295, 0.0, 0.013), roty(headPitch));
    // 6）相机内部，连接自身与外层结构体
    rotMatCamToD455 = roty(0.6981) * roty(1.5707) * rotz(-1.5707);
    homoMatCamToD455 = homoMatrix(Vec3<double>(0.04061, 0.01000, -0.02207), rotMatCamToD455);
    // 7）球从相机系到相机原点的位移，笛卡尔坐标
    homoMatBallToCam = homoMatrix(ball_position_in_cam, RotMat<double>::Identity());
    

    // 组合所有变换
    HomoMat<double> homoMatBallToWorldAligned =
        homoMatPelvisToWorldAligned *
        homoMatTorsoToPelvis *
        homoMatHeadServoToTorso *
        homoMatXl330ToHeadServo *
        homoMatD455ToXl330 *
        homoMatCamToD455 *
        homoMatBallToCam;

    // 返回球在全局坐标系中的位置
    return Vec3<double>(
        homoMatBallToWorldAligned(0, 3),
        homoMatBallToWorldAligned(1, 3),
        homoMatBallToWorldAligned(2, 3)
    );
}


Pose2D BrainData::robot2field(const Pose2D &poseToRobot)
{
    Pose2D poseToField;
    transCoord(
        poseToRobot.x, poseToRobot.y, poseToRobot.theta,
        robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
        poseToField.x, poseToField.y, poseToField.theta);
    poseToField.theta = toPInPI(poseToField.theta);
    return poseToField;
}

Pose2D BrainData::field2robot(const Pose2D &poseToField)
{
    Pose2D poseToRobot;
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(robotPoseToField.theta) * robotPoseToField.x - cos(robotPoseToField.theta) * robotPoseToField.y;
    xfr = -cos(robotPoseToField.theta) * robotPoseToField.x - sin(robotPoseToField.theta) * robotPoseToField.y;
    thetafr = -robotPoseToField.theta;
    transCoord(
        poseToField.x, poseToField.y, poseToField.theta,
        xfr, yfr, thetafr,
        poseToRobot.x, poseToRobot.y, poseToRobot.theta);
    return poseToRobot;
}