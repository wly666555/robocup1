#include "brain_data.h"
#include <cmath>
#include "locate/math_utils.h"
#include "locate/pose.h"




std::vector<FieldMarker> Locator::getMarkers()
{
    std::vector<FieldMarker> res;
    for (size_t i = 0; i < markings.size(); i++){
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

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
<double> BrainData::computeBallPosition(
    const RotMat<double>& rotMatPelvisToGlobal,
    double waist_yaw_q,
    double servo0_q,
    double servo1_q,
    const Vec3<double>& ball_position_in_cam)
{
    // 从IMU四元数获取旋转矩阵(如果未传入)
    RotMat
<double> actualRotMat = rotMatPelvisToGlobal;
    if(rotMatPelvisToGlobal.isZero()) {
        Quat
<double> quat;
        quat 
<< cur_imu.quaternion[0], cur_imu.quaternion[1], 
                cur_imu
.quaternion[2], cur_imu.quaternion[3];
        actualRotMat 
= quatToRotMat(quat);
    }

    // 分解RPY角并去除偏航分量
    Vec3
<double> _B2G_rpy = rotMatToRPY(actualRotMat);       
    RotMat
<double> rotMatPelvisToGlobal_no_yaw = rpyToRotMat(_B2G_rpy(0), _B2G_rpy(1), 0);

    // 构建完整的坐标变换链
    HomoMat
<double> homoMatPelvisToWorldAligned = homoMatrix(Vec3<double>(0.0, 0.0, 0.0), rotMatPelvisToGlobal_no_yaw);
    HomoMat
<double> homoMatTorsoToPelvis = homoMatrix(Vec3<double>(-0.0039635, 0.0, 0.044), rotz(waist_yaw_q));
    HomoMat
<double> homoMatHeadServoToTorso = homoMatrix(Vec3<double>(0.0039635, 0.0, -0.047), RotMat<double>::Identity());
    RotMat
<double> rotMatXl330ToHeadServo = roty(0.039968) * rotz(servo0_q);
    HomoMat
<double> homoMatXl330ToHeadServo = homoMatrix(Vec3<double>(0.030518, 0.0, 0.52486), rotMatXl330ToHeadServo);
    HomoMat
<double> homoMatD455ToXl330 = homoMatrix(Vec3<double>(0.0295, 0.0, 0.013), roty(servo1_q));
    RotMat
<double> rotMatCamToD455 = roty(0.6981) * roty(1.5707) * rotz(-1.5707);
    HomoMat
<double> homoMatCamToD455 = homoMatrix(Vec3<double>(0.04061, 0.01000, -0.02207), rotMatCamToD455);
    HomoMat
<double> homoMatBallToCam = homoMatrix(ball_position_in_cam, RotMat<double>::Identity());

    // 组合所有变换
    HomoMat
<double> homoMatBallToWorld = homoMatPelvisToWorldAligned * 
                                       homoMatTorsoToPelvis 
* 
                                       homoMatHeadServoToTorso 
*  
                                       homoMatXl330ToHeadServo 
* 
                                       homoMatD455ToXl330 
* 
                                       homoMatCamToD455 
* 
                                       homoMatBallToCam
;

    // 返回球在全局坐标系中的位置
    return Vec3<double>(
        homoMatBallToWorld(0,3),
        homoMatBallToWorld(1,3),
        homoMatBallToWorld(2,3)
    );
}


