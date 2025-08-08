#include "g1_brain/data_utils.hpp"
#include "g1_brain/math.h"
#include <cmath>

namespace g1_brain {

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



RobotPose robotToField(const RobotPose& poseToRobot, const RobotPose& robotPoseToField) {
    RobotPose out;
    // 期望的是：pose_field = (robotPoseToField) ⊕ (poseToRobot)
    g1_brain::math::transCoord(
        robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
        poseToRobot.x, poseToRobot.y, poseToRobot.theta,
        out.x, out.y, out.theta);
    out.theta = g1_brain::math::normalizeAngle(out.theta);
    return out;
}

RobotPose fieldToRobot(const RobotPose& poseToField, const RobotPose& robotPoseToField) {
    // 计算 field->robot 的等效变换：先对 robotPoseToField 求逆
    double xfr, yfr, thetafr;
    g1_brain::math::invertPose(robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
                               xfr, yfr, thetafr);

    RobotPose out;
    // pose_robot = (T_robot<-field) ⊕ (pose_field)
    g1_brain::math::transCoord(
        xfr, yfr, thetafr,
        poseToField.x, poseToField.y, poseToField.theta,
        out.x, out.y, out.theta);
    out.theta = g1_brain::math::normalizeAngle(out.theta);
    return out;
}

} // namespace g1_brain


