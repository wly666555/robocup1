#pragma once

#include <string>
#include <mutex>
#include "locator.h"
#include "robot_interfaces/msg/imu_state.hpp"

using namespace std;

/**
 * The BrainData class records the data needed by the Brain during decision-making.
 * Currently, multi-threaded read/write issues are not considered, but this may be addressed in the future if necessary.
 */
class BrainData
{
public:

    /* ------------------------------------ Data Recording ------------------------------------ */

    // Robot position & velocity commands
    Pose2D robotPoseToOdom;  // The robot's Pose in the Odom coordinate system, updated via odomCallback
    Pose2D odomToField;      // The origin of the Odom coordinate system in the Field coordinate system, can be calibrated using known positions, e.g., by calibration at the start of the game
    Pose2D robotPoseToField; // The robot's current position and orientation in the field coordinate system. The field center is the origin, with the x-axis pointing towards the opponent's goal (forward), and the y-axis pointing to the left. The positive direction of theta is counterclockwise.

    // Head position, updated through lowStateCallback
    double headPitch; // The current head pitch, in radians. 0 is horizontal forward, positive is downward.
    double headYaw;   // The current head yaw, in radians. 0 is forward, positive is left.
    robot_interfaces::msg::IMUState cur_imu;
    // Ball
    bool ballDetected = false;    // Whether the camera has detected the ball
    GameObject ball;              // Records the ball's information, including position, bounding box, etc.
    GameObject opponents;      
    GameObject goalposts; 
    GameObject markings;     
    
    double robotBallAngleToField; // The angle between the robot's vector to the ball and the X-axis in the field coordinate system, (-PI, PI]

    <double> computeBallPosition(
        const RotMat<double>& rotMatPelvisToGlobal,
        double waist_yaw_q,
        double servo0_q,
        double servo1_q,
        const Vec3<double>& ball_position_in_cam);
};