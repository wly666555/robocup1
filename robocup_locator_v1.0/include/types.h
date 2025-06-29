#pragma once
#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>


using namespace std;

/* ------------------ Struct ------------------------*/

// Pitch dimension information
struct FieldDimensions
{
    double length;            // The length of the pitch.
    double width;             // The width of the pitch.
    double penaltyDist;       // The straight-line distance from the penalty spot to the bottom line.
    double goalWidth;         // The width of the goal.
    double circleRadius;      // The radius of the center circle.
    double penaltyAreaLength; // The length of the penalty area.
    double penaltyAreaWidth;  // The width of the penalty area.
    double goalAreaLength;    // The length of the goal area.
    double goalAreaWidth;     // The width of the goal area.
                              // Note: The penalty area is larger than the goal area; the actual lengths and widths of the penalty area and the goal area are smaller. This naming is to be consistent with the competition rules.
};
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};

// Pose2D, used to record a point on a plane and its orientation
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // rad, counterclockwise is positive starting from the positive direction of the x-axis.
};

// Point, used to record a three-dimensional point
struct Point
{
    double x;
    double y;
    double z;
};

// Point2D, used to record a two-dimensional point
struct Point2D
{
    double x;
    double y;
};

// BoundingBox
struct BoundingBox
{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
};

/// GameObject, used to store the information of important entities in the game, such as Ball, Goalpost, etc. Compared with the detection::DetectedObject in the /detect message, it has more abundant information.
// GameObject结构体用于存储游戏中重要实体的信息，如球、球门柱等。与/detect消息中的detection::DetectedObject相比，它包含了更丰富的信息。
struct GameObject
{
    // --- Obtained from the /detect message ---//该部分用于存储从/detect消息中获取的目标物信息。
    string label;              // What the object is identified as.目标物名字
    BoundingBox boundingBox;   // The recognition box of the object in the camera, with the upper left corner as the origin, x increasing to the right and y increasing downward.
    //boundingBox是一个矩形框，用于表示目标物在相机图像中的位置。它的左上角是原点，x轴向右增加，y轴向下增加。
    Point2D precisePixelPoint; // The precise pixel point position of the object. Only ground landmark points have this data.
    //解释：precisePixelPoint是目标物在相机图像中的精确像素点位置。只有地面上的地标点才会有这个数据。
    double confidence;         // The confidence of the identification. 解释：识别目标物的置信度，范围为0-1，值越大表示识别的越准确。
    Point posToRobot;          // The position of the object in the robot's body coordinate system. The position is 2D, ignoring the z value.
    //解释：posToRobot是目标物在机器人身体坐标系中的位置。这个位置是二维的，忽略了z值。x轴表示前进方向，y轴表示左侧方向。

    // --- Calculated and obtained in the processDetectedObject function ---//解释 ：该部分用于在processDetectedObject函数中计算并获取的目标物信息。
    string info;                     // Used to store additional information. For example, for a goalpost object, it can store which goalpost it is.
    //解释：info用于存储附加信息。例如，对于球门柱对象，可以存储它是哪个球门柱的信息。
    Point posToField;                // The position of the object in the field coordinate system. The position is 2D, ignoring the z value. x is forward and y is leftward.
    //解释：posToField是目标物在场地坐标系中的位置。这个位置是二维的，忽略了z值。x轴表示前进方向，y轴表示左侧方向。
    double range;                    // The straight-line distance from the object to the projection point of the robot's center on the field plane.
    //解释：range是目标物到机器人中心在场地平面上的投影点的直线距离。它表示目标物与机器人之间的距离。
    double pitchToRobot, yawToRobot; // The pitch and yaw of the object relative to the front of the robot, in rad. Downward and leftward are positive.        // The time when the object was detected.
    //解释：pitchToRobot和yawToRobot是目标物相对于机器人前方的俯仰角和偏航角，单位为弧度。向下和向左为正方向。当目标物被检测到时，这两个角度的值会被计算出来。
};

struct PoseBox2D
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double thetamin;
	double thetamax;
};

struct FieldMarker
{
	char type;			// L|T|X|P, representing different types of landmark points, where P represents the penalty mark.
	double x, y; // The position of the landmark point (in meters).
	double confidence;	// The recognition confidence level.
};


enum JointIndex {
    // Left leg
    kLeftHipYaw = 0,
    kLeftHipPitch = 1,
    kLeftHipRoll = 2,
    kLeftKnee = 3,
    kLeftAnkle = 4,
    kLeftAnkleRoll = 5,
    // Right leg
    kRightHipYaw = 6,
    kRightHipPitch = 7,
    kRightHipRoll = 8,
    kRightKnee = 9,
    kRightAnkle = 10,
    kRightAnkleRoll = 11,
  
    kWaistYaw = 12,
  
    // Left arm
    kLeftShoulderPitch = 13,
    kLeftShoulderRoll = 14,
    kLeftShoulderYaw = 15,
    kLeftElbow = 16,
    kLeftWristRoll = 17,
    kLeftWristPitch = 18,
    kLeftWristYaw = 19,
    // Right arm
    kRightShoulderPitch = 20,
    kRightShoulderRoll = 21,
    kRightShoulderYaw = 22,
    kRightElbow = 23,
    kRightWristRoll = 24,
    kRightWristPitch = 25,
    kRightWristYaw = 26,
  
    kNotUsedJoint = 27,
    kNotUsedJoint1 = 28,
    kNotUsedJoint2 = 29,
    kNotUsedJoint3 = 30,
    kNotUsedJoint4 = 31,
    kNotUsedJoint5 = 32,
    kNotUsedJoint6 = 33,
    kNotUsedJoint7 = 34
};
