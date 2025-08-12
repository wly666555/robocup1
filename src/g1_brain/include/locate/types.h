#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>


using namespace std;

/* ------------------ Struct ------------------------*/

const int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
const int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
const int32_t ROBOT_SPORT_API_ID_EULER = 1007;
const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
const int32_t ROBOT_SPORT_API_ID_SIT = 1009;
const int32_t ROBOT_SPORT_API_ID_RISESIT = 1010;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;
const int32_t ROBOT_SPORT_API_ID_HELLO = 1016;
const int32_t ROBOT_SPORT_API_ID_STRETCH = 1017;
const int32_t ROBOT_SPORT_API_ID_CONTENT = 1020;
const int32_t ROBOT_SPORT_API_ID_DANCE1 = 1022;
const int32_t ROBOT_SPORT_API_ID_DANCE2 = 1023;
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;
const int32_t ROBOT_SPORT_API_ID_POSE = 1028;
const int32_t ROBOT_SPORT_API_ID_SCRAPE = 1029;
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1030;
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP = 1031;
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032;
const int32_t ROBOT_SPORT_API_ID_HEART = 1036;
const int32_t ROBOT_SPORT_API_ID_STATICWALK = 1061;
const int32_t ROBOT_SPORT_API_ID_TROTRUN = 1062;
const int32_t ROBOT_SPORT_API_ID_ECONOMICGAIT = 1063;
const int32_t ROBOT_SPORT_API_ID_LEFTFLIP = 2041;
const int32_t ROBOT_SPORT_API_ID_BACKFLIP = 2043;
const int32_t ROBOT_SPORT_API_ID_HANDSTAND = 2044;
const int32_t ROBOT_SPORT_API_ID_FREEWALK = 2045;
const int32_t ROBOT_SPORT_API_ID_FREEBOUND = 2046;
const int32_t ROBOT_SPORT_API_ID_FREEJUMP = 2047;
const int32_t ROBOT_SPORT_API_ID_FREEAVOID = 2048;
const int32_t ROBOT_SPORT_API_ID_CLASSICWALK = 2049;
const int32_t ROBOT_SPORT_API_ID_WALKUPRIGHT = 2050;
const int32_t ROBOT_SPORT_API_ID_CROSSSTEP = 2051;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_SET = 2054;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_GET = 2055;
const int32_t ROBOT_SPORT_API_ID_SWITCHAVOIDMODE = 2058;

#pragma pack(1)
struct PathPoint {
  float timeFromStart;
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vyaw;
};
#pragma pack()
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
struct GameObject
{
    // --- Obtained from the /detect message ---
    string label;              // What the object is identified as.
    BoundingBox boundingBox;   // The recognition box of the object in the camera, with the upper left corner as the origin, x increasing to the right and y increasing downward.
    Point2D precisePixelPoint; // The precise pixel point position of the object. Only ground landmark points have this data.
    double confidence;         // The confidence of the identification.
    Point posToRobot;          // The position of the object in the robot's body coordinate system. The position is 2D, ignoring the z value.

    // --- Calculated and obtained in the processDetectedObject function ---
    string info;                     // Used to store additional information. For example, for a goalpost object, it can store which goalpost it is.
    Point posToField;                // The position of the object in the field coordinate system. The position is 2D, ignoring the z value. x is forward and y is leftward.
    double range;                    // The straight-line distance from the object to the projection point of the robot's center on the field plane.
    double pitchToRobot, yawToRobot; // The pitch and yaw of the object relative to the front of the robot, in rad. Downward and leftward are positive.        // The time when the object was detected.
};


// 定位范围约束条件
struct PoseBox2D 
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double thetamin;
	double thetamax;
};

// 球场标志点
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


