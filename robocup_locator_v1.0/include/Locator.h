#pragma once

#include <iostream>
#include <vector>

#include "DetectionModule.hpp"
#include "types.h"
#include "ParticleFilter.h"
#include "display.h"
#include "pose.h"
#include "yaml_parser.h"

class Locator {
    private:

        FieldDimensions fd;
        std::chrono::high_resolution_clock::time_point  lastSuccessfulLocalizeTime;

        // Other objects on the field
        vector<GameObject> opponents = {}; // Records information about opponent players, including position, bounding box, etc.
        vector<GameObject> goalposts = {}; // Records information about goalposts, including position, bounding box, etc.
        vector<GameObject> markings = {};  // Records information about field markings and intersections

        void detectProcessBalls(const std::vector<GameObject> &ballObjs);

        void detectProcessMarkings(const vector<GameObject> &markingObjs);
    
        std::vector<FieldMarker> getMarkers();

    public:

        YamlParser config;

        bool odomCalibrated = false;

        // Robot position & velocity commands
        //Pose2D是用于表示二维平面上位置和朝向的数据结构
        Pose2D robotPoseToOdom;  // The robot's Pose in the Odom coordinate system, updated via odomCallback
        //机器人在“里程计坐标系”下的位姿（位置和朝向），通常通过里程计回调函数实时更新。这个坐标系随机器人移动而变化，主要反映机器人自身的运动轨迹。
        
        Pose2D odomToField;      // The origin of the Odom coordinate system in the Field coordinate system, can be calibrated using known positions, e.g., by calibration at the start of the game
        //里程计坐标系的原点在“场地坐标系”下的位置，可以通过已知位置进行校准，例如在比赛开始时进行校准。这个变量表示机器人在整个场地中的绝对位置和朝向。

        Pose2D robotPoseToField; // The robot's current position and orientation in the field coordinate system. The field center is the origin, with the x-axis pointing towards the opponent's goal (forward), and the y-axis pointing to the left. The positive direction of theta is counterclockwise.
        //机器人在“场地坐标系”下的当前位姿。场地坐标系的原点设在足球场中心，x轴指向对方球门（前向），y轴指向左侧，theta（朝向）逆时针为正。这个变量表示机器人在整个场地中的绝对位置和朝向。
       
        // The locator object.
        std::shared_ptr<ParticleFilter> pf_locator;
        // The display debug board
        std::shared_ptr<DisplayBoard> display_board;

        void init(YamlParser _config);

        void processDetections(const std::vector<::DetectionModule::DetectionResult> &detection_results, const Pose &p_eye2base);
        
        void selfLocate();
        
        void calibrateOdom(double x, double y, double theta);
};