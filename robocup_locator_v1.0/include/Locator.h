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
        Pose2D robotPoseToOdom;  // The robot's Pose in the Odom coordinate system, updated via odomCallback
        Pose2D odomToField;      // The origin of the Odom coordinate system in the Field coordinate system, can be calibrated using known positions, e.g., by calibration at the start of the game
        Pose2D robotPoseToField; // The robot's current position and orientation in the field coordinate system. The field center is the origin, with the x-axis pointing towards the opponent's goal (forward), and the y-axis pointing to the left. The positive direction of theta is counterclockwise.
        
        // The locator object.
        std::shared_ptr<ParticleFilter> pf_locator;
        // The display debug board
        std::shared_ptr<DisplayBoard> display_board;

        void init(YamlParser _config);

        void processDetections(const std::vector<::DetectionModule::DetectionResult> &detection_results, const Pose &p_eye2base);
        
        void selfLocate();
        
        void calibrateOdom(double x, double y, double theta);
};