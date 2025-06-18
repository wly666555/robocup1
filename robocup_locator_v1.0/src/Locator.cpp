#include <iostream>
#include <vector>
#include <chrono>

#include "Locator.h"
#include "types.h"
#include "display.h"
#include "misc.h"

void Locator::init(YamlParser _config) {

    config = _config;

    std::string field_size = config.ReadStringFromYaml("field_size");
    fd = field_size == "kid" ? FD_ADULTSIZE : FD_KIDSIZE;

    pf_locator = std::make_shared<ParticleFilter>();
    pf_locator -> init(fd, 3, 0.1, 0.5);

    display_board = std::make_shared<DisplayBoard>();
    display_board->init(fd);
}

void Locator::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    const double confidenceValve = 0.1;

    markings.clear();

    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        if (marking.confidence < confidenceValve)
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)
            continue;

        markings.push_back(marking);
    }
}


void Locator::processDetections(const std::vector<::DetectionModule::DetectionResult> &detection_results, const Pose &p_eye2base) {



    std::vector<GameObject> gameObjects;
    for (const auto &result : detection_results) {

        // cout<<"result.class_name(): "<<result.class_name()<<" result.xyz()[0]: "<<result.xyz()[0]<<" result.xyz()[1]: "<<result.xyz()[1]<<endl;

        GameObject gObj;

        gObj.label = result.class_name();

        gObj.boundingBox.xmin = result.box()[0];
        gObj.boundingBox.ymin = result.box()[1];
        gObj.boundingBox.xmax = result.box()[2];
        gObj.boundingBox.ymax = result.box()[3];
        gObj.confidence = result.score() * 100;

        Pose pose = Pose(result.xyz()[0], result.xyz()[1], result.xyz()[2], 0, 0, 0);
        auto trans = pose.getTranslation();

        Pose obj_pose = p_eye2base * pose;
        auto obj_trans = obj_pose.getTranslation();

        gObj.posToRobot.x = obj_trans[2];
        gObj.posToRobot.y = -obj_trans[0];

        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(1.3, gObj.range);

        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);

        gameObjects.push_back(gObj);
    }

    std::vector<GameObject> balls, goalPosts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);
        if (obj.label == "Person")
            persons.push_back(obj);
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "L" || obj.label == "T" || obj.label == "X") {
            // 目前仅识别L,T,X
            markings.push_back(obj);
        }
    }

    detectProcessMarkings(markings);
}


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

void Locator::selfLocate() {

    auto markers = getMarkers();
    if (markers.size() < 3) {
        return;
    }

    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    
    std::string mode = config.ReadStringFromYaml("location_mode");

    if (mode == "enter_field")
    {

        xMin = -fd.length / 2;
        xMax = -fd.circleRadius;

        std::string playerStartPos = config.ReadStringFromYaml("playerStartPos");

        if (playerStartPos == "left")
        {
            yMin = fd.width / 2;
            yMax = fd.width / 2 + 1.0;
        }
        else if (playerStartPos == "right")
        {
            yMin = -fd.width / 2 - 1.0;
            yMax = -fd.width / 2;
        }

        if (playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 6;
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;
            thetaMax = M_PI / 2 + M_PI / 6;
        }
    }
    else if (mode == "face_forward")
    {
        xMin = -fd.length / 2;
        xMax = fd.length / 2;
        yMin = -fd.width / 2;
        yMax = fd.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "center" || (mode == "normal" && !odomCalibrated))
    {
        xMin = -0.5; // TODO: 目前只测试半场
        xMax = fd.length / 2;
        yMin = -fd.width / 2;
        yMax = fd.width / 2;
        thetaMin = -M_PI / 2;
        thetaMax = M_PI / 2;
    }
    else if (mode == "normal" && odomCalibrated)
    {
        int msec = msecsSince(lastSuccessfulLocalizeTime);
        double maxDriftSpeed = 0.2; // 假设每秒最大偏差0.2米
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = max(-0.5, robotPoseToField.x - maxDrift);
        xMax = min(fd.length / 2, robotPoseToField.x + maxDrift);
        yMin = max(-fd.width / 2, robotPoseToField.y - maxDrift);
        yMax = min(fd.width / 2, robotPoseToField.y + maxDrift);
        thetaMin = robotPoseToField.theta - M_PI / 4;
        thetaMax = robotPoseToField.theta + M_PI / 4;
    } else {
        std::cout << "[ERROR]: Unsupported mode, " << mode << std::endl;
        return;
    }

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    auto res = pf_locator -> locateRobot(markers, constraints);

    // 0: Success
    // 1: Failure to generate new particles (quantity is 0)
    // 2: The residual error after convergence is unreasonable
    // 3: Not converged
    // 4: The number of Markers is insufficient
    // 5: The probabilities of all particles are too low
    std::cout << "locate result: res: " << to_string(res.code) << " time: " << to_string(res.msecs) << std::endl;

    if (res.success) {
        calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
        odomCalibrated = true;
        lastSuccessfulLocalizeTime = std::chrono::high_resolution_clock::now();
    }
    
    std::cout << "locate success: " << to_string(res.pose.x) << " " << to_string(res.pose.y) << " " + to_string(rad2deg(res.pose.theta)) << " Dur: " << to_string(res.msecs) << std::endl;
}

void Locator::calibrateOdom(double x, double y, double theta)
{
    // Calculate odomToField according to robotToOdom(by odometry) and robotToField(by locator)
    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(robotPoseToOdom.theta) * robotPoseToOdom.x - sin(robotPoseToOdom.theta) * robotPoseToOdom.y;
    y_or = sin(robotPoseToOdom.theta) * robotPoseToOdom.x - cos(robotPoseToOdom.theta) * robotPoseToOdom.y;
    theta_or = -robotPoseToOdom.theta;

    transCoord(x_or, y_or, theta_or,
                x, y, theta,
                odomToField.x, odomToField.y, odomToField.theta);

    // transform markers for display
    display_board->clearMarkers();
    for (auto &marking : markings) {
        transCoord(marking.posToRobot.x, marking.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            marking.posToField.x, marking.posToField.y, marking.posToField.z
        );
        display_board->addMarker(marking.label, marking.posToField.x, marking.posToField.y);
    }

}
        
