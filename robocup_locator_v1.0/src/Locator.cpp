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

        if (marking.confidence < confidenceValve)// 如果置信度低于阈值，则跳过该标记
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)//如果标记的x坐标小于-0.5或大于10.0，则跳过该标记
            continue;

        markings.push_back(marking);// 将标记添加到标记列表中
    }
}


void Locator::processDetections(const std::vector<::DetectionModule::DetectionResult> &detection_results, const Pose &p_eye2base) {
 // 该函数处理检测结果，将检测到的目标转换为GameObject对象，并计算其在机器人的坐标系和场地坐标系中的位置。


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
            balls.push_back(obj);//如果检测到的目标是足球，则将其添加到balls列表中
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);//如果检测到的目标是球门柱，则将其添加到goalPosts列表中
        if (obj.label == "Person")
            persons.push_back(obj);//如果检测到的目标是队员，则将其添加到persons列表中？
        if (obj.label == "Opponent")
            robots.push_back(obj);//如果检测到的目标是对方队员，则将其添加到robots列表中
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
    }// 如果标记点数量少于3个，则无法进行定位

    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    
    std::string mode = config.ReadStringFromYaml("location_mode");

    if (mode == "enter_field")//进场模式，约束机器人只能在场地一侧的特定区域
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
    else if (mode == "face_forward")//正对球场：允许在整个场地范围内，朝向在 ±45°
    {
        xMin = -fd.length / 2;
        xMax = fd.length / 2;
        yMin = -fd.width / 2;
        yMax = fd.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "center" || (mode == "normal" && !odomCalibrated))//未校准：允许在半场范围，朝向在 ±90°。
    {
        xMin = -0.5; // TODO: 目前只测试半场
        xMax = fd.length / 2;
        yMin = -fd.width / 2;
        yMax = fd.width / 2;
        thetaMin = -M_PI / 2;
        thetaMax = M_PI / 2;
    }
    else if (mode == "normal" && odomCalibrated)//允许在上次定位结果附近一定范围内，范围随时间漂移增大，防止误差积累。
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
    auto res = pf_locator -> locateRobot(markers, constraints);// 通过粒子滤波器定位机器人在场地上的位置，传入标记点和约束条件。
    //粒子滤波会在约束范围内生成大量假设位姿（粒子），通过与实际观测到的标记进行匹配，最终收敛到最可能的机器人位姿。
    // 0: Success
    // 1: Failure to generate new particles (quantity is 0)粒子生成失败
    // 2: The residual error after convergence is unreasonable 收敛后残差过大
    // 3: Not converged 未收敛
    // 4: The number of Markers is insufficient  标记点数量不足
    // 5: The probabilities of all particles are too low 所有粒子概率过低
    std::cout << "locate result: res: " << to_string(res.code) << " time: " << to_string(res.msecs) << std::endl;

    if (res.success) {
        calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);// 如果定位成功，则调用calibrateOdom函数进行里程计校准。
        odomCalibrated = true;
        lastSuccessfulLocalizeTime = std::chrono::high_resolution_clock::now();// 更新上次成功定位的时间戳。
    }
    
    //打印定位得到的场地坐标（x, y, theta，theta已转为角度）和本次定位耗时
    std::cout << "locate success: " << to_string(res.pose.x) << " " << to_string(res.pose.y) << " " + to_string(rad2deg(res.pose.theta)) << " Dur: " << to_string(res.msecs) << std::endl;
}

void Locator::calibrateOdom(double x, double y, double theta)// 该函数用于校准里程计，将机器人在场地坐标系中的位置和朝向转换为里程计坐标系中的位置和朝向。
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
        
