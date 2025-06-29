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
    fd = field_size == "kid" ? FD_ADULTSIZE : FD_KIDSIZE;// 根据配置文件中的场地大小选择相应的场地尺寸
 
    pf_locator = std::make_shared<ParticleFilter>();// 创建一个粒子滤波器对象，用于定位机器人在场地上的位置。
    pf_locator -> init(fd, 3, 0.1, 0.5);// 初始化粒子滤波器，传入场地尺寸、最小标记点数量、残差容忍度和imu偏移参数。

    display_board = std::make_shared<DisplayBoard>();// 创建一个显示板对象，用于在图形界面上显示定位结果和其他信息。
    display_board->init(fd);// 初始化显示板，传入场地尺寸。
}

void Locator::detectProcessMarkings(const vector<GameObject> &markingObjs)// 处理标记L，T，X
{
    const double confidenceValve = 0.1;// 置信度阈值，低于该值的标记将被忽略

    markings.clear();// 清空之前的标记列表

    for (int i = 0; i < markingObjs.size(); i++)// 遍历所有检测到的标记对象
    {
        auto marking = markingObjs[i];// 获取当前标记对象

        if (marking.confidence < confidenceValve)// 如果置信度低于阈值，则跳过该标记
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)//如果标记LTX相对于机器人位姿的x坐标小于-0.5或大于10.0，则跳过该标记
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
        //将检测到的目标转换为GameObject对象
        gObj.label = result.class_name();

        gObj.boundingBox.xmin = result.box()[0];
        gObj.boundingBox.ymin = result.box()[1];
        gObj.boundingBox.xmax = result.box()[2];
        gObj.boundingBox.ymax = result.box()[3];
        gObj.confidence = result.score() * 100;

        //将目标物在 相机坐标系 中的位姿转换为Pose对象
        Pose pose = Pose(result.xyz()[0], result.xyz()[1], result.xyz()[2], 0, 0, 0);
        //Pose(const float &x, const float &y, const float &z,const float &roll, const float &pitch, const float &yaw);
        //解释：result.xyz()[]表示目标物在相机坐标系中的位置。0, 0, 0表示目标的rpy为0，单位为弧度。
        auto trans = pose.getTranslation();//获取目标在相机坐标系中的平移向量

        //坐标变换：相机(pose)→人(obj_pose)
        Pose obj_pose = p_eye2base * pose;
        //将目标物的Pose从相机坐标系转换到机器人坐标系，p_eye2base是相机到机器人的变换矩阵。
        auto obj_trans = obj_pose.getTranslation();//获取目标在机器人坐标系中的平移向量

        gObj.posToRobot.x = obj_trans[2];//目标物在机器人坐标系中的x坐标
        gObj.posToRobot.y = -obj_trans[0];//目标物在机器人坐标系中的y坐标，注意y轴方向是向左的，所以取负值

        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        //计算目标物到机器人的直线距离（勾股定理）
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        //计算目标物相对于机器人的偏航角，使用arctan函数计算y/x的反正切值，结果范围在[-π, π]之间。
        gObj.pitchToRobot = atan2(1.3, gObj.range); 
        //计算目标物相对于机器人的俯仰角，假设机器人高度为1.3米，使用arctan函数计算1.3/距离的反正切值。

        //坐标变换：人→场地
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);
            //将目标物在机器人坐标系中的位置转换到场地坐标系中，
            //robotPoseToField是机器人的位姿，gObj.posToField是目标物在场地坐标系中的位置。

        gameObjects.push_back(gObj);//将转换后的GameObject对象添加到gameObjects列表中
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
            markings.push_back(obj);//如果检测到的目标是标记点（L,T,X），则将其添加到markings列表中
        }
    }

    detectProcessMarkings(markings);//处理标记点，置信度低的或者位置不合理的标记点将被忽略（见上一个函数）
}


std::vector<FieldMarker> Locator::getMarkers()// 获取当前检测到的标记点列表
{
    std::vector<FieldMarker> res;
    for (size_t i = 0; i < markings.size(); i++){// 遍历所有标记
        auto label = markings[i].label;// 获取标记的标签
        auto x = markings[i].posToRobot.x;// 获取标记相对于机器人的x坐标
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;// 获取标记的置信度

        char markerType = ' ';// 初始化标记类型为一个空格
        if (label == "L")
            markerType = 'L';
        else if (label == "T")
            markerType = 'T';
        else if (label == "X")
            markerType = 'X';
        else if (label == "P")
            markerType = 'P';
            
        res.push_back(FieldMarker{markerType, x, y, confidence});// 将标记点信息添加到结果列表中
    }
    return res;// 返回标记点列表
}

void Locator::selfLocate() {

    auto markers = getMarkers();// 获取当前检测到的标记点列表
    if (markers.size() < 3) {
        return;
    }// 如果标记点数量少于3个，则无法进行定位

    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    
    std::string mode = config.ReadStringFromYaml("location_mode");// 读取定位模式，可能的值有 "enter_field", "face_forward", "center", "normal" 等。

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
            thetaMin = -M_PI / 2 - M_PI / 6;// 左侧进场，允许朝向在 -30° 到 -150° 之间
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;// 右侧进场，允许朝向在 30° 到 150° 之间
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
        int msec = msecsSince(lastSuccessfulLocalizeTime);// 计算自上次成功定位以来经过的毫秒数
        double maxDriftSpeed = 0.2; // 假设每秒最大偏差0.2米
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // 计算最大漂移距离：每秒最大偏差0.2米，乘以经过的秒数

        xMin = max(-0.5, robotPoseToField.x - maxDrift);// 计算x轴最小值，不能小于-0.5米或机器人当前位置减去最大漂移距离
        // 这里的-0.5是为了防止机器人在场地边界附近漂移过大，导致定位失败。
        xMax = min(fd.length / 2, robotPoseToField.x + maxDrift);// 计算x轴最大值，不能大于场地长度的一半或者机器人当前位置加上最大漂移距离
        yMin = max(-fd.width / 2, robotPoseToField.y - maxDrift);// 计算y轴最小值，不能小于场地宽度的一半
        yMax = min(fd.width / 2, robotPoseToField.y + maxDrift);
        thetaMin = robotPoseToField.theta - M_PI / 4;// 计算朝向范围，允许偏差 ±45°
        thetaMax = robotPoseToField.theta + M_PI / 4;
    } else {
        std::cout << "[ERROR]: Unsupported mode, " << mode << std::endl;
        return;
    }

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};// 创建约束框
    auto res = pf_locator -> locateRobot(markers, constraints);// 通过粒子滤波器定位机器人在场地上的位置，传入标记点和约束条件!!!
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
    //解释：该函数根据机器人在里程计坐标系中的位置和朝向（robotToOdom）以及机器人在场地坐标系中的位置和朝向（robotToField）来计算里程计到场地的转换关系（odomToField）。
    double x_or, y_or, theta_or; // or = odom to robot
    //坐标逆变换
    x_or = -cos(robotPoseToOdom.theta) * robotPoseToOdom.x - sin(robotPoseToOdom.theta) * robotPoseToOdom.y;
    y_or = sin(robotPoseToOdom.theta) * robotPoseToOdom.x - cos(robotPoseToOdom.theta) * robotPoseToOdom.y;
    theta_or = -robotPoseToOdom.theta;

    transCoord(x_or, y_or, theta_or,
                x, y, theta,
                odomToField.x, odomToField.y, odomToField.theta);

    // transform markers for display 解释：将标记点从机器人坐标系转换到场地坐标系，以便在显示板上正确显示。
    display_board->clearMarkers();
    for (auto &marking : markings) {
        transCoord(marking.posToRobot.x, marking.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            marking.posToField.x, marking.posToField.y, marking.posToField.z
        );// 将标记点从机器人坐标系转换到场地坐标系
        display_board->addMarker(marking.label, marking.posToField.x, marking.posToField.y);// 将转换后的标记点添加到显示板上
    }

}
        
