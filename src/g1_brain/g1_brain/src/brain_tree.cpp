#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include "brain_tree.h"
#include "brain_data.h"
#include "locate/yaml_parser.h"
#include "locator.h"
#include "brain.h"

// 注册节点时的宏（你可以用 lambda注册，见头文件说明）

void BrainTree::init()
{
    BehaviorTreeFactory factory;
    // factory.registerBehaviorTreeFromFile(config_.treeFilePath);
    data = std::make_shared<BrainData>();
    // Action Nodes
    factory.registerBuilder<SelfLocate>(
        "SelfLocate",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<SelfLocate>(name, config, this->brain, *(this->brain->config), this->data);
        }
    );
    factory.registerBuilder<Adjust>(
        "Adjust",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<Adjust>(name, config, this->brain, *(this->brain->config));
        }
    );
    factory.registerBuilder<Kick>(
        "Kick",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<Kick>(name, config, this->brain, *(this->brain->config));
        }
    );
    factory.registerBuilder<CamTrackBall>(
        "CamTrackBall",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CamTrackBall>(name, config, this->brain, *(this->brain->config));
        }
    );
    factory.registerBuilder<CamFindBall>(
        "CamFindBall",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CamFindBall>(name, config, this->brain, *(this->brain->config));
        }
    );
    factory.registerBuilder<robotTrackField>(
        "robotTrackField",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<robotTrackField>(name, config, this->brain, *(this->brain->config));
        }
    );
    factory.registerBuilder<PrintMsg>(
        "PrintMsg",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<PrintMsg>(name, config, this->brain, *(this->brain->config));
        }
    );
    factory.registerBuilder<playerDecision>(
        "playerDecision",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<playerDecision>(name, config, this->brain, *(this->brain->config));
        }
    );

    factory.registerBehaviorTreeFromFile(this->brain->config->treeFilePath);
    tree = factory.createTree("CamFindAndTrackBall");
    
    //init blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    // setEntry<std::string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<std::string>("decision", "");
    setEntry<std::string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

}

void BrainTree::tick()
{
    tree.tickRoot();
    RCLCPP_INFO(rclcpp::get_logger("BrainTree"), "After tree.tickRoot()");
}

// =================== 节点实现 ===================


BT::NodeStatus SelfLocate::tick() {
    if (!data) {
    std::cout << "[FATAL] data is nullptr!" << std::endl;
    return BT::NodeStatus::FAILURE;
    }
    if (!brain) {
        std::cout << "[FATAL] brain is nullptr!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    
    bool calibrated = data->odomCalibrated;
    brain->blackboard_->set("odom_calibrated", calibrated);
    auto markers = data->locator.getMarkers();
    std::cout << "[DEBUG] markers.size(): " << markers.size() << std::endl;
    for (const auto& m : markers) {
        std::cout << "[DEBUG] marker: type=" << m.type << " x=" << m.x << " y=" << m.y << " conf=" << m.confidence << std::endl;
    }
    if (markers.size() < 4) {
        std::cout << "[WARN] Not enough markers for localization!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    
    std::string mode = yamlparser.ReadStringFromYaml("location_mode");

    if (mode == "enter_field")
    {
        xMin = -brain->fd.length / 2;
        xMax = -brain->fd.circleRadius;

        std::string playerStartPos = yamlparser.ReadStringFromYaml("playerStartPos");

        if (playerStartPos == "left")
        {
            yMin = brain->fd.width / 2;
            yMax = brain->fd.width / 2 + 1.0;
        }
        else if (playerStartPos == "right")
        {
            yMin = -brain->fd.width / 2 - 1.0;
            yMax = -brain->fd.width / 2;
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
        xMin = -brain->fd.length / 2;
        xMax = brain->fd.length / 2;
        yMin = -brain->fd.width / 2;
        yMax = brain->fd.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "center" || (mode == "normal" && !calibrated))
    {
        xMin = -brain->fd.length / 2;
        xMax = brain->fd.length / 2;
        yMin = -brain->fd.width / 2;
        yMax = brain->fd.width / 2;
        thetaMin = -M_PI / 2;
        thetaMax = M_PI / 2;
    }
    else if (mode == "normal" && calibrated)
    {
        int msec = msecsSince(lastSuccessfulLocalizeTime);
        double maxDriftSpeed = 0.2;
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = std::max(-brain->fd.length / 2, data->robotPoseToField.x - maxDrift);
        xMax = std::min(brain->fd.length / 2, data->robotPoseToField.x + maxDrift);
        yMin = std::max(-brain->fd.width / 2, data->robotPoseToField.y - maxDrift);
        yMax = std::min(brain->fd.width / 2, data->robotPoseToField.y + maxDrift);
        thetaMin = data->robotPoseToField.theta - M_PI / 4;
        thetaMax = data->robotPoseToField.theta + M_PI / 4;
    } else {
        std::cout << "[ERROR]: Unsupported mode, " << mode << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    auto res = brain->locator.locateRobot(markers, constraints);

    std::cout << "locate result: res: " << std::to_string(res.code) << " time: " << std::to_string(res.msecs) << std::endl;

    if (res.success) {
        brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
        data->odomCalibrated = true;
        lastSuccessfulLocalizeTime = std::chrono::high_resolution_clock::now();
    }
    
    std::cout << "locate success: " << std::to_string(res.pose.x) << " " << std::to_string(res.pose.y) << " " << std::to_string(rad2deg(res.pose.theta)) << " Dur: " << std::to_string(res.msecs) << std::endl;

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Adjust::tick()
{
    double ballYawToPelvis = atan2(brain->data->ballPositionInPelvis(1), brain->data->ballPositionInPelvis(0));
    double p = 0.8;
    double vx = p * cos(ballYawToPelvis);
    double vy = p * sin(ballYawToPelvis);

    vx = saturation(vx, Vec2<double>(-1.2,1.2));
    vy = saturation(vy, Vec2<double>(-1.2,1.2));

    double vyaw = ballYawToPelvis;
    vyaw = saturation(vyaw, Vec2<double>(-1.2,1.2));

    // brain->_interface->locoClient.Move(vx,vy,0);

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus CamTrackBall::tick()
{
    if (!brain->data->ballDetected)
    {
        return BT::NodeStatus::SUCCESS;
    }

    // float fov_x = brain->_interface->ball_offset_fov(0);
    // float fov_y = brain->_interface->ball_offset_fov(1);

    // yaw_angle_add = fov_x * 0.6;
    // pitch_angle_add = fov_y * 0.6;

    float control_yaw = brain->getMotorStates().states[0].q; // -yaw_angle_add;
    float control_pitch = brain->getMotorStates().states[1].q; // +pitch_angle_add;

    brain->getMotorCmds().states[0].mode = 1;
    brain->getMotorCmds().states[0].q = control_yaw;
    brain->getMotorCmds().states[1].mode = 1;
    brain->getMotorCmds().states[1].q = control_pitch;
    brain->publishMotorCmds();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CamFindBall::tick()
{
    constexpr float Y_SERVO_MIN = -M_PI / 3.0f;
    constexpr float Y_SERVO_MAX =  M_PI / 6.0f;

    if (brain->data->ballDetected)
    {
        return BT::NodeStatus::SUCCESS;
    }

    if (firstRun)
    {
        Vec2f initAngle(
            brain->getMotorStates().states[0].q,
            brain->getMotorStates().states[1].q
        );

        interpolator.reset(initAngle);

        for (const auto& [target, duration] : predefinedPhases)
        {
            interpolator.addPhase(target, duration);
        }
        firstRun = false;
    }

    interpolator.interpolate(targetAngle);

    brain->getMotorCmds().states[0].mode = 1;
    brain->getMotorCmds().states[0].q = targetAngle(0);

    float limitedY = std::clamp(targetAngle(1), Y_SERVO_MIN, Y_SERVO_MAX);
    brain->getMotorCmds().states[1].mode = 1;
    brain->getMotorCmds().states[1].q = limitedY;

    brain->publishMotorCmds();

    client->Move(0.5, 0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus robotTrackField::tick()
{
    double ballYawToPelvis = atan2(brain->data->ballPositionInPelvis(1), brain->data->ballPositionInPelvis(0));
    double ballRange = sqrt(pow(brain->data->ballPositionInPelvis(0), 2) + pow(brain->data->ballPositionInPelvis(1), 2));
    
    double vx_chase = brain->data->ballPositionInPelvis(0);
    double vy_chase = brain->data->ballPositionInPelvis(1);

    double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYawToPelvis)) - 3));
    vx_chase *= linearFactor;
    vy_chase *= linearFactor;

    vx_chase = saturation(vx_chase, Vec2<double>(-1,1));
    vy_chase = saturation(vy_chase, Vec2<double>(-1,1));

    double vyaw_chase = ballYawToPelvis;
    vyaw_chase = saturation(vyaw_chase, Vec2<double>(-1,1));

    Vec2<double> ballPositionInField;
    ballPositionInField(0) = brain->data->homoMatPelvisToField(0,2) + brain->data->ballPositionInPelvis(0);
    ballPositionInField(1) = brain->data->homoMatPelvisToField(1,2) + brain->data->ballPositionInPelvis(1);

    Vec2<double> vec_goal_ball_field;
    vec_goal_ball_field(0) = 4.5 - brain->data->ballPositionInField(0);
    vec_goal_ball_field(1) = 0 - brain->data->ballPositionInField(1);

    double angle_goal_ball_field = atan2(vec_goal_ball_field(1),vec_goal_ball_field(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = ballPositionInField(0) - brain->data->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = ballPositionInField(1) - brain->data->homoMatPelvisToField(1,2);

    double angle_robot_ball_field = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double deltaDir = angle_goal_ball_field - angle_robot_ball_field;

    double dir = deltaDir > 0 ? -1.0 : 1.0;

    double s = 0.4;
    double r = 0.8;

    double vtheta = (ballYawToPelvis - dir * s) / r;

    double vx_adjust = -s * dir * sin(ballYawToPelvis);
    double vy_adjust = s * dir * cos(ballYawToPelvis);
    vy_adjust = saturation(vy_adjust, Vec2<double>(-1,1));

    double vyaw_adjust = vtheta;
    vyaw_adjust = saturation(vtheta, Vec2<double>(-1,1));

    double d_switch = 1.5;
    double w_chase = std::clamp(ballRange / d_switch, 0.0, 1.0);
    double w_orbit = 1.0 - w_chase;

    double vx = w_chase * vx_chase + w_orbit * vx_adjust;
    double vy = w_chase * vy_chase + w_orbit * vy_adjust;
    double vyaw = w_chase * vyaw_chase + w_orbit * vyaw_adjust;

    // brain->_interface->locoClient.Move(vx, vy, vyaw);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PrintMsg::tick()
{
    auto msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Kick::tick()
{
    Vec2<double> ballPositionInField;
    ballPositionInField(0) = brain->data->homoMatPelvisToField(0,2) + brain->data->ballPositionInPelvis(0);
    ballPositionInField(1) = brain->data->homoMatPelvisToField(1,2) + brain->data->ballPositionInPelvis(1);

    Vec2<double> leftGoalField;
    leftGoalField(0) = 4.5;
    leftGoalField(1) = 2.6 / 2;

    Vec2<double> rightGoalField;
    rightGoalField(0) = 4.5;
    rightGoalField(1) = -2.6 / 2;

    double margin = 0.1;

    Vec2<double> vecBallLeftGoalField;
    vecBallLeftGoalField(0) = leftGoalField(0) - ballPositionInField(0);
    vecBallLeftGoalField(1) = leftGoalField(1) - margin - ballPositionInField(1);
    double angleballLeftGoalField = atan2(vecBallLeftGoalField(1),vecBallLeftGoalField(0));

    Vec2<double> vecBallRightGoalField;
    vecBallRightGoalField(0) = rightGoalField(0) - ballPositionInField(0);
    vecBallRightGoalField(1) = rightGoalField(1) + margin - ballPositionInField(1);
    double angleballRightGoalField = atan2(vecBallRightGoalField(1),vecBallRightGoalField(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = ballPositionInField(0) - brain->data->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = ballPositionInField(1) - brain->data->homoMatPelvisToField(1,2);
    double angleRobotBallField = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    if(abs(angleRobotBallField)<abs(angleballLeftGoalField-angleballRightGoalField))
    {
        std::cout<<"[node::kick] The shooting angle looks good "<<std::endl;
    }
    else
    {   
        Vec2<double> goalField;
        goalField(0) = 4.5;
        goalField(1) = 0;

        std::cout<<"[node::kick] The shooting angle doesn't look good "<<std::endl;
        Vec2<double> vecBallGoalField;
        vecBallGoalField(0) = goalField(0) - ballPositionInField(0);
        vecBallGoalField(1) = goalField(1) - ballPositionInField(1);
        double angleBallGoalField = atan2(vecBallGoalField(1),vecBallGoalField(0));

        Vec2<double> vecPelvisBallField;
        vecPelvisBallField(0) = ballPositionInField(0) - brain->data->homoMatPelvisToField(0,2);
        vecPelvisBallField(1) = ballPositionInField(1) - brain->data->homoMatPelvisToField(1,2);
        double anglePlevisBallfield = atan2(vecPelvisBallField(1),vecPelvisBallField(0));
        double biasAngle = angleBallGoalField - anglePlevisBallfield;
        double vx =0, vy =0, vtheta = 0;
        double s = 0.4, r=0.8;
        double ball_yaw =atan2(brain->data->ballPositionInPelvis(1),brain->data->ballPositionInPelvis(0));
        vx = s*sin(ball_yaw);
        vy = -s*cos(ball_yaw);
        vtheta = (ball_yaw+s)/r;

        // brain->_interface->locoClient.Move(vx,vy,vtheta);
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus playerDecision::tick()
{
    std::string decision;
    if(brain->data->ballPositionInField(0)> 4.6||abs(brain->data->ballPositionInField(1))>3)
    {
        goalSignal = true;
    }
    else
    {
        goalSignal = false;
    }

    bool enableCamFindBallNode;
    if(!brain->data->ballDetected)
    {
        enableCamFindBallNode = true;
    }
    else 
    {
        enableCamFindBallNode = false;
    }

    Vec2<double> goalField;
    goalField(0) = 4.5;
    goalField(1) = 0;

    bool enableRobotTrackFieldNode;
    Vec2<double> vecBallGoalField;
    vecBallGoalField(0) = goalField(0) - brain->data->ballPositionInField(0);
    vecBallGoalField(1) = goalField(1) - brain->data->ballPositionInField(1);
    double angleBallGoalField = atan2(vecBallGoalField(1),vecBallGoalField(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = brain->data->ballPositionInField(0) - brain->data->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = brain->data->ballPositionInField(1) - brain->data->homoMatPelvisToField(1,2);
    double anglePlevisBallfield = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double biasAngle = angleBallGoalField - anglePlevisBallfield;
    if(goalSignal|| ((abs(brain->data->ballYawToPelvis)<=0.20) && (abs(brain->data->ballPositionInPelvis(0)) <= 0.55) && (abs(brain->data->ballPositionInPelvis(1)) <= 0.3)))
    {
        enableRobotTrackFieldNode = false;
    }
    else 
    {
        enableRobotTrackFieldNode = true;
    }

    if(enableCamFindBallNode)
    {
        decision = "camFindBall";
    }
    else if(enableRobotTrackFieldNode)
    {
        decision = "robotTrackField";
    }
    else if(!goalSignal)
    {
        decision = "kick";
    }
    else
    {
        decision = "stop";
        // goal_wly = true;
    }
    
    std::cout<<"debug decision: "<<std::endl;
    std::cout<<decision<<std::endl;

    setOutput("decision", decision);
    return BT::NodeStatus::SUCCESS;
}
