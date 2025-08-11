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
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void BrainTree::init()
{
    BehaviorTreeFactory factory;
    // Action Nodes
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(playerDecision)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(SelfLocate)
    

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
    tree.tickWhileRunning(0ms);
    RCLCPP_INFO(rclcpp::get_logger("BrainTree"), "After tree.tickWhileRunning(0ms)");
}

// =================== 节点实现 ===================


BT::NodeStatus SelfLocate::tick() {

    // string mode = getInput<string>("mode").value();   //可以通过行为树输入
    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0; // 结束条件
    auto markers = brain->data->getMarkers();

    std::cout << "[DEBUG] markers.size(): " << markers.size() << std::endl;
    


    if (brain->config->location_mode == "enter_field")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = -brain->config->fieldDimensions.circleRadius;


        if (brain->config->playerStartPos == "left")
        {
            yMin = brain->config->fieldDimensions.width / 2;
            yMax = brain->config->fieldDimensions.width / 2 + 1.0;
        }
        else if (brain->config->playerStartPos == "right")
        {
            yMin = -brain->config->fieldDimensions.width / 2 - 1.0;
            yMax = -brain->config->fieldDimensions.width / 2;
        }

        if (brain->config->playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 6;
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (brain->config->playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;
            thetaMax = M_PI / 2 + M_PI / 6;
        }
    }
    else if (brain->config->location_mode ==  "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (brain->config->location_mode ==  "center" || (brain->config->location_mode ==  "normal" && !calibrated))
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 2;
        thetaMax = M_PI / 2;
    }
    else if (brain->config->location_mode ==  "normal" && calibrated)
    {
        int msec = msecsSince(lastSuccessfulLocalizeTime);
        double maxDriftSpeed = 0.2;
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = std::max(-brain->config->fieldDimensions.length / 2, data->robotPoseToField.x - maxDrift);
        xMax = std::min(brain->config->fieldDimensions.length / 2, data->robotPoseToField.x + maxDrift);
        yMin = std::max(-brain->config->fieldDimensions.width / 2, data->robotPoseToField.y - maxDrift);
        yMax = std::min(brain->config->fieldDimensions.width / 2, data->robotPoseToField.y + maxDrift);
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
    if(!brain->tree->get_Entry<bool>("ball_location_known"))
    {
        return BT::NodeStatus::SUCCESS; 
    }

    double s = 0.4;
    double r = 0.8;
    Vec2<double> vec_goal_ball_field;
    vec_goal_ball_field(0) = 4.5 - brain->data->ballPositionInField(0);
    vec_goal_ball_field(1) = 0 - brain->data->ballPositionInField(1);

    double angle_goal_ball_field = atan2(vec_goal_ball_field(1),vec_goal_ball_field(0));


    double deltaDir = toPInPI(angle_goal_ball_field - data->robotBallAngleToField);

    double dir = deltaDir > 0 ? -1.0 : 1.0;



    double vx = -s * dir * sin(data->ballYawToPelvis);
    double vy = s * dir * cos(data->ballYawToPelvis);
    vx = saturation(vx, Vec2<double>(-1.2,1.2));
    vy = saturation(vy, Vec2<double>(-1.2,1.2));

    double vyaw = (data->ballYawToPelvis-dir*s)/r;
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


BT::NodeStatus Chase::tick()
{
    if(!brain->tree->get_Entry<bool>("ball_location_known"))
    {
        client->Move(0,0,0);
        return BT::NodeStatus::SUCCESS; 
    }
    double vx_chase = brain->data->ballPositionInPelvis(0);
    double vy_chase = brain->data->ballPositionInPelvis(1);

    double linearFactor = 1 / (1 + exp(3 * (data->ballRange * fabs(data->ballYawToPelvis)) - 3));
    vx_chase *= linearFactor;
    vy_chase *= linearFactor;

    vx_chase = saturation(vx_chase, Vec2<double>(-1,1));
    vy_chase = saturation(vy_chase, Vec2<double>(-1,1));

    double vyaw_chase = data->ballYawToPelvis;
    vyaw_chase = saturation(vyaw_chase, Vec2<double>(-1,1));

    Vec2<double> vec_goal_ball_field;
    vec_goal_ball_field(0) = 4.5 - brain->data->ballPositionInField(0);
    vec_goal_ball_field(1) = 0 - brain->data->ballPositionInField(1);

    double angle_goal_ball_field = atan2(vec_goal_ball_field(1),vec_goal_ball_field(0));


    double deltaDir = angle_goal_ball_field - data->robotBallAngleToField;

    double dir = deltaDir > 0 ? -1.0 : 1.0;

    double s = 0.4;
    double r = 0.8;

    double vtheta = (data->ballYawToPelvis - dir * s) / r;

    double vx_adjust = -s * dir * sin(data->ballYawToPelvis);
    double vy_adjust = s * dir * cos(data->ballYawToPelvis);
    vy_adjust = saturation(vy_adjust, Vec2<double>(-1,1));

    double vyaw_adjust = vtheta;
    vyaw_adjust = saturation(vtheta, Vec2<double>(-1,1));

    double d_switch = 1.5;
    double w_chase = std::clamp(data->ballRange / d_switch, 0.0, 1.0);
    double w_orbit = 1.0 - w_chase;

    double vx = w_chase * vx_chase + w_orbit * vx_adjust;
    double vy = w_chase * vy_chase + w_orbit * vy_adjust;
    double vyaw = w_chase * vyaw_chase + w_orbit * vyaw_adjust;

    // brain->_interface->locoClient.Move(vx, vy, vyaw);
    return BT::NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const std::string& name, const NodeConfig& config, Brain* _brain)
    : SyncActionNode(name, config), brain(_brain)
{
    // 初始化预定义动作
    double lowPitch = 0.3;
    double highPitch = 0.3;
    double leftYaw = 0.3;
    double rightYaw = -0.3;

    predefinedPhases_ = {
        {Vec2f(lowPitch, leftYaw), 200},
        {Vec2f(lowPitch, 0.0), 500},
        {Vec2f(lowPitch, rightYaw), 500},
        {Vec2f(highPitch, rightYaw), 500},
        {Vec2f(highPitch, 0.0), 500},
        {Vec2f(highPitch, leftYaw), 500}
    };
}

BT::NodeStatus CamFindBall::tick()          //可以尝试时间控制
{
    constexpr float Y_SERVO_MIN = -M_PI / 3.0f;
    constexpr float Y_SERVO_MAX =  M_PI / 6.0f;

    if (brain->data->ballDetected)
        return BT::NodeStatus::SUCCESS;

    // 第一次进入，初始化插值器
    if (firstRun_) {
        Vec2f initAngle(
            brain->getMotorStates().states[0].q,
            brain->getMotorStates().states[1].q
        );
        interpolator_.reset(initAngle);
        for (const auto& [target, duration] : predefinedPhases_)
            interpolator_.addPhase(target, duration);
        firstRun_ = false;
    }

    Vec2f targetAngle;
    bool inProgress = interpolator_.interpolate(targetAngle); // 每tick推进一步

    // 舵机控制
    brain->getMotorCmds().states[0].mode = 1;
    brain->getMotorCmds().states[0].q = targetAngle(0);

    float limitedY = std::clamp(targetAngle(1), Y_SERVO_MIN, Y_SERVO_MAX);
    brain->getMotorCmds().states[1].mode = 1;
    brain->getMotorCmds().states[1].q = limitedY;
    brain->publishMotorCmds();

    // 如果插值器全部做完了，下次tick会重启
    if (!inProgress)
        firstRun_ = true;

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
