#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include "brain_tree.h"
#include "locate/yaml_parser.h"
#include "brain.h"

// 注册节点时的宏（你可以用 lambda注册，见头文件说明）
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfiguration &config) { return make_unique<Name>(name, config, brain); });


void BrainTree::init()
{
    BehaviorTreeFactory factory;
    // Action Nodes
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(SetVelocity)
    

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
    RCLCPP_INFO(rclcpp::get_logger("BrainTree"), "tree.tickRoot();");
}

// =================== 节点实现 ===================


BT::NodeStatus SelfLocate::tick() {

    string mode = getInput<string>("mode").value();   //可以通过行为树输入
    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0; // 结束条件
    auto markers = brain->data->getMarkers();

    std::cout << "[DEBUG] markers.size(): " << markers.size() << std::endl;
    


    if (mode == "enter_field")
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
    else if (mode ==  "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode ==  "center" || (brain->config->location_mode ==  "normal" && !"odom_calibrated"))
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 2;
        thetaMax = M_PI / 2;
    }
    else if (mode ==  "normal" && "odom_calibrated")
    {
        int msec = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
        double maxDriftSpeed = 0.2;
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = std::max(-brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = std::min(brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = std::max(-brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = std::min(brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 4;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 4;
    } else {
        std::cout << "[ERROR]: Unsupported mode, " << brain->config->location_mode << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    auto res = brain->locator->locateRobot(markers, constraints);

    std::cout << "locate result: res: " << std::to_string(res.code) << " time: " << std::to_string(res.msecs) << std::endl;
    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.
    
    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);

    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    
    std::cout << "locate success: " << std::to_string(res.pose.x) << " " << std::to_string(res.pose.y) << " " << std::to_string(rad2deg(res.pose.theta)) << " Dur: " << std::to_string(res.msecs) << std::endl;

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Adjust::tick()
{
    if(!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return BT::NodeStatus::SUCCESS; 
    }

    double vx = 0, vy = 0, vtheta = 0;
    double vxLimit = 1.0;
    double vyLimit = 0.8;
    double vthetaLimit = 1.0;


    double kickDir = atan2(-brain->data->ballPositionInField[1], brain->config->fieldDimensions.length / 2 - brain->data->ballPositionInField[0]);
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double dir = deltaDir > 0 ? -1.0 : 1.0;
    double ballRange = brain->data->ballRange;
    double ballYaw = brain->data->ballYawToPelvis;


    double s = 0.4;
    double r = 0.8;

    vx = -s * dir * sin(ballYaw);
    vy = s * dir * cos(ballYaw);
    vtheta = (ballYaw - dir * s) / r;


    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    // brain->client->Move(vx,vy,vtheta);

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus CamTrackBall::tick()
{
    double pitch, yaw;
    if (!brain->data->ballDetected)
    {
        pitch = brain->data->ballPitchToPelvis;
        yaw = brain->data->ballYawToPelvis;
    }
    else
    {
        //追踪逻辑

    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;

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

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->move(x, y, theta);
    return NodeStatus::SUCCESS;
}

BT::NodeStatus Chase::tick()
{
    if(!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->Move(0,0,0);
        return BT::NodeStatus::SUCCESS; 
    }

    double dist;
    getInput("dist", dist);


    double vxLimit = 1.0;
    double vyLimit = 0.8;
    double vthetaLimit = 1.0;


    double ballRange = brain->data->ballRange;
    double ballYaw = brain->data->ballYawToPelvis;

    Pose2D target_f, target_r;
    if (brain->data->robotPoseToField.x - brain->data->ballPositionInField[0] > (_state == "chase" ? 1.0 : 0.0))
    { // circle back
        _state = "circle_back";
        // 目标 x 坐标
        target_f.x = brain->data->ballPositionInField[0] - dist;

        // 目标 y 坐标. 即决策从哪边绕, 并防止震荡
        if (brain->data->robotPoseToField.y > brain->data->ballPositionInField[1] - _dir)
            _dir = 1.0;
        else
            _dir = -1.0;

        target_f.y = brain->data->ball.ballPositionInField[1] + _dir * dist;
    }
    else
    { // chase
        _state = "chase";
        target_f.x = brain->data->ballPositionInField[0] - dist;
        target_f.y = brain->data->ballPositionInField[1];
    }

    target_r = brain->data->field2robot(target_f);
    double vx = target_r.x;
    double vy = target_r.y;
    double vtheta = ballYaw * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYaw)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;


    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    
    // brain->client.Move(vx, vy, vtheta);
    return BT::NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const std::string& name, const NodeConfiguration& config, G1Brain* _brain)
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


BT::NodeStatus Kick::onStart()
{
    _startTime = brain->get_clock()->now();

    double vxLimit = 1.6;
    double vyLimit = 0.6;
    double vthetaLimit = 1.6;

    int minMSecKick = 1000;

    double adjustedYaw = brain->data->ballYawToPelvis
    double tx = cos(adjustedYaw) * brain->data->ballRange; // 移动的目标
    double ty = sin(adjustedYaw) * brain->data->ballRange;
    double vx, vy;
    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    {
        vx = vxLimit;
        vy = 0.0;
    }
    else
    { 
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }
    double speed = norm(vx, vy);
    _msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ballRange / speed * 1000) : minMSecKick;
    
    
    // brain->client->setVelocity(vx, vy, 0);
    return BT::NodeStatus::SUCCESS;
}

NodeStatus Kick::onRunning()
{
    if (brain->msecsSince(_startTime) < _msecKick)
        return NodeStatus::RUNNING;

    // else
    // brain->client->move(0, 0, 0);
    return NodeStatus::SUCCESS;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

BT::NodeStatus StrikerDecide::tick()
{
    string lastDecision;
    getInput("decision_in", lastDecision);


    double kickDir = atan2(-brain->data->ballPositionInField[1], brain->config->fieldDimensions.length / 2 - brain->data->ballPositionInField[0]);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; // 球到左边门柱的角度(我们的左)
    double theta_r = goalPostAngles[1]; // 球到右边门柱的角度
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = brain->data->ballRange;
    double ballYaw = brain->data->ballYawToPelvis;

    string newDecision;
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);


    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
    }
    else
    {
        newDecision = "adjust";
    }

    setOutput("decision_out", newDecision);

    return NodeStatus::SUCCESS;
}

BT::NodeStatus GoalieDecide::tick()
{
    string lastDecision;
    getInput("decision_in", lastDecision);


    double kickDir = atan2(-brain->data->ballPositionInField[1], brain->config->fieldDimensions.length / 2 - brain->data->ballPositionInField[0]);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; // 球到左边门柱的角度(我们的左)
    double theta_r = goalPostAngles[1]; // 球到右边门柱的角度
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = brain->data->ballRange;
    double ballYaw = brain->data->ballYawToPelvis;

    string newDecision;
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);


    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "gohome"))
    {
        newDecision = "gohome";
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
    }
    else
    {
        newDecision = "adjust";
    }

    setOutput("decision_out", newDecision);
    
    return NodeStatus::SUCCESS;
}


