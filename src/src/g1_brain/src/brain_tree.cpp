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
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


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
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(CamScanField)
    

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    
    tree = factory.createTree("MainTree");
    
    //init blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<std::string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<std::string>("decision", "");
    setEntry<std::string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);
    setEntry<int>("control_state", 0);

}

void BrainTree::tick()
{
    // RCLCPP_INFO(rclcpp::get_logger("BrainTree"), "Starting tree.tickOnce()");
    tree.tickOnce();
    // RCLCPP_INFO(rclcpp::get_logger("BrainTree"), "Completed tree.tickOnce()");
}

// =================== 节点实现 ===================

BT::NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch = -0.3;
    double highPitch = 0.6;
    double leftYaw = 0.85;
    double rightYaw = -0.85;
    
    // 写死扫描周期为 3000ms
    int msecCycle = 3000;

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}


BT::NodeStatus SelfLocate::tick() {

    string mode = getInput<string>("mode").value();   //可以通过行为树输入
    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0; // 结束条件
    auto markers = brain->data->getMarkers();

    // std::cout << "[DEBUG] markers.size(): " << markers.size() << std::endl;
    


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
    else if (mode == "trust_direction")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                      // m/s  0.1vs 0.2
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // 在这个时间内, odom 最多漂移了多少距离

        xMin = max(-brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = min(brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = max(-brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = min(brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 18;  // 18 vs 4
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 18;  // 18 vs 4
    }
    else {
        std::cout << "[ERROR]: Unsupported mode, " << mode << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    auto res = brain->locator->locateRobot(markers, constraints);

    // std::cout << "locate result: res: " << std::to_string(res.code) << " time: " << std::to_string(res.msecs) << std::endl;
    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.
    
    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);

    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    
    // std::cout << "locate success: " << std::to_string(res.pose.x) << " " << std::to_string(res.pose.y) << " " << std::to_string(rad2deg(res.pose.theta)) << " Dur: " << std::to_string(res.msecs) << std::endl;

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Adjust::tick()
{
    if(!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return BT::NodeStatus::SUCCESS; 
    }


    double vx = 0, vy = 0, vtheta = 0;
    double vxLimit = 0.5;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;


    double kickDir = atan2(-brain->data->ball.posToField.x, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double dir = deltaDir > 0 ? -1.0 : 1.0;
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.pitchToRobot;


    double s = 0.4;
    double r = 0.8;

    vx = -s * dir * sin(ballYaw);
    vy = s * dir * cos(ballYaw);
    vtheta = (ballYaw - dir * s) / r;


    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    brain->client->SetVelocity(vx,vy,vtheta);

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus CamTrackBall::tick()
{
    double pitch, yaw;
    if (!brain->data->ballDetected)
    {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
    }
    else
    {
        const double pixTolerance = 10;

        double deltaX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin) - brain->config->camPixX / 2;
        double deltaY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin) - brain->config->camPixY * 2 / 3;

        if (std::fabs(deltaX) < pixTolerance && std::fabs(deltaY) < pixTolerance)
        {
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5;
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;//追踪逻辑

        brain->client->moveHead(pitch,yaw);
    }
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    brain->client->SetVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

BT::NodeStatus Chase::tick()
{
    if(!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->SetVelocity(0,0,0);
        return BT::NodeStatus::SUCCESS; 
    }

    double dist;
    getInput("dist", dist);


    double vxLimit = 1.0;
    double vyLimit = 1.0;
    double vthetaLimit = 0.25;


    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    Pose2D target_f, target_r;
    if (brain->data->robotPoseToField.x - brain->data->ball.posToField.x > (_state == "chase" ? 1.0 : 0.0))
    { // circle back
        _state = "circle_back";
        // 目标 x 坐标
        target_f.x = brain->data->ball.posToField.x - dist;

        // 目标 y 坐标. 即决策从哪边绕, 并防止震荡
        if (brain->data->robotPoseToField.y > brain->data->ball.posToField.y - _dir)
            _dir = 1.0;
        else
            _dir = -1.0;

        target_f.y = brain->data->ball.posToField.y + _dir * dist;
    }
    else
    { // chase
        _state = "chase";
        target_f.x = brain->data->ball.posToField.x - dist;
        target_f.y = brain->data->ball.posToField.y;
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

    
    brain->client->SetVelocity(vx, vy, vtheta);
    return BT::NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const std::string& name, const NodeConfig& config, G1Brain* _brain)
    : SyncActionNode(name, config), brain(_brain)
{
    // 初始化预定义动作
    double lowPitch = -0.25;
    double highPitch = 0.5;
    double leftYaw = 0.6;
    double rightYaw = -0.6;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 800;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();

}

BT::NodeStatus CamFindBall::tick()          //可以尝试时间控制
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    }

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    }
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {
        _cmdIndex = 0;
    }
    else
    {
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();

    return NodeStatus::SUCCESS;
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

    int minMSecKick = 1000;

    double adjustedYaw = brain->data->ball.yawToRobot;
    double tx = cos(adjustedYaw) * brain->data->ball.range; // 移动的目标
    double ty = sin(adjustedYaw) * brain->data->ball.range;
    double vx, vy;
    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    {
        vx = vxLimit;
        vy = 0.0;
    }
    else
    { 
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }
    double speed = norm(vx, vy);
    _msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    
    brain->client->SetVelocity(vx, vy, 0);
    return BT::NodeStatus::SUCCESS;
}

NodeStatus Kick::onRunning()
{
    if (brain->msecsSince(_startTime) < _msecKick)
        return NodeStatus::RUNNING;

    // else
    // brain->client->SetVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

BT::NodeStatus StrikerDecide::tick()
{
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision ;
    getInput("decision_in", lastDecision);


    double kickDir = atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; // 球到左边门柱的角度(我们的左)
    double theta_r = goalPostAngles[1]; // 球到右边门柱的角度
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;


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

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);


    double kickDir = atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; // 球到左边门柱的角度(我们的左)
    double theta_r = goalPostAngles[1]; // 球到右边门柱的角度
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;


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

BT::NodeStatus MoveToPoseOnField::tick()
{

    double tx, ty, ttheta, longRangeThreshold, turnThreshold, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);

    double vxLimit = 1.0 ;
    double vyLimit = 0.5 ;
    double vthetaLimit = 0.4 ;

    brain->client->moveToPoseOnField(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);
    return NodeStatus::SUCCESS;

}

BT::NodeStatus RobotFindBall::onStart()
{
    if(brain->data->ballDetected)
    {
        brain->client->SetVelocity(0,0,0);
        return BT::NodeStatus::SUCCESS;
    }
    turn_dir = brain->data->ball.yawToRobot >0 ? 1.0 : -1.0;

    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus RobotFindBall::onRunning()
{
    if(brain->data->ballDetected)
    {
        brain->client->SetVelocity(0,0,0);
        return BT::NodeStatus::SUCCESS;
    }
    double vyawLimit = 1.0;
    getInput("vyaw_limit", vyawLimit);

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    brain->client->SetVelocity(0, 0, vyawLimit * turn_dir);
    return BT::NodeStatus::RUNNING;
}
void RobotFindBall::onHalted()
{
    turn_dir = 1.0;
}