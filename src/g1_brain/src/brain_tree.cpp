#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include "brain_tree.h"
#include "brain.h"



void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes
    REGISTER_BUILDER(selfLocate)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(robotTrackField)

    // Action Nodes for debug
    REGISTER_BUILDER(PrintMsg)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // init blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<bool>("gamecontroller_isKickOffExecuted", true);

    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("treat_person_as_robot", false);
    setEntry<int>("control_state", 0);
    setEntry<bool>("B_pressed", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);
}

void BrainTree::tick()
{

    tree.tickOnce();
}

NodeStatus selfLocate::tick() {
    auto markers = getMarkers();
    std::cout << "[DEBUG] markers.size(): " << markers.size() << std::endl;
    for (const auto& m : markers) {
        std::cout << "[DEBUG] marker: type=" << m.type << " x=" << m.x << " y=" << m.y << " conf=" << m.confidence << std::endl;
    }
    if (markers.size() < 4) {
        std::cout << "[WARN] Not enough markers for localization!" << std::endl;
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
        xMin = -fd.length / 2; // TODO: 目前只测试半场
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

        xMin = max(-fd.length / 2, robotPoseToField.x - maxDrift);
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


NodeStatus Adjust::tick()
{
    // 计算球相对于骨盆的偏航角
    double ballYawToPelvis = atan2(brain->ballPositionInPelvis(1), brain->ballPositionInPelvis(0));
    
    double p = 0.8;
    double vx = p * cos(ballYawToPelvis);
    double vy = p * sin(ballYawToPelvis);

    vx = saturation(vx, Vec2<double>(-1.2,1.2));
    vy = saturation(vy, Vec2<double>(-1.2,1.2));

    double vyaw = ballYawToPelvis;
    vyaw = saturation(vyaw, Vec2<double>(-1.2,1.2));

    _interface->locoClient.Move(vx,vy,0);  //
        
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    if (!brain->data->ballDetected)
    {
        return BT::NodeStatus::SUCCESS;
    }

   // float fov_x = _interface->ball_offset_fov(0);
   // float fov_y = _interface->ball_offset_fov(1);

   // yaw_angle_add = fov_x * 0.6;
   // pitch_angle_add = fov_y * 0.6;

    float control_yaw = motor_states->states[0].q//-yaw_angle_add;
    float control_pitch = motor_states->states[1].q//+pitch_angle_add;

    motor_cmds->states[0].mode = 1;
    motor_cmds->states[0].q = control_yaw;
    motor_cmds->states[1].mode = 1;
    motor_cmds->states[1].q = control_pitch;
    motor_cmd_publisher_->publish(*motor_cmds);
    return NodeStatus::SUCCESS;
}

NodeStatus CamFindBall::tick()
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
            motorstates.states[0].q,
            motorstates.states[1].q  // 当前X轴角度  // 当前Y轴角度
        );

        interpolator.reset(initAngle);

        
        for (const auto& [target, duration] : predefinedPhases)
        {
            interpolator.addPhase(target, duration);
        }
        firstRun = false;
    }


    interpolator.interpolate(targetAngle);

    
    motor_cmds->states[0].mode = 1;  // 位置模式
    motor_cmds->states[0].q= targetAngle(0);//目标位置

    
    float limitedY = std::clamp(targetAngle(1), Y_SERVO_MIN, Y_SERVO_MAX);
     motor_cmds->states[1].mode= 1;  // 位置模式
     motor_cmds->states[1].q= limitedY;

    motor_cmd_publisher_->publish(*motor_cmds);

    return NodeStatus::SUCCESS;  
}

NodeStatus robotTrackField::tick()
{
    // 计算球相对于骨盆的位置和角度
    double ballYawToPelvis = atan2(brain->ballPositionInPelvis(1), brain->ballPositionInPelvis(0));
    double ballRange = sqrt(pow(brain->ballPositionInPelvis(0), 2) + pow(brain->ballPositionInPelvis(1), 2));
    
    double vx_chase = brain->ballPositionInPelvis(0);
    double vy_chase = brain->ballPositionInPelvis(1);

    double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYawToPelvis)) - 3));
    vx_chase *= linearFactor;
    vy_chase *= linearFactor;

    vx_chase = saturation(vx_chase, Vec2<double>(-1,1));
    vy_chase = saturation(vy_chase, Vec2<double>(-1,1));

    double vyaw_chase = ballYawToPelvis;
    vyaw_chase = saturation(vyaw_chase, Vec2<double>(-1,1));

    // 计算球在场上的位置
    Vec2<double> ballPositionInField;
    ballPositionInField(0) = brain->homoMatPelvisToField(0,2) + brain->ballPositionInPelvis(0);
    ballPositionInField(1) = brain->homoMatPelvisToField(1,2) + brain->ballPositionInPelvis(1);

    Vec2<double> vec_goal_ball_field;
    vec_goal_ball_field(0) = 4.5 - brain(0);
    vec_goal_ball_field(1) = 0 - brain(1);

    double angle_goal_ball_field = atan2(vec_goal_ball_field(1),vec_goal_ball_field(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = ballPositionInField(0) - brain->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = ballPositionInField(1) - brain->homoMatPelvisToField(1,2);

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

    _interface->locoClient.Move(vx, vy,vyaw);

    return NodeStatus::SUCCESS;
}

NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    if (!getInput("x", x) || !getInput("y", y) || !getInput("theta", theta)) {
        return NodeStatus::FAILURE;
    }

    _interface->locoClient.Move(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus Kick::tick()
{
    // 计算球在场上的位置
    Vec2<double> ballPositionInField;
    ballPositionInField(0) = brain->homoMatPelvisToField(0,2) + brain->ballPositionInPelvis(0);
    ballPositionInField(1) = brain->homoMatPelvisToField(1,2) + brain->ballPositionInPelvis(1);

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
    vecPelvisBallField(0) = ballPositionInField(0) - brain->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = ballPositionInField(1) - brain->homoMatPelvisToField(1,2);
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
        vecPelvisBallField(0) = ballPositionInField(0) - brain->homoMatPelvisToField(0,2);
        vecPelvisBallField(1) = ballPositionInField(1) - brain->homoMatPelvisToField(1,2);
        double anglePlevisBallfield = atan2(vecPelvisBallField(1),vecPelvisBallField(0));
        double biasAngle = angleBallGoalField - anglePlevisBallfield;
        double vx =0, vy =0, vtheta = 0;
        double s = 0.4, r=0.8;
        double ball_yaw =atan2(brain->ballPositionInPelvis(1),brain->ballPositionInPelvis(0));
        vx = s*sin(ball_yaw);
        vy = -s*cos(ball_yaw);
        vtheta = (ball_yaw+s)/r;

        _interface->locoClient.Move(vx,vy,vtheta);
    }
    return NodeStatus::SUCCESS;
}
NodeStatus PlayerDesicion::tick()
{
    std::string decision;
    if(brain->ballPositionInField(0)> 4.6||abs(brain->ballPositionInField(1))>3) // Here, we simply treat the situation where the ball’s x-direction distance in the Field coordinate system is greater than 4.5 as a goal signal.
    {
         goalSignal = true;
        //goalSignal = false; 
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
    vecBallGoalField(0) = goalField(0) - brain->ballPositionInField(0);
    vecBallGoalField(1) = goalField(1) - brain->ballPositionInField(1);
    double angleBallGoalField = atan2(vecBallGoalField(1),vecBallGoalField(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = brain->ballPositionInField(0) - brain->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = brain->ballPositionInField(1) - brain->homoMatPelvisToField(1,2);
    double anglePlevisBallfield = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double biasAngle = angleBallGoalField - anglePlevisBallfield;
    if(goalSignal|| ((abs(brain->ballYawToPelvis)<=0.20) && (abs(brain->ballPositionInPelvis(0)) <= 0.55) && (abs(brain->ballPositionInPelvis(1)) <= 0.3)))
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
        goal_wly = true;
        
    }
    
    std::cout<<"debug decision: "<<std::endl;
    std::cout<<decision<<std::endl;

    setOutput("decision", decision);
    return BT::NodeStatus::SUCCESS;
}