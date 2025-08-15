#include "control/node.h"

#include <thread>  // 添加该头文件
#include <chrono>  // 添加该头文件
#include <string>  // 添加该头文件


BT::NodeStatus camFindBall::tick()
{

    if (!_interface->ballDetected)
    {
        if (firstRun)
        {
            initAngle << _interface->servoState->msg_.states()[0].q(), _interface->servoState->msg_.states()[1].q();

            Vec2f initAngle(
                _interface->servoState->msg_.states()[0].q(),
                _interface->servoState->msg_.states()[1].q()
            );

            interpolator.reset(initAngle);

            for (const auto& [target, duration] : predefinedPhases)
            {
                interpolator.addPhase(target, duration);
            }
            firstRun = false;
        }
    }
    else
    {
        firstRun = true;
        interpolator = MultiStageInterpolator();
        return BT::NodeStatus::SUCCESS;
    }
    if(!_interface->ballDetected && !firstRun){
        _interface->locoClient.Move(0, 0, 1);
    }
    // 限位角度：单位是弧度（45度 = π/4）
    constexpr float Y_SERVO_MIN = -M_PI / 3.0f; // 最小俯视角
    constexpr float Y_SERVO_MAX =  M_PI / 6.0f; // 最大仰角：30度


    interpolator.interpolate(targetAngle);

    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = targetAngle(0);


    // 舵机y轴限位
    float limitedY = std::clamp(targetAngle(1), Y_SERVO_MIN, Y_SERVO_MAX);
    _interface->servoCmd->msg_.cmds()[1].mode() = 1; 
    _interface->servoCmd->msg_.cmds()[1].q() = targetAngle(1);
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
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

    Pose2D target_f, target_r; // 移动目标在 field 和 robot 坐标系中的 Pose
    target_f.x = tx;
    target_f.y = ty;
    target_f.theta = ttheta;
    target_r = _interface->field2robot(target_f);
    double targetAngle = atan2(target_r.y, target_r.x);
    double targetDist = norm(target_r.x, target_r.y);

    double vx, vy, vtheta;
    // 已经到达目标?
    if ((fabs(_interface->robotpose2field_x - target_f.x) < xTolerance) && (fabs(_interface->robotpose2field_y - target_f.y) < yTolerance) && (fabs(toPInPI(_interface->robotpose2field_theta - target_f.theta)) < thetaTolerance))
    {
        _interface->locoClient->Move(0, 0, 0);
        return BT::NodeStatus::SUCCESS;
    }

    static double breakOscillate = 0.0;
    if (targetDist > longRangeThreshold - breakOscillate)
    {
        breakOscillate = 0.5;

        // 角度较大, 先转向目标点
        if (fabs(targetAngle) > turnThreshold)
        {
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
            _interface->locoClient->Move(0, 0, vtheta);
        }

        // else

        vx = cap(target_r.x, vxLimit, -vxLimit);
        vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        _interface->locoClient->Move(vx, 0, vtheta);
    }

    // else 比较近了
    breakOscillate = 0.0;
    vx = cap(target_r.x, vxLimit, -vxLimit);
    vy = cap(target_r.y, vyLimit, -vyLimit);
    vtheta = cap(target_r.theta, vthetaLimit, -vthetaLimit);
    _interface->locoClient->Move(vx, vy, vtheta);

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Adjust::tick()
{
    bool ball_location_known;
    if(!getEntry("ball_location_known", ball_location_known) || !ball_location_known)
    {
        return BT::NodeStatus::SUCCESS; 
    }


    double vx = 0, vy = 0, vtheta = 0;
    double vxLimit = 0.5;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;

    double kickDir = atan2(-_interface->ballPositionInField[0], 4.5 - _interface->ballPositionInField[0]);
    double dir_rb_f = _interface->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double dir = deltaDir > 0 ? -1.0 : 1.0;
    double ballRange = _interface->ball_range_selected;
    double ballYaw = _interface->ball_pitchtorobot;


    double s = 0.4;
    double r = 0.8;

    vx = -s * dir * sin(ballYaw);
    vy = s * dir * cos(ballYaw);
    vtheta = (ballYaw - dir * s) / r;


    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    _interface->locoClient.Move(vx,vy,vtheta);

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Chase::tick()
{
   bool ball_location_known;
   if(!getEntry("ball_location_known", ball_location_known) || !ball_location_known)
    {
        _interface->locoClient->Move(0,0,0);
        return BT::NodeStatus::SUCCESS; 
    }

    double dist;
    getInput("dist", dist);


    double vxLimit = 1.0;
    double vyLimit = 1.0;
    double vthetaLimit = 0.25;


    double ballRange = _interface->ball_range_selected;
    double ballYaw = _interface->ball_pitchtorobot;

    Pose2D target_f, target_r;
    if (_interface->robotpose2field_x - _interface->ballPositionInField[0] > (_state == "chase" ? 1.0 : 0.0))
    { // circle back
        _state = "circle_back";
        // 目标 x 坐标
        target_f.x = _interface->ballPositionInField[0] - dist;

        // 目标 y 坐标. 即决策从哪边绕, 并防止震荡
        if (_interface->robotpose2field_y > _interface->ballPositionInField[1] - _dir)
            _dir = 1.0;
        else
            _dir = -1.0;

        target_f.y = _interface->ballPositionInField[1] + _dir * dist;
    }
    else
    { // chase
        _state = "chase";
        target_f.x = _interface->ballPositionInField[0] - dist;
        target_f.y = _interface->ballPositionInField[1];
    }

    target_r = _interface->field2robot(target_f);
    double vx = target_r.x;
    double vy = target_r.y;
    double vtheta = ballYaw * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYaw)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;


    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    
    _interface->locoClient.Move(vx, vy, vtheta);
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus kick::onStart()
{
    _startTime = std::chrono::steady_clock::now(); // 获取当前时间点作为开始时间

    double vxLimit = 1.6;
    double vyLimit = 0.6;

    int minMSecKick = 1000; // 最小踢球时间（毫秒）

    double adjustedYaw = _interface->ballYawToPelvis;
    double tx = cos(adjustedYaw) * _interface->ball_range_selected; // 移动的目标位置
    double ty = sin(adjustedYaw) * _interface->ball_range_selected;
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

    double speed = sqrt(vx * vx + vy * vy); // norm(vx, vy)，计算速度
    _msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(_interface->ball_range_selected / speed * 1000) : minMSecKick;

    _interface->locoClient->Move(vx, vy, 0);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus kick::onRunning()
{
    // 计算从 _startTime 到现在的时间差
    auto elapsed_time = std::chrono::steady_clock::now() - _startTime;
    auto elapsed_msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count();

    if (elapsed_msec < _msecKick)
    {
        return BT::NodeStatus::RUNNING; // 如果未达预期时间，继续执行
    }

    // 如果达预期时间，执行完成逻辑
    _interface->locoClient->Move(0, 0, 0);
    return BT::NodeStatus::SUCCESS;
}

void kick::onHalted()
{
    // 修改开始时间 `_startTime`，可以用负偏移模仿提前结束的效果
    _startTime -= std::chrono::milliseconds(100); // 让启动时间向前调整 100ms
}


BT::NodeStatus camTrackBall::tick()
{
    if (!_interface->ballDetected)
    {
        return BT::NodeStatus::SUCCESS;
    }

    float fov_x = _interface->ball_offset_fov(0);
    float fov_y = _interface->ball_offset_fov(1);

    yaw_angle_add = fov_x * 0.6;
    pitch_angle_add = fov_y * 0.6;

    float control_yaw = _interface->servoState->msg_.states()[0].q()-yaw_angle_add;
    float control_pitch = _interface->servoState->msg_.states()[1].q()+pitch_angle_add;

    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = control_yaw;
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    std::cout << "angle" << control_pitch <<std::endl;
    _interface->servoCmd->msg_.cmds()[1].q() = control_pitch;
    _interface->servoCmd->unlockAndPublish();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StrikerDecide::tick()
{
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision ;
    getEntry("decision_in", lastDecision);
    string playerRole;
    getEntry("player_role", playerRole);


    double kickDir = atan2(-_interface->ballPositionInField[0], 4.5 - _interface->ballPositionInField[0]);
    double dir_rb_f = _interface->robotBallAngleToField;
    auto goalPostAngles = _interface->getGoalPostAngles();
    double theta_l = goalPostAngles[0]; // 球到左边门柱的角度(我们的左)
    double theta_r = goalPostAngles[1]; // 球到右边门柱的角度
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = _interface->ball_range_selected;
    double ballYaw = _interface->ballYawToPelvist;

    string newDecision;


    bool ball_location_known;
    if (!getEntry("ball_location_known", ball_location_known) || !ball_location_known)
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

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getEntry("decision_in", lastDecision);
    string playerRole;
    getEntry("player_role", playerRole);


    double kickDir = atan2(-_interface->ballPositionInField[0], 4.5 - _interface->ballPositionInField[0]);
    double dir_rb_f = _interface->robotBallAngleToField;
    auto goalPostAngles = _interface->getGoalPostAngles();
    double theta_l = goalPostAngles[0]; // 球到左边门柱的角度(我们的左)
    double theta_r = goalPostAngles[1]; // 球到右边门柱的角度
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = _interface->ball_range_selected;
    double ballYaw = _interface->ballYawToPelvist;

    string newDecision;


    bool ball_location_known;
    if (!getEntry("ball_location_known", ball_location_known) || !ball_location_known)
    {
        newDecision = "find";
    }
    else if (_interface->ballPositionInField[0] > 0 - static_cast<double>(lastDecision == "gohome"))
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
    
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus RobotFindBall::onStart()
{
    if(_interface->ballDetected)
    {
        _interface->locoClient->Move(0,0,0);
        return BT::NodeStatus::SUCCESS;
    }
    turn_dir = _interface->ballYawToPelvis >0 ? 1.0 : -1.0;

    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus RobotFindBall::onRunning()
{
    if(_interface->ballDetected)
    {
        _interface->locoClient->Move(0,0,0);
        return BT::NodeStatus::SUCCESS;
    }
    double vyawLimit = 1.0;

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    _interface->locoClient->Move(0, 0, vyawLimit * turn_dir);
    return BT::NodeStatus::RUNNING;
}
void RobotFindBall::onHalted()
{
    turn_dir = 1.0;
}





BT::NodeStatus AbnormalCondition::tick()
{
    B2G_RotMat = _interface->rotMatPelvisToGlobal;
    G2B_RotMat = B2G_RotMat.transpose();
    Vec3<double> projected_gravity_body,projected_gravity_world;
    projected_gravity_world<<0,0,-1;
    projected_gravity_body = G2B_RotMat * projected_gravity_world;
    std::cout<<"[AbnormalCondition::tick]"<<std::endl;
    // std::cout<<"[AbnormalCondition::tick] debug projected_gravity_body: "<<std::endl;
    // std::cout<< projected_gravity_body <<std::endl;

    if(abs(projected_gravity_body(0))>=0.4 || abs(projected_gravity_body(1))>=0.4)
    {   
        std::cout<<"enter abnormal condition: "<<std::endl;
        if(projected_gravity_body(1)>0)
        {
            _interface->servoCmd->msg_.cmds()[0].mode() = 1;
            _interface->servoCmd->msg_.cmds()[0].q() = -50;
            _interface->servoCmd->msg_.cmds()[1].mode() = 1;
            _interface->servoCmd->msg_.cmds()[1].q() = 10;
            _interface->servoCmd->unlockAndPublish();
        }

        if(projected_gravity_body(1)<0)
        {
            _interface->servoCmd->msg_.cmds()[0].mode() = 1;
            _interface->servoCmd->msg_.cmds()[0].q() = 50;
            _interface->servoCmd->msg_.cmds()[1].mode() = 1;
            _interface->servoCmd->msg_.cmds()[1].q() = 10;
            _interface->servoCmd->unlockAndPublish();
        }
      
    }
    else
    {
        _interface->servoCmd->msg_.cmds()[0].mode() = 1;
        _interface->servoCmd->msg_.cmds()[0].q() = 0;
        _interface->servoCmd->msg_.cmds()[1].mode() = 1;
        _interface->servoCmd->msg_.cmds()[1].q() = 0;
        _interface->servoCmd->unlockAndPublish();

    }
    return BT::NodeStatus::SUCCESS;
}