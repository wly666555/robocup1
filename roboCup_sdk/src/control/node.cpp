#include "control/node.h"

BT::NodeStatus camToPosition::tick()
{
    double joint0_angle, joint1_angle, duration;
    getInput("joint0_angle", joint0_angle);
    getInput("joint1_angle", joint1_angle);
    getInput("duration", duration);

    if(!firstRun)
    {
        initAngle<<_interface->servoState->msg_.states()[0].q(),_interface->servoState->msg_.states()[1].q();
        targetAngle.Zero(2);
        targetAngle<<joint0_angle,joint1_angle;
        firstRun = true;
    }

    percent += (float)1 / duration;
    percent = percent > 1 ? 1 : percent;

    int desired0_position = (1 - percent) * initAngle(0) + percent * targetAngle(0);
    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = desired0_position;

    int desired1_position = (1 - percent) * initAngle(1) + percent * targetAngle(1);
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = desired1_position;
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
}

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

    interpolator.interpolate(targetAngle);

    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = targetAngle(0);
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = targetAngle(1);
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus robotTrackPelvis::tick()
{
    if (!_interface->ballDetected)
    {
        _interface->locoClient.Move(0, 0, 0);
        return BT::NodeStatus::SUCCESS;
    }

    if((abs(_interface->ballYawToPelvis)<=0.20) && (abs(_interface->ballPositionInPelvis(0)) <= 0.55)&& (abs(_interface->ballPositionInPelvis(1)) <= 0.3))
    {
        _interface->locoClient.Move(0, 0, 0);
    }
    else
    {
        double vx = _interface->ballPositionInPelvis(0);
        double vy = _interface->ballPositionInPelvis(1);

        const double linearFactorSlope = 3.0;     
        const double linearFactorOffset = 3.0;     
        double linearFactor = 1.0 / (1.0 + exp(
            linearFactorSlope * (_interface->ball_range_selected * fabs(_interface->ballYawToPelvis)) - linearFactorOffset
        ));

        vx *= linearFactor;
        vy *= linearFactor;
        vx = saturation(vx, Vec2<double>(-0.8,1.0));
        vy = saturation(vy, Vec2<double>(-0.8,1.0));
        double vyaw = _interface->ballYawToPelvis;
        vyaw = saturation(vyaw, Vec2<double>(-0.5,0.5));

        _interface->locoClient.Move(vx, vy,vyaw);
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus robotTrackField::tick()
{
   
    double vx_chase = _interface->ballPositionInPelvis(0);
    double vy_chase = _interface->ballPositionInPelvis(1);

    double linearFactor = 1 / (1 + exp(3 * (_interface->ball_range_selected * fabs(_interface->ballYawToPelvis)) - 3));
    vx_chase *= linearFactor;
    vy_chase *= linearFactor;

    
    vx_chase = saturation(vx_chase, Vec2<double>(-0.6,0.6));
    vy_chase = saturation(vy_chase, Vec2<double>(-0.6,0.6));

    double vyaw_chase= _interface->ballYawToPelvis;
    vyaw_chase = saturation(vyaw_chase, Vec2<double>(-0.5,0.5));
 
    Vec2<double> vec_goal_ball_field;
    vec_goal_ball_field(0) = 4.5 - _interface->ballPositionInField(0);
    vec_goal_ball_field(1) = 0 - _interface->ballPositionInField(1);

    double angle_goal_ball_field = atan2(vec_goal_ball_field(1),vec_goal_ball_field(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);

    double angle_robot_ball_field = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double deltaDir = angle_goal_ball_field - angle_robot_ball_field;

    double dir = deltaDir > 0 ? -1.0 : 1.0;

    double s = 0.4;
    double r = 0.8;

    double ballYawToPelvis = _interface->ballYawToPelvis;
    double vtheta = (ballYawToPelvis - dir * s) / r;

    double vx_adjust = 0, vy_adjust = 0;
    vx_adjust = -s * dir * sin(ballYawToPelvis);
    vy_adjust = s * dir * cos(ballYawToPelvis);
    vy_adjust = saturation(vy_adjust, Vec2<double>(-0.4,0.4));
    // std::cout<<"debug \n"<<std::endl;
    // std::cout<<"vy: "<<vy<<std::endl;
    // std::cout<<"deltaDir: "<<deltaDir<<std::endl;
    // std::cout<<"angle_goal_ball_field: "<<rad2deg(angle_goal_ball_field)<<std::endl;
    // std::cout<<"angle_robot_ball_field: "<<rad2deg(angle_robot_ball_field)<<std::endl;
    double vyaw_adjust = vtheta;
    vyaw_adjust = saturation(vtheta, Vec2<double>(-0.4,0.4));



    double d_switch = 1.5; // 切换距离，可根据需求调整
    double ballRange = _interface->ball_range_selected;
    double w_chase = std::clamp(ballRange / d_switch, 0.0, 1.0);
    double w_orbit = 1.0 - w_chase;
    // std::cout<<std::endl;
    // std::cout<<"debug d_switch: "<<d_switch<<std::endl;
    // std::cout<<"debug w_orbit: "<<w_orbit<<std::endl;
    // std::cout<<"ballRange: "<<ballRange<<std::endl;
    // std::cout<<"deltaDir: "<<deltaDir<<std::endl;
    // std::cout<<"vec_goal_ball_field(1): "<<vec_goal_ball_field(1)<<std::endl;

    double vx = w_chase * vx_chase + w_orbit * vx_adjust;
    double vy = w_chase * vy_chase + w_orbit * vy_adjust;
    double vyaw = w_chase * vyaw_chase + w_orbit * vyaw_adjust;

    _interface->locoClient.Move(vx, vy,vyaw);

    // if((abs(_interface->ballYawToPelvis)<=0.20) && (abs(_interface->ballPositionInPelvis(0)) <= 0.55)&& (abs(_interface->ballPositionInPelvis(1)) <= 0.3)&&(fabs(deltaDir) < 0.20))
    // {
    //     // _interface->locoClient.Move(0, 0, 0);
    //     _interface->trackDone = true;
    // }
    // else 
    // {
        
    //     _interface->locoClient.Move(vx, vy,vyaw);
    //     _interface->trackDone = false;
    // }


    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus kick::tick()
{
    Vec2<double> leftGoalField;
    leftGoalField(0) = 4.5;
    leftGoalField(1) = 2.6 / 2;

    Vec2<double> rightGoalField;
    rightGoalField(0) = 4.5;
    rightGoalField(1) = -2.6 / 2;

    double margin = 0.3;

    Vec2<double> vecBallLeftGoalField;
    vecBallLeftGoalField(0) = leftGoalField(0) - _interface->ballPositionInField(0);
    vecBallLeftGoalField(1) = leftGoalField(1) - margin - _interface->ballPositionInField(1);
    double angleballLeftGoalField = atan2(vecBallLeftGoalField(1),vecBallLeftGoalField(0));

    Vec2<double> vecBallRightGoalField;
    vecBallRightGoalField(0) = rightGoalField(0) - _interface->ballPositionInField(0);
    vecBallRightGoalField(1) = rightGoalField(1) + margin - _interface->ballPositionInField(1);
    double angleballRightGoalField = atan2(vecBallRightGoalField(1),vecBallRightGoalField(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);
    double angleRobotBallField = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    if((angleRobotBallField<angleballLeftGoalField) && (angleRobotBallField>angleballRightGoalField))
    {
        std::cout<<"[node::kick] The shooting angle looks good "<<std::endl;
    }
    else
    {
        std::cout<<"[node::kick] The shooting angle doesn’t look good "<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    double ballYawToPelvis = _interface->ballYawToPelvis;
    double p = 0.6;
    double vx = 0,vy = 0;
    vx = p * cos(ballYawToPelvis);
    vy = p * sin(ballYawToPelvis);

    vx = saturation(vx, Vec2<double>(-0.6,0.6));
    vy = saturation(vy, Vec2<double>(-0.6,0.6));

    double vyaw = _interface->ballYawToPelvis;
    vyaw = saturation(vyaw, Vec2<double>(-0.6,0.6));
    
    if(_interface->ballPositionInField(0)>=4.5)
    {
            _interface->locoClient.Move(0,0,0);
    }
    else
    {
            _interface->locoClient.Move(vx,vy,0);
    }
    
   return BT::NodeStatus::SUCCESS;
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
    float control_pitch = _interface->servoState->msg_.states()[1].q()-pitch_angle_add;

    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = control_yaw;
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = control_pitch;
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus playerDecision::tick()
{
    std::string decision;
    bool goalSignal;
    if(_interface->ballPositionInField(0)>4.5) // Here, we simply treat the situation where the ball’s x-direction distance in the Field coordinate system is greater than 4.5 as a goal signal.
    {
        goalSignal = true;
    }
    else
    {
        goalSignal = false;
    }

    bool enableCamFindBallNode;
    if(!_interface->ballDetected)
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
    vecBallGoalField(0) = goalField(0) - _interface->ballPositionInField(0);
    vecBallGoalField(1) = goalField(1) - _interface->ballPositionInField(1);
    double angleBallGoalField = atan2(vecBallGoalField(1),vecBallGoalField(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);
    double anglePlevisBallfield = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double biasAngle = angleBallGoalField - anglePlevisBallfield;
    if(goalSignal|| ((abs(_interface->ballYawToPelvis)<=0.20) && (abs(_interface->ballPositionInPelvis(0)) <= 0.55) && (abs(_interface->ballPositionInPelvis(1)) <= 0.3) && (fabs(biasAngle) < 0.20) ))
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
        _interface->locoClient.Move(0, 0, 0);
    }
    
    std::cout<<"debug decision: "<<std::endl;
    std::cout<<decision<<std::endl;

    setOutput("decision", decision);
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus WristControl::tick()
{
    getInput("wrist_angle", target_angle);
    if(!hasSubWristState)
    {
        init_angle = _interface->lowState->msg_.motor_state()[JointIndex::kWaistYaw].q();
        std::cout<<"debug init_angle: "<<std::endl;
        std::cout<<init_angle<<std::endl;
        hasSubWristState = true;
        weight = 1.0;
        _interface->armCmd->msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
        _interface->armCmd->unlockAndPublish();
    }
    _percent_1 += (float)1 / _duration_1;
    _percent_1 = _percent_1 > 1 ? 1 : _percent_1;

    float desired_position =  (1 - _percent_1) * init_angle + _percent_1 * target_angle;

    std::cout<<"debug desired_position: "<<std::endl;
    std::cout<<desired_position<<std::endl;
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).q() = desired_position;
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).kp() = kp;
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).kd() = kd;
    _interface->armCmd->unlockAndPublish();
    // _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    // _interface->servoCmd->msg_.cmds()[0].q() = desired0_position;

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Speak::tick()
{
    int32_t ret;
    std::string context;
    getInput("text", context);
    uint8_t volume;
    ret = _interface->audioClient.GetVolume(volume);
    std::cout << "GetVolume API ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;
    ret = _interface->audioClient.SetVolume(100);
    std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

    std::cout<<"debug text: "<<context<<std::endl;
    ret = _interface->audioClient.TtsMaker(context,
                        0);  // Auto play
    std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

    return BT::NodeStatus::SUCCESS;
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
