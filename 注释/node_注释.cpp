#include "control/node.h"

BT::NodeStatus camToPosition::tick() 
//云台位置控制
{
    //获取目标角度和运动时间
    double joint0_angle, joint1_angle, duration;
    getInput("joint0_angle", joint0_angle);//云台水平角度
    getInput("joint1_angle", joint1_angle);//云台俯仰角度
    getInput("duration", duration);         //运动时间



    //初始化
    if(!firstRun)
    {
        //记录当前关节角度
        initAngle<<_interface->servoState->msg_.states()[0].q(),_interface->servoState->msg_.states()[1].q();
        targetAngle.Zero(2);
        targetAngle<<joint0_angle,joint1_angle;//设置目标角度
        firstRun = true;
    }

    //计算运动进度
    percent += (float)1 / duration;
    percent = percent > 1 ? 1 : percent;

    //关节0线性插值
    int desired0_position = (1 - percent) * initAngle(0) + percent * targetAngle(0);
    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = desired0_position;

    //关节1线性插值
    int desired1_position = (1 - percent) * initAngle(1) + percent * targetAngle(1);
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = desired1_position;
    
    //发布控制命令
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus camFindBall::tick()//云台找球
/*
功能：当丢失球体时，执行预定义的扫描路径寻找球
核心逻辑：多阶段插值扫描
*/
{


    //计算当前目标角度
    interpolator.interpolate(targetAngle);
    //发送控制命令
    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = targetAngle(0);
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = targetAngle(1);
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
}


/*
    扫描示例
    predefinedPhases = {
    {{30, 20}, 2.0},   // 位置1，持续2秒
    {{-30, 20}, 2.0},  // 位置2
    {{0, 40}, 2.0}     // 位置3
}*/

BT::NodeStatus robotTrackPelvis::tick()
/*robotTrackPelvis（骨盆坐标系跟踪）
功能：在机器人自身坐标系下跟踪球体,直至靠近球附近
核心逻辑：基于距离的S型速度曲线*/
{
    if (!_interface->ballDetected)
    {
        _interface->locoClient.Move(0, 0, 0);//停止移动
        return BT::NodeStatus::SUCCESS;
    }


    //检查是否在目标区域内
    if((abs(_interface->ballYawToPelvis)<=0.20) && (abs(_interface->ballPositionInPelvis(0)) <= 0.55)&& (abs(_interface->ballPositionInPelvis(1)) <= 0.3))
    
    /*
    在机器人正前方（左右偏角不超过0.2rad）

    在机器人前方0.55米以内（不会太远）

    在机器人横向0.3米范围内（不会太偏左或太偏右）
    */
    {
        _interface->locoClient.Move(0, 0, 0);//已在目标位置
    }
    else
    {
        
        //计算速度分量
        double vx = _interface->ballPositionInPelvis(0);
        double vy = _interface->ballPositionInPelvis(1);

        const double linearFactorSlope = 3.0;     
        const double linearFactorOffset = 3.0;  
        //计算S型速度系数（近快远慢）   
        double linearFactor = 1.0 / (1.0 + exp(
            linearFactorSlope * (_interface->ball_range_selected * fabs(_interface->ballYawToPelvis)) - linearFactorOffset
        ));

        //速度限幅
        vx *= linearFactor;
        vy *= linearFactor;
        vx = saturation(vx, Vec2<double>(-0.8,1.0));
        vy = saturation(vy, Vec2<double>(-0.8,1.0));
        double vyaw = _interface->ballYawToPelvis;
        vyaw = saturation(vyaw, Vec2<double>(-0.5,0.5));

        //发送移动命令
        _interface->locoClient.Move(vx, vy,vyaw);
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus robotTrackField::tick()
/*robotTrackField（全局坐标系跟踪）
功能：在场地全局坐标系下跟踪球并调整射门角度
核心逻辑：追逐+射门角度调整的混合控制*/
{
    //基础追逐速度计算
    double vx_chase = _interface->ballPositionInPelvis(0);
    double vy_chase = _interface->ballPositionInPelvis(1);

    double linearFactor = 1 / (1 + exp(3 * (_interface->ball_range_selected * fabs(_interface->ballYawToPelvis)) - 3));
    vx_chase *= linearFactor;
    vy_chase *= linearFactor;

    
    vx_chase = saturation(vx_chase, Vec2<double>(-0.6,0.6));
    vy_chase = saturation(vy_chase, Vec2<double>(-0.6,0.6));



    double vyaw_chase= _interface->ballYawToPelvis;
    vyaw_chase = saturation(vyaw_chase, Vec2<double>(-0.5,0.5));
 
    //射门角度计算
    Vec2<double> vec_goal_ball_field;
    vec_goal_ball_field(0) = 4.5 - _interface->ballPositionInField(0);
    vec_goal_ball_field(1) = 0 - _interface->ballPositionInField(1);

    double angle_goal_ball_field = atan2(vec_goal_ball_field(1),vec_goal_ball_field(0));

    //计算机器人-球体向量
    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);

    double angle_robot_ball_field = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    //计算角度差以及旋转方向
    double deltaDir = angle_goal_ball_field - angle_robot_ball_field;

    double dir = deltaDir > 0 ? -1.0 : 1.0;//判断旋转方向


    
    double s = 0.4;
    double r = 0.8;
    //计算旋转速度
    double ballYawToPelvis = _interface->ballYawToPelvis;
    double vtheta = (ballYawToPelvis - dir * s) / r;

    //计算速度调整量
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


    //混合权重计算
    double d_switch = 1.5; // 切换距离，可根据需求调整
    double ballRange = _interface->ball_range_selected;
    /*动态追踪权重计算，距离越远，追踪权重越高*/
    double w_chase = std::clamp(ballRange / d_switch, 0.0, 1.0);
    double w_orbit = 1.0 - w_chase;
    // std::cout<<std::endl;
    // std::cout<<"debug d_switch: "<<d_switch<<std::endl;
    // std::cout<<"debug w_orbit: "<<w_orbit<<std::endl;
    // std::cout<<"ballRange: "<<ballRange<<std::endl;
    // std::cout<<"deltaDir: "<<deltaDir<<std::endl;
    // std::cout<<"vec_goal_ball_field(1): "<<vec_goal_ball_field(1)<<std::endl;

    //合成最终速度
    double vx = w_chase * vx_chase + w_orbit * vx_adjust;
    double vy = w_chase * vy_chase + w_orbit * vy_adjust;
    double vyaw = w_chase * vyaw_chase + w_orbit * vyaw_adjust;

    //发送移动命令
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
/*kick（踢球动作）
功能：执行踢球动作
核心逻辑：角度检查+直线冲刺*/
{
    Vec2<double> leftGoalField;
    leftGoalField(0) = 4.5;     //球门左柱x坐标
    leftGoalField(1) = 2.6 / 2; //球门左柱y坐标

    Vec2<double> rightGoalField;
    rightGoalField(0) = 4.5;    //球门右柱x坐标
    rightGoalField(1) = -2.6 / 2;//球门右柱y坐标

    double margin = 0.3; //安全距离

    Vec2<double> vecBallLeftGoalField; //球到左门柱向量计算
    vecBallLeftGoalField(0) = leftGoalField(0) - _interface->ballPositionInField(0);
    vecBallLeftGoalField(1) = leftGoalField(1) - margin - _interface->ballPositionInField(1);
    double angleballLeftGoalField = atan2(vecBallLeftGoalField(1),vecBallLeftGoalField(0));

    Vec2<double> vecBallRightGoalField;//球到右门柱向量计算
    vecBallRightGoalField(0) = rightGoalField(0) - _interface->ballPositionInField(0);
    vecBallRightGoalField(1) = rightGoalField(1) + margin - _interface->ballPositionInField(1);
    double angleballRightGoalField = atan2(vecBallRightGoalField(1),vecBallRightGoalField(0));

    Vec2<double> vecPelvisBallField;   //机器人到球的向量计算
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);
    
    
    //计算射门角度
    double angleRobotBallField = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    //检查射门角度是否合适，角度不合适不踢球
    if((angleRobotBallField<angleballLeftGoalField) && (angleRobotBallField>angleballRightGoalField))
    {
        std::cout<<"[node::kick] The shooting angle looks good "<<std::endl;
    }
    else
    {
        std::cout<<"[node::kick] The shooting angle doesn’t look good "<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    //计算踢球方向向量
    double ballYawToPelvis = _interface->ballYawToPelvis;
    double p = 0.6;//冲刺速度系数
    double vx = 0,vy = 0;
    vx = p * cos(ballYawToPelvis);//x方向分量
    vy = p * sin(ballYawToPelvis);//y方向分量

    //速度限制（最大）
    vx = saturation(vx, Vec2<double>(-0.6,0.6));
    vy = saturation(vy, Vec2<double>(-0.6,0.6));

    double vyaw = _interface->ballYawToPelvis;
    vyaw = saturation(vyaw, Vec2<double>(-0.6,0.6));
    
    //执行踢球（球未过线时冲刺）
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
/*camTrackBall（视觉跟踪）
核心逻辑：比例控制*/
{
    if (!_interface->ballDetected)
    {
        return BT::NodeStatus::SUCCESS;
    }

    //获取球在图像中偏移量
    float fov_x = _interface->ball_offset_fov(0);
    float fov_y = _interface->ball_offset_fov(1);

    //计算需要调整的角度
    yaw_angle_add = fov_x * 0.6;    //水平调整系数
    pitch_angle_add = fov_y * 0.6;  //垂直调整系数

    //计算新的云台角度
    float control_yaw = _interface->servoState->msg_.states()[0].q()-yaw_angle_add;
    float control_pitch = _interface->servoState->msg_.states()[1].q()-pitch_angle_add;

    //发送控制命令
    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = control_yaw;
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = control_pitch;
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
}
/*控制原理：
偏移越大 → 调整角度越大
系数0.6决定系统响应速度*/


BT::NodeStatus playerDecision::tick()
/*playerDecision（决策引擎）
功能：根据环境状态选择最佳行为
决策逻辑：

1.未检测到球 → 启动找球模式

2.检测到球但位置不佳 → 全局跟踪

3.球在理想位置 → 踢球

4.球已进球门 → 停止*/
{
    std::string decision;
    //判断进球
    bool goalSignal;
    if(_interface->ballPositionInField(0)>4.5) // Here, we simply treat the situation where the ball’s x-direction distance in the Field coordinate system is greater than 4.5 as a goal signal.
    {
        goalSignal = true;
    }
    else
    {
        goalSignal = false;
    }

    //判断是否需要找球
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
    goalField(0) = 4.5;  // 球门在场地坐标系中的X坐标
    goalField(1) = 0;    // 球门在场地坐标系中的Y坐标（中心位置）

    bool enableRobotTrackFieldNode;
    Vec2<double> vecBallGoalField;//球-门角度计算
    vecBallGoalField(0) = goalField(0) - _interface->ballPositionInField(0);
    vecBallGoalField(1) = goalField(1) - _interface->ballPositionInField(1);
    double angleBallGoalField = atan2(vecBallGoalField(1),vecBallGoalField(0));

    Vec2<double> vecPelvisBallField;//机器人-球角度计算
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);
    double anglePlevisBallfield = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double biasAngle = angleBallGoalField - anglePlevisBallfield;//角度偏差计算（门-球-机器人是否三点一线）
    if(goalSignal|| ((abs(_interface->ballYawToPelvis)<=0.20) && (abs(_interface->ballPositionInPelvis(0)) <= 0.55) && (abs(_interface->ballPositionInPelvis(1)) <= 0.3) && (fabs(biasAngle) < 0.20) ))
    {
       enableRobotTrackFieldNode = false;
    }
    else 
    {
       enableRobotTrackFieldNode = true;
    }
    
    
    // 行为决策——是否需要全局追踪
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

    // 输出决策结果
    setOutput("decision", decision);
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus WaistControl::tick()
/*WaistControl（手腕控制）
功能：平滑控制手腕关节角度
核心逻辑：时间插值+PD控制*/
{
    getInput("waist_angle", target_angle);

    //首次运行初始化
    if(!hasSubWaistState)
    {
        init_angle = _interface->lowState->msg_.motor_state()[JointIndex::kWaistYaw].q();
        std::cout<<"debug init_angle: "<<std::endl;
        std::cout<<init_angle<<std::endl;
        hasSubWaistState = true;
        weight = 1.0;
        _interface->armCmd->msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
        _interface->armCmd->unlockAndPublish();
    }
    _percent_1 += (float)1 / _duration_1;
    _percent_1 = _percent_1 > 1 ? 1 : _percent_1;

    //计算插值进度
    float desired_position =  (1 - _percent_1) * init_angle + _percent_1 * target_angle;

    std::cout<<"debug desired_position: "<<std::endl;
    std::cout<<desired_position<<std::endl;

    //发送控制命令（包含PD参数（刚度+阻尼））
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).q() = desired_position;
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).kp() = kp;   //刚度系数
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).kd() = kd;   //阻尼系数
    _interface->armCmd->unlockAndPublish();
    // _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    // _interface->servoCmd->msg_.cmds()[0].q() = desired0_position;

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Speak::tick()
/*Speak（语音播报）
功能：文本转语音输出*/
{
    int32_t ret;
    std::string context;
    getInput("text", context);//获取要播报的文本
    uint8_t volume;
    ret = _interface->audioClient.GetVolume(volume);
    std::cout << "GetVolume API ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;
    
    //设置音量
    ret = _interface->audioClient.SetVolume(100);
    std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

    std::cout<<"debug text: "<<context<<std::endl;

    //文本转语音并播放
    ret = _interface->audioClient.TtsMaker(context,
                        0);  // 自动播放
    std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

    return BT::NodeStatus::SUCCESS;
}




BT::NodeStatus AbnormalCondition::tick()
{
    B2G_RotMat = _interface->rotMatPelvisToGlobal;
    G2B_RotMat = B2G_RotMat.transpose();

    //重力向量投影计算
    Vec3<double> projected_gravity_body,projected_gravity_world;
    projected_gravity_world<<0,0,-1;
    projected_gravity_body = G2B_RotMat * projected_gravity_world;
    std::cout<<"[AbnormalCondition::tick]"<<std::endl;
    // std::cout<<"[AbnormalCondition::tick] debug projected_gravity_body: "<<std::endl;
    // std::cout<< projected_gravity_body <<std::endl;

    //异常状态检测（倾斜幅度过大）
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
