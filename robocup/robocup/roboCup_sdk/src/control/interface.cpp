#include "control/interface.h"

Interface::Interface()
{
    init();
}

Interface::~Interface()
{
    if(lowStateThreadPtr)
    {   
        lowStateThreadPtr->Wait();
        lowStateThreadPtr.reset();
    }
}

void Interface::init()
{
    locoClient = unitree::robot::g1::LocoClient();
    locoClient.Init();
    locoClient.SetTimeout(10.f);

    audioClient = unitree::robot::g1::AudioClient();
    audioClient.Init();
    audioClient.SetTimeout(10.0f);

    servoCmd = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>("rt/g1_comp_servo/cmd");
    servoCmd->msg_.cmds().resize(2);

    servoState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/g1_comp_servo/state");
    servoState->msg_.states().resize(2);

    armCmd = std::make_unique<unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::LowCmd_>>("rt/arm_sdk");
    armState = std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
    lowState = std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
    torsoImu = std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::IMUState_>>("rt/secondary_imu");
    detection = std::make_shared<unitree::robot::SubscriptionBase<DetectionModule::DetectionResults>>("detectionresults");
    odomState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>>("rt/odommodestate");
    locateResult = std::make_shared<unitree::robot::SubscriptionBase<LocationModule::LocationResult>>("rt/locationresults");
    lowStateThreadPtr = CreateRecurrentThreadEx("lowStateHandle", UT_CPU_ID_NONE, 2000, &Interface::lowStateHandle, this); // 500 hz

    sleep(1); // todo

    ballYawToPelvis = 0;
    ballDetected = false;
    ballDetected_counter = 0;
}

void Interface::lowStateHandle()
{
    homoMatPelvisToField = homoMatrix(rotMat2D(locateResult->msg_.robot2field_theta()),Vec2<double>(locateResult->msg_.robot2field_x(),locateResult->msg_.robot2field_y()));
    

    ballDetected_counter++;
    memcpy(&keyData, &lowState->msg_.wireless_remote()[0], 40);    
    Quat<double> quat;
    quat << torsoImu->msg_.quaternion()[0],  // x 
            torsoImu->msg_.quaternion()[1],  // y 
            torsoImu->msg_.quaternion()[2],  // z 
            torsoImu->msg_.quaternion()[3];  // w 
    rotMatPelvisToGlobal = quatToRotMat(quat);

    debug_rpy =  rotMatToRPY(rotMatPelvisToGlobal);

    if (!detection->msg_.results().empty())
    {
        for (const auto& result : detection->msg_.results())
        {
            if(result.class_name() == "Ball")
            {
                double wrist_yaw_angle = lowState->msg_.motor_state()[JointIndex::kWaistYaw].q();
                double servo0_angle = deg2rad(servoState->msg_.states()[0].q());
                double servo1_angle = -deg2rad(servoState->msg_.states()[1].q());
                compute_ball_position(rotMatPelvisToGlobal,wrist_yaw_angle,servo0_angle,servo1_angle,Vec3<double>(result.xyz()[0], result.xyz()[1], result.xyz()[2]));
           
                ball_offset_fov(0) = result.offset_fov()[0];
                ball_offset_fov(1) = result.offset_fov()[1];

                ball_offset(0) = result.offset()[0];
                ball_offset(1) = result.offset()[1];
            }
        
        }

    }

    if(ballDetected_counter >= 150)
    {
        ballDetected = false;
    }

}


void Interface::compute_ball_position(RotMat<double>rotMatPelvisToGlobal,double waist_yaw_q, double servo0_q, double servo1_q, Vec3<double> ball_position_in_cam)
{
    Vec3<double> _B2G_rpy = rotMatToRPY(rotMatPelvisToGlobal);       
    RotMat<double> rotMatPelvisToGlobal_no_yaw = rpyToRotMat(_B2G_rpy(0),_B2G_rpy(1),0);
    HomoMat<double> homoMatPelvisToWorldAligned = homoMatrix(Vec3<double>(0.0, 0.0, 0.0), rotMatPelvisToGlobal_no_yaw);
    HomoMat<double> homoMatTorsoToPelvis = homoMatrix(Vec3<double>(-0.0039635, 0.0, 0.044), rotz(waist_yaw_q));
    HomoMat<double> homoMat_head_servo_to_torso = homoMatrix(Vec3<double>(0.0039635, 0.0, -0.047), RotMat<double>(RotMat<double>::Identity()));
    RotMat<double> rotMat_xl330_to_head_servo = roty(0.039968)* rotz(servo0_q);
    HomoMat<double> homoMat_xl330_to_head_servo = homoMatrix(Vec3<double>(0.030518, 0.0, 0.52486), rotMat_xl330_to_head_servo);
    HomoMat<double> homoMat_d455_to_xl330 = homoMatrix(Vec3<double>(0.0295, 0.0, 0.013), roty(servo1_q));
    RotMat<double> rotMat_cam_to_d455 = roty(0.6981) * roty(1.5707) * rotz(-1.5707) ;
    HomoMat<double> homoMat_cam_to_d455 = homoMatrix(Vec3<double>(0.04061, 0.01000, -0.02207), rotMat_cam_to_d455);
    HomoMat<double> homoMat_ball_to_cam = homoMatrix(Vec3<double>(ball_position_in_cam(0), ball_position_in_cam(1), ball_position_in_cam(2)), RotMat<double>(RotMat<double>::Identity()) );

    HomoMat<double> homoMatBallToWorldAligned =  homoMatPelvisToWorldAligned * homoMatTorsoToPelvis * homoMat_head_servo_to_torso  *  homoMat_xl330_to_head_servo * homoMat_d455_to_xl330 * homoMat_cam_to_d455 * homoMat_ball_to_cam;

    double yaw_to_pelvis =  atan2(homoMatBallToWorldAligned(1,3),homoMatBallToWorldAligned(0,3));
    double x = homoMatBallToWorldAligned(0,3);
    double y = homoMatBallToWorldAligned(1,3);
    double z = homoMatBallToWorldAligned(2,3);
    double length = std::sqrt(x * x + y * y + z * z);
    if((length < 0.15) || (z >= -0.25)) // todo add score 
    {
        std::cout << "高度" << z << "/"
            << "，长度" << length << std::endl;
    }
    else
    {
        ballYawToPelvis = atan2(homoMatBallToWorldAligned(1,3),homoMatBallToWorldAligned(0,3));
        ballPositionInPelvis << homoMatBallToWorldAligned(0,3), homoMatBallToWorldAligned(1,3);
        ballPositionInField = dehomoVec(homoMatPelvisToField * homoVec(ballPositionInPelvis));

        double x_T = homoMatBallToWorldAligned(0,3);
        double y_T = homoMatBallToWorldAligned(1,3);
        double z_T = homoMatBallToWorldAligned(2,3);
        ball_range_selected = std::sqrt(x_T * x_T + y_T * y_T);

        ballDetected_counter = 0;
        ballDetected = true;
    }   
}