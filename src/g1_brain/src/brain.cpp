#include "g1_brain/brain.hpp"



RobotClient::RobotClient(rclcpp::Node* node) : node_(node), currentHeadYaw_(0.0), currentHeadPitch_(0.0) {
    RCLCPP_INFO(node_->get_logger(), "RobotClient created");
}

void RobotClient::init() {
    // 初始化LocoClient用于本体控制
    locoClient_ = std::make_unique<LocoClient>();
    locoClient_->Init();
    locoClient_->SetTimeout(10.0f);
    
    // 创建发布者 - 使用g1_comp_servo_service的话题（仅用于头部控制）
    motor_cmd_pub_ = node_->create_publisher<robot_interfaces::msg::MotorCmd>(
        "rt/g1_comp_servo/cmd", 10);
    
    // 创建订阅者 - 订阅舵机状态
    motor_states_sub_ = node_->create_subscription<robot_interfaces::msg::MotorStates>(
        "rt/g1_comp_servo/state", 10,
        std::bind(&RobotClient::motorStatesCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "RobotClient initialized with LocoClient for body control");
}

void RobotClient::setVelocity(double vx, double vy, double omega) {
    // 使用LocoClient进行本体移动控制
    RCLCPP_DEBUG(node_->get_logger(), "Set velocity: vx=%.2f, vy=%.2f, omega=%.2f", vx, vy, omega);
    
    // 限制速度范围，参考roboCup_sdk中的用法
    vx = std::clamp(vx, -1.0, 1.0);
    vy = std::clamp(vy, -1.0, 1.0);
    omega = std::clamp(omega, -1.0, 1.0);
    
    // 使用LocoClient的Move方法进行本体控制
    locoClient_->Move(vx, vy, omega);
}

void RobotClient::moveHead(double yaw, double pitch) {
    // 控制头部舵机
    RCLCPP_DEBUG(node_->get_logger(), "Move head: yaw=%.2f, pitch=%.2f", yaw, pitch);
    
    // 创建头部控制命令 - 使用正确的消息结构
    auto motorCmd = robot_interfaces::msg::MotorCmd();
    
    // 设置头部舵机命令 - 简化版本，只控制yaw
    motorCmd.mode = 1;  // 位置模式
    motorCmd.q = yaw;  // yaw角度
    motorCmd.kp = 500.0f;  // 位置增益
    motorCmd.kd = 300.0f;  // 速度增益
    motorCmd.tau = 0.0f;  // 力矩命令
    
    motor_cmd_pub_->publish(motorCmd);
}

void RobotClient::motorStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg) {
    // 处理舵机状态回调
    currentHeadYaw_ = msg->q;
    currentHeadPitch_ = 0.0;  // 简化版本，假设pitch为0
    
    RCLCPP_DEBUG(node_->get_logger(), "Head servo states: yaw=%.2f, pitch=%.2f", 
                 currentHeadYaw_, currentHeadPitch_);
}

BrainLog::BrainLog(rclcpp::Node* node) : node_(node), enabled_(false) {
    RCLCPP_INFO(node_->get_logger(), "BrainLog created");
}

void BrainLog::prepare() {
    // 占位符实现
    RCLCPP_INFO(node_->get_logger(), "BrainLog prepared");
}

void BrainLog::setTimeNow() {
    // 占位符实现
}

void BrainLog::setTimeSeconds(double time) {
    (void)time;  // 避免未使用参数警告
    // 占位符实现
}

G1Brain::G1Brain() : Node("g1_brain") {
    RCLCPP_INFO(this->get_logger(), "G1Brain node created");
    
    // 初始化组件
    config = std::make_shared<BrainConfig>();
    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>(this);
    tree = std::make_shared<BehaviorTree>(this);
    client = std::make_shared<RobotClient>(this);
    log = std::make_shared<BrainLog>(this);
}

void G1Brain::init() {
    declareParameters();
    loadConfig();
    
    // 初始化各个组件
    locator->init(fd, 3, 0.4, 0.5);
    tree->init();
    client->init();
    log->prepare();
    
    // 创建订阅者
    motorStatesSubscription = this->create_subscription<robot_interfaces::msg::MotorStates>(
        "servo/motor_states", 10, 
        std::bind(&G1Brain::motorStatesCallback, this, std::placeholders::_1));
    detectionsSubscription = this->create_subscription<robot_interfaces::msg::DetectionResults>(
        "detection_results", 10, 
        std::bind(&G1Brain::detectionsCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&G1Brain::odomCallback, this, std::placeholders::_1));
    lowstate_sub_ = this->create_subscription<robot_interfaces::msg::LowState>(
        "lowstate", 10, std::bind(&G1Brain::lowstateCallback, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_pose", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&G1Brain::mainLoop, this));
    RCLCPP_INFO(this->get_logger(), "G1Brain initialized");
}

void G1Brain::tick() {
    // 更新定位
    locator->update(data->markings);
    data->robotPoseToField = locator->getRobotPose();
    
    // 更新行为树状态
    tree->setRobotPose(data->robotPoseToField);
    tree->setBall(data->ball);
    tree->setBallDetected(data->ballDetected);
    tree->setMarkings(data->markings);
    tree->setOpponents(data->opponents);
    tree->setGoalposts(data->goalposts);
    
    // 执行行为树
    tree->tick();
    
    // 发送控制命令
    client->setVelocity(tree->getVelocityX(), tree->getVelocityY(), tree->getVelocityOmega());
    client->moveHead(tree->getHeadYaw(), tree->getHeadPitch());
    
    // 更新记忆
    updateMemory();
}

void G1Brain::declareParameters() {
    // 游戏相关参数
    this->declare_parameter<std::string>("game.field_size", "kid");
    this->declare_parameter<std::string>("game.playerStartPos", "left");
    this->declare_parameter<std::string>("game.location_mode", "normal");
    // 机器人相关参数
    this->declare_parameter<double>("robot.height", 1.3);
    this->declare_parameter<double>("robot.scale_factor", 1.4);
    this->declare_parameter<double>("robot.pitch_compensation", -45.0);
    this->declare_parameter<double>("robot.yaw_compensation", 0.0);
    // 记忆相关参数
    this->declare_parameter<double>("memory.ball_memory_length", 5.0);
    this->declare_parameter("field_length", 9.0);
    this->declare_parameter("field_width", 6.0);
    this->declare_parameter("goal_width", 2.6);
    this->declare_parameter("memory_length", 5.0);
}

void G1Brain::loadConfig() {
    config->field_size = this->get_parameter("game.field_size").as_string();
    config->playerStartPos = this->get_parameter("game.playerStartPos").as_string();
    config->location_mode = this->get_parameter("game.location_mode").as_string();

    config->height = this->get_parameter("robot.height").as_double();
    config->scale_factor = this->get_parameter("robot.scale_factor").as_double();
    config->pitch_compensation = this->get_parameter("robot.pitch_compensation").as_double();
    config->yaw_compensation = this->get_parameter("robot.yaw_compensation").as_double();

    config->memoryLength = this->get_parameter("memory.ball_memory_length").as_double();

    odometry_factor_ = config->scale_factor;
    servo_pitch_compensation_ = config->pitch_compensation;
    servo_yaw_compensation_ = config->yaw_compensation;
    servo_height_ = config->height;


    RCLCPP_INFO(this->get_logger(), "Configuration loaded");
}

void G1Brain::updateMemory() {
    // 更新记忆
    updateBallMemory();
}

void G1Brain::updateBallMemory() {
    Quat<double> quat;
    quat << torsoImu->msg_.quaternion()[0],  // x 
            torsoImu->msg_.quaternion()[1],  // y 
            torsoImu->msg_.quaternion()[2],  // z 
            torsoImu->msg_.quaternion()[3];  // w 
    rotMatPelvisToGlobal = quatToRotMat(quat);

    RotMat<double>rotMatPelvisToGlobal,double waist_yaw_q, double servo0_q, double servo1_q, Vec3<double> ball_position_in_cam;

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

    } 
}

void G1Brain::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = *msg;

    data->robotPoseToOdom.x = last_odom_.pose.pose.position.x * odometry_factor_;
    data->robotPoseToOdom.y = last_odom_.pose.pose.position.y * odometry_factor_;

    double qw = last_odom_.pose.pose.orientation.w;
    double qx = last_odom_.pose.pose.orientation.x;
    double qy = last_odom_.pose.pose.orientation.y;
    double qz = last_odom_.pose.pose.orientation.z;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    data->robotPoseToOdom.theta = std::atan2(siny_cosp, cosy_cosp);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Odometer information: (%.3f, %.3f, %.3f)",
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta);

    
}


void G1Brain::lowstateCallback(const robot_interfaces::msg::LowState::SharedPtr msg) {
    last_lowstate_ = *msg;
    double wrist_yaw_angle = rad2deg(last_lowstate_.motor_state[JointIndex::kWaistYaw].q);
    double servo_yaw_angle = last_lowstate_.motor_state[JointIndex::kLeftHipYaw].q;
    double servo_pitch_angle = last_lowstate_.motor_state[JointIndex::kLeftHipPitch].q + servo_pitch_compensation_;
    Pose p_eye2base(0, -servo_height_, 0,
                    deg2rad(servo_pitch_angle),
                    -deg2rad(wrist_yaw_angle) - deg2rad(servo_yaw_angle),
                    0);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Servo information: wrist_yaw_angle(%.2f), servo_yaw_angle(%.2f), servo_pitch_angle(%.2f)",
        wrist_yaw_angle, servo_yaw_angle, servo_pitch_angle);
}

void G1Brain::motorStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg) {
    // 处理舵机状态回调
    RCLCPP_DEBUG(this->get_logger(), "Received motor states");
}


void G1Brain::mainLoop() {
    // 4. 计算并发布定位结果
    if (locator->odomCalibrated) {
        calibrateOdom
        transCoord(
            data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
            locator->odomToField.x, locator->odomToField.y, locator->odomToField.theta,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "== Final RobotToField: (%.3f, %.3f, %.3f)",
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

        geometry_msgs::msg::Pose2D pose_msg;
        pose_msg.x = data->robotPoseToField.x;
        pose_msg.y = data->robotPoseToField.y;
        pose_msg.theta = data->robotPoseToField.theta;
        pose_pub_->publish(pose_msg);
    }
}



void G1Brain::detectionsCallback(const robot_interfaces::msg::DetectionResults::SharedPtr msg) {
    // 1. 解析检测结果
    auto gameObjects = getGameObjects(*msg);

    // 2. 分类处理
    std::vector<GameObject> balls, goalPosts, persons, robots, markings;
    for (const auto& obj : gameObjects) {
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);
        if (obj.label == "Person")
            persons.push_back(obj);
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "L" || obj.label == "T" || obj.label == "X")
            markings.push_back(obj);
    }

    // 3. 处理球
    detectProcessBalls(balls);

    // 4. 处理标记
    detectProcessMarkings(markings);

    // 5. 更新其他数据
    data->opponents = robots;
    data->goalposts = goalPosts;

    // 6. 记录检测结果（可选）
    last_detections_ = msg->results;

    RCLCPP_DEBUG(this->get_logger(), "Processed detections: %zu balls, %zu markings, %zu robots, %zu goalposts",
                 balls.size(), markings.size(), robots.size(), goalPosts.size());
}


std::vector<GameObject> G1Brain::getGameObjects(
    const std::vector<robot_interfaces::msg::DetectionResult>& detection_results,
    const Pose& p_eye2base,
    const Pose& robotPoseToField)
{
    std::vector<GameObject> gameObjects;
    for (const auto& result : detection_results) {
        GameObject gObj;

        gObj.label = result.class_name;

        gObj.boundingBox.xmin = result.box[0];
        gObj.boundingBox.ymin = result.box[1];
        gObj.boundingBox.xmax = result.box[2];
        gObj.boundingBox.ymax = result.box[3];
        gObj.confidence = result.score * 100; // 与第一个函数一致

        // Get object pose in camera coord
        Pose pose(result.xyz[0], result.xyz[1], result.xyz[2], 0, 0, 0);

        // Get object pose in robot coord
        Pose obj_pose = p_eye2base * pose;
        auto obj_trans = obj_pose.getTranslation();

        gObj.posToRobot.x = obj_trans[2];
        gObj.posToRobot.y = -obj_trans[0];
        gObj.posToRobot.z = obj_trans[1]; // 可选，按需保留

        gObj.range = std::hypot(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(1.3, gObj.range); // 1.3为摄像头高度，可参数化

        // Get object pose in field coord
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);

        // 可选：记录时间戳
        gObj.timePoint = this->now();

        gameObjects.push_back(gObj);
    }
    return gameObjects;
}


void G1Brain::detectProcessBalls(const std::vector<GameObject>& ballObjs) {
    if (!ballObjs.empty()) {
        // 选择置信度最高的球
        auto bestBall = std::max_element(ballObjs.begin(), ballObjs.end(),
            [](const GameObject& a, const GameObject& b) {
                return a.confidence < b.confidence;
            });
        
        data->ball = Ball(*bestBall);
        data->ballDetected = true;
        
        // 计算球相对于机器人的角度
        data->robotBallAngleToField = atan2(data->ball.posToRobot.y, data->ball.posToRobot.x);
        
        RCLCPP_DEBUG(this->get_logger(), "Ball detected: confidence=%.2f, range=%.2f", 
                     data->ball.confidence, data->ball.range);
    } else {
        data->ballDetected = false;
    }
}

void G1Brain::detectProcessMarkings(const std::vector<GameObject>& markingObjs) {
    const double confidenceValve = 0.5;
    data->markings.clear();
    for (const auto& marking : markingObjs) {
        if (marking.confidence < confidenceValve)
            continue;
        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)
            continue;
        data->markings.push_back(marking);
    }
}


void G1Brain::calibrateOdom(double x, double y, double theta)
{
    // Calculate odomToField according to robotToOdom(by odometry) and robotToField(by locator)
    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(robotPoseToOdom.theta) * robotPoseToOdom.x - sin(robotPoseToOdom.theta) * robotPoseToOdom.y;
    y_or = sin(robotPoseToOdom.theta) * robotPoseToOdom.x - cos(robotPoseToOdom.theta) * robotPoseToOdom.y;
    theta_or = -robotPoseToOdom.theta;

    transCoord(x_or, y_or, theta_or,
                x, y, theta,
                odomToField.x, odomToField.y, odomToField.theta);

}
