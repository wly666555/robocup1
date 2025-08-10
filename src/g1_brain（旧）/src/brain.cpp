#include "g1_brain/brain.hpp"
#include "brain_data.h"

G1Brain::G1Brain() : Node("g1_brain") {
    RCLCPP_INFO(this->get_logger(), "G1Brain node created");
    
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "");

    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.95);
    declare_parameter<double>("robot.yaw_offset", 0.1);

}

void G1Brain::init() {
    config = std::make_shared<BrainConfig>();
    loadConfig();
    
    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();

    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);

    // 初始化粒子滤波定位器
    locator->init(config->fieldDimensions, 3, 0.4, 0.5);

    // 构建 BehaviorTree
    tree->init();

    // 初始化 client
    client->init();
    
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
    motor_cmd_pub_ = this->create_publisher<robot_interfaces::msg::MotorCmds>(
        "rt/g1_comp_servo/cmd", 10);

    motor_states_sub_ = this->create_subscription<robot_interfaces::msg::MotorStates>(
        "rt/g1_comp_servo/state", 10,std::bind(&G1Brain::motorStatesCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&G1Brain::mainLoop, this));
    RCLCPP_INFO(this->get_logger(), "G1Brain initialized");
}

void G1Brain::tick() {

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


void G1Brain::loadConfig() {
    get_parameter("game.field_type", config->fieldType);
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
    config->handle();

    RCLCPP_INFO(this->get_logger(), "Configuration loaded");
}

void G1Brain::updateMemory() {
    // 更新记忆
    updateBallMemory();
}

void G1Brain::updateBallMemory() {
    Vec3<double> ball_global_pos = data->computeBallPosition(
    rotMatPelvisToGlobal,
    waist_yaw_q,
    servo0_q,
    servo1_q,
    ball_position_in_cam
    );
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
    data->cur_imu.quaternion [0]= last_lowstate_.imu_state.quaternion[0];
    data->cur_imu.quaternion [1]= last_lowstate_.imu_state.quaternion[1];
    data->cur_imu.quaternion [2]= last_lowstate_.imu_state.quaternion[2];
    data->cur_imu.quaternion [3]= last_lowstate_.imu_state.quaternion[3];
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Servo information: wrist_yaw_angle(%.2f), servo_yaw_angle(%.2f), servo_pitch_angle(%.2f)",
        wrist_yaw_angle, servo_yaw_angle, servo_pitch_angle);
}

void G1Brain::motorStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg) {
    if (!msg->states.empty()) {
        currentHeadYaw_ = msg->states[0].q;
        currentHeadPitch_ = 0.0;
    }// 处理舵机状态回调
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
