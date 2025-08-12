#include "brain.h"


G1Brain::G1Brain() : Node("g1_brain_node") {
    RCLCPP_INFO(this->get_logger(), "G1Brain node created");
    declare_parameter<std::string>("game.field_type", "");
    declare_parameter<std::string>("game.playerStartPos", "left");

    declare_parameter<double>("robot.height", 1.3);
    declare_parameter<double>("robot.scale_factor", 1.4);
    declare_parameter<double>("robot.pitch_compensation", -45.0);
    declare_parameter<double>("robot.yaw_compensation", 0.0);

    declare_parameter<double>("memory.ball_memory_length", 5.0);
}

void G1Brain::init() {
    config = std::make_shared<BrainConfig>();
    loadConfig();
    
    data = std::make_shared<BrainData>();
    // locator = std::make_shared<Locator>();

    // tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);

    // 初始化粒子滤波定位器
    locator->init(config->fieldDimensions, 3, 0.4, 0.5);

    // 构建 BehaviorTree
    tree = std::make_shared<BrainTree>(this); // ← 创建 BrainTree
    tree->init(); // ← 初始化行为树

    // 初始化 client
    client->init();


    data->lastSuccessfulLocalizeTime = get_clock()->now();
    
    // 创建订阅者
    servoStatesSubscription = this->create_subscription<robot_interfaces::msg::MotorStates>(
        "servo/motor_states", 10, std::bind(&G1Brain::servoStatesCallback, this, std::placeholders::_1));
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

    servo_states_sub_ = this->create_subscription<robot_interfaces::msg::MotorStates>(
        "rt/g1_comp_servo/state", 10,std::bind(&G1Brain::servoStatesCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&G1Brain::mainLoop, this));
    RCLCPP_INFO(this->get_logger(), "G1Brain initialized");
}

void G1Brain::tick() {
    
    //执行行为树
    tree->tick();
    
    // 更新记忆
    updateMemory();
}


void G1Brain::loadConfig() {
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.playerStartPos", config->playerStartPos);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("robot.height", config->height);
    get_parameter("robot.scale_factor", config->scale_factor);
    get_parameter("robot.pitch_compensation", config->pitch_compensation);
    get_parameter("robot.yaw_compensation", config->yaw_compensation);
    get_parameter("memory.ball_memory_length", config->ball_memory_length);

    odometry_factor_ = config->scale_factor;
    servo_pitch_compensation_ = config->pitch_compensation;
    servo_yaw_compensation_ = config->yaw_compensation;
    servo_height_ = config->height;
    config->handle();

    RCLCPP_INFO(this->get_logger(), "Configuration loaded");
}

double G1Brain::msecsSince(rclcpp::Time time)
{
    return (this->get_clock()->now() - time).nanoseconds() / 1e6;
}


void G1Brain::updateMemory() {
    // 更新记忆
    updateBallMemory();
}

void G1Brain::updateBallMemory() {
    double waist_yaw_angle = low_state.motor_state[JointIndex::kWaistYaw].states[0].q;
    double servo0_angle = deg2rad(motor_states.states[0].q);
    double servo1_angle = -deg2rad(motor_states.states[1].q);

    // 用 posToRobot 构造 Vec2
    Vec3<double> ball_pos_in_cam(
        data->ballPositionInPelvis[0],
        data->ballPositionInPelvis[1],
        0.0
    );

    // 假设 compute_ball_position 返回 Vec3<double>
    Vec3<double> ball_global = data->computeBallPosition(
        data->rotMatPelvisToGlobal,
        waist_yaw_angle,
        servo0_angle,
        servo1_angle,
        ball_pos_in_cam
    );
    data->homoMatPelvisToField = homoMatrix(rotMat2D(data->robotPoseToField.theta), Vec2<double>(data->robotPoseToField.x,data->robotPoseToField.y));
    double yaw_to_pelvis =  atan2(data->homoMatBallToWorldAligned(1,3),data->homoMatBallToWorldAligned(0,3));
    double x = data->homoMatBallToWorldAligned(0,3);
    double y = data->homoMatBallToWorldAligned(1,3);
    double z = data->homoMatBallToWorldAligned(2,3);
    // 计算长度和高度
    double length = std::sqrt(ball_global[0] * ball_global[0] + ball_global[1] * ball_global[1]);

    if ((length < 0.15) || (z >= -0.25)) // todo add score 
    {
        std::cout << "高度" << z << "/"
                  << "，长度" << length << std::endl;
    }
    else
    {
        // 这里假设 homoMatBallToWorldAligned 已经被正确赋值
        data->ballYawToPelvis = atan2(data->homoMatBallToWorldAligned(1,3), data->homoMatBallToWorldAligned(0,3));
        data->ballPositionInPelvis << data->homoMatBallToWorldAligned(0,3), data->homoMatBallToWorldAligned(1,3);
        data->ballPositionInField = dehomoVec(data->homoMatPelvisToField * homoVec(data->ballPositionInPelvis));
        data->ballRange = sqrt(pow(data->ballPositionInPelvis(0), 2) + pow(data->ballPositionInPelvis(1), 2));
        double x_T = data->homoMatBallToWorldAligned(0,3);
        double y_T = data->homoMatBallToWorldAligned(1,3);
        double z_T = data->homoMatBallToWorldAligned(2,3);
        data->ball_range_selected = std::sqrt(x_T * x_T + y_T * y_T);
        data->ballPitchToPelvis = asin(config->Height / data->ballRange);
    } 
}

vector<double> G1Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY; // 球门柱在球场中的坐标

    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;

    // 如果看到了对方球门, 则使用看到的位置, 可以抵消 odom 的误差
    for (int i = 0; i < data->goalposts.size(); i++)
    {
        auto post = data->goalposts[i];
        if (post.info == "oppo-left")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.info == "oppo-right")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ballPositionInField[1], leftX - data->ballPositionInField[0]);
    const double theta_r = atan2(rightY + margin - data->ballPositionInField[1], rightX - data->ballPositionInField[0]);

    vector<double> vec = {theta_l, theta_r};
    return vec;
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
    double waist_yaw_angle = rad2deg(last_lowstate_.motor_state[JointIndex::kWaistYaw].states[0].q);
    double servo_yaw_angle = last_lowstate_.motor_state[JointIndex::kLeftHipYaw].states[0].q;
    double servo_pitch_angle = last_lowstate_.motor_state[JointIndex::kLeftHipPitch].states[0].q + servo_pitch_compensation_;
    // 不要再写 Pose p_eye2base(...)，而是直接赋值
    p_eye2base = Pose(0, -servo_height_, 0,
                    deg2rad(servo_pitch_angle),
                    -deg2rad(waist_yaw_angle) - deg2rad(servo_yaw_angle),
                    0);
    data->cur_imu.quaternion [0]= last_lowstate_.imu_state.quaternion[0];
    data->cur_imu.quaternion [1]= last_lowstate_.imu_state.quaternion[1];
    data->cur_imu.quaternion [2]= last_lowstate_.imu_state.quaternion[2];
    data->cur_imu.quaternion [3]= last_lowstate_.imu_state.quaternion[3];
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Servo information: waist_yaw_angle(%.2f), servo_yaw_angle(%.2f), servo_pitch_angle(%.2f)",
        waist_yaw_angle, servo_yaw_angle, servo_pitch_angle);
}

void G1Brain::servoStatesCallback(const robot_interfaces::msg::MotorStates::SharedPtr msg) {
    // if (!msg->states.empty()) {
    //     client->currentHeadYaw_ = msg->states[0].q;
    //     client->currentHeadPitch_ = 0.0;
    // }// 处理舵机状态回调
    RCLCPP_DEBUG(this->get_logger(), "Received motor states");
}


void G1Brain::mainLoop() {
    RCLCPP_INFO(this->get_logger(), "mainLoop() called");

    if (!data) {
        RCLCPP_ERROR(this->get_logger(), "data is nullptr in mainLoop!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "odomCalibrated: %d", data->odomCalibrated);

    if (data->odomCalibrated) {
        transCoord(
            data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
            data->odomToField.x, data->odomToField.y, data->odomToField.theta,
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
    auto gameObjects = getGameObjects(msg->results, p_eye2base, data->robotPoseToField);
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
    const Pose2D& robotPoseToField)
{
    std::vector<GameObject> gameObjects;
    for (const auto &result : detection_results) {
        GameObject gObj;

        gObj.label = result.class_name;

        gObj.boundingBox.xmin = result.box[0];
        gObj.boundingBox.ymin = result.box[1];
        gObj.boundingBox.xmax = result.box[2];
        gObj.boundingBox.ymax = result.box[3];
        gObj.confidence = result.score * 100;

        // Get object pose in camera coord
        Pose pose = Pose(result.xyz[0], result.xyz[1], result.xyz[2], 0, 0, 0);

        // Get object pose in robot coord
        Pose obj_pose = p_eye2base * pose;
        auto obj_trans = obj_pose.getTranslation();

        gObj.posToRobot.x = obj_trans[2];
        gObj.posToRobot.y = -obj_trans[0];

        gObj.range = std::hypot(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(1.3, gObj.range);

        // Get object pose in field coord
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);

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
        
        data->ball = GameObject(*bestBall);
        data->ballDetected = true;
        
        // 计算球相对于机器人的角度
        data->robotBallAngleToField = atan2(data->ball.posToRobot.y, data->ball.posToRobot.x);
        
        RCLCPP_DEBUG(this->get_logger(), "Ball detected: confidence=%.2f, range=%.2f", 
                    data->ball.confidence, data->ball.range);
    } else {
        data->ballDetected = false;
    }
    data->robotBallAngleToField = atan2(data->ballPositionInField(1) - data->robotPoseToField.y , data->ballPositionInField(0) - data->robotPoseToField.x);
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
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;

    transCoord(x_or, y_or, theta_or,
                x, y, theta,
                data->odomToField.x, data->odomToField.y, data->odomToField.theta);

}
