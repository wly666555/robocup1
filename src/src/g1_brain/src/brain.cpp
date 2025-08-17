#include "brain.h"

using namespace std;
using std::placeholders::_1;

G1Brain::G1Brain() : Node("g1_brain_node") {
    RCLCPP_INFO(this->get_logger(), "G1Brain node created");

    declare_parameter<string>("game.field_type", "");
    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.playerStartPos", "");


    declare_parameter<double>("robot.height", 1.3);
    declare_parameter<double>("robot.scale_factor", 1.4);
    declare_parameter<double>("robot.pitch_compensation", -45.0);
    declare_parameter<double>("robot.yaw_compensation", 0.0);

    declare_parameter<double>("robot.yaw_limit_min", -50.0);
    declare_parameter<double>("robot.yaw_limit_max", 50.0);
    declare_parameter<double>("robot.pitch_limit_min", -20.0);
    declare_parameter<double>("robot.pitch_limit_max", 85.0);

    declare_parameter<double>("memory.memoryLength", 5.0);
    declare_parameter<string>("tree_file_path", "");

}

void G1Brain::init() {
    config = std::make_shared<BrainConfig>();
    loadConfig();
    
    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();

    client = std::make_shared<RobotClient>(this);

    // 初始化粒子滤波定位器
    locator->init(config->fieldDimensions, 4, 0.4, 0.5);

    // 构建 BehaviorTree
    tree = std::make_shared<BrainTree>(this); // ← 创建 BrainTree
    tree->init(); // ← 初始化行为树

    // 初始化 client
    client->init();


    data->lastSuccessfulLocalizeTime = get_clock()->now();
    
    // 创建订阅者
    servoStatesSubscription = this->create_subscription<robot_interfaces::msg::MotorStates>(
        "/servo/motor_state", 10, std::bind(&G1Brain::servoStatesCallback, this, std::placeholders::_1));

    detectionsSubscription = this->create_subscription<robot_interfaces::msg::DetectionResults>(
        "detection_results", 10, std::bind(&G1Brain::detectionsCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "/lf/odommodestate", 10,std::bind(&G1Brain::odomCallback, this, std::placeholders::_1));

    lowstate_sub_ = this->create_subscription<robot_interfaces::msg::LowState>(
        "/lowstate", 10,std::bind(&G1Brain::lowstateCallback, this, std::placeholders::_1));

    // pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_pose", 10);

    joystick_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,std::bind(&G1Brain::joystickCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "G1Brain initialized");
}

void G1Brain::tick() {
    
    //执行行为树
    tree->tick();
    
    // 更新记忆
    updateMemory();
}


void G1Brain::loadConfig() {
    this->get_parameter("game.field_type", config->fieldType);
    this->get_parameter("game.playerStartPos", config->playerStartPos);
    this->get_parameter("game.player_role", config->playerRole);
    this->get_parameter("robot.height", config->height);
    this->get_parameter("robot.scale_factor", config->scale_factor);
    this->get_parameter("robot.pitch_compensation", config->pitch_compensation);
    this->get_parameter("robot.yaw_compensation", config->yaw_compensation);
    this->get_parameter("robot.yaw_limit_min", config->yaw_limit_min);
    this->get_parameter("robot.yaw_limit_max", config->yaw_limit_max);
    this->get_parameter("robot.pitch_limit_min", config->pitch_limit_min);
    this->get_parameter("robot.pitch_limit_max", config->pitch_limit_max);
    this->get_parameter("memory.memoryLength", config->memoryLength);
    this->get_parameter("tree_file_path", config->treeFilePath);


    RCLCPP_INFO(this->get_logger(), "height: %s",config->playerRole.c_str());
    RCLCPP_INFO(this->get_logger(), "game.field_type loaded: %s", config->playerStartPos.c_str());
    RCLCPP_INFO(this->get_logger(), "tree_file_path: %s", config->treeFilePath.c_str());

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
    // headYaw = yaw, headPitch = pitch; clamp using config limits (degrees)
    double yaw_deg = std::clamp(rad2deg(data->headYaw), config->yaw_limit_min, config->yaw_limit_max);
    double pitch_deg = std::clamp(rad2deg(data->headPitch), config->pitch_limit_min, config->pitch_limit_max);
    // Use degrees locally; convert at call site
    double servo0_deg = yaw_deg;     // yaw (servo0)
    double servo1_deg = pitch_deg;   // pitch (servo1)

    // 用 posToRobot 构造 Vec2
    Vec3<double> ball_pos_in_cam(
        data->ball.posToRobot.x,
        data->ball.posToRobot.y,
        0.0
    );

    // 假设 compute_ball_position 返回 Vec3<double>
    Vec3<double> ball_global = data->computeBallPosition(
        data->rotMatPelvisToGlobal,
        data->waist_yaw_angle,           // 直接使用弧度值
        deg2rad(servo0_deg),            // 这是弧度
        -deg2rad(servo1_deg),           // 这是弧度
        ball_pos_in_cam
);
    data->homoMatPelvisToField = homoMatrix(rotMat2D(data->robotPoseToField.theta), Vec2<double>(data->robotPoseToField.x,data->robotPoseToField.y));
    double yaw_to_pelvis =  atan2(data->homoMatBallToWorldAligned(1,3),data->homoMatBallToWorldAligned(0,3));
    double x = data->homoMatBallToWorldAligned(0,3);
    double y = data->homoMatBallToWorldAligned(1,3);
    double z = data->homoMatBallToWorldAligned(2,3);
    // 计算长度和高度
    double length = std::sqrt(ball_global[0] * ball_global[0] + ball_global[1] * ball_global[1]);
    Vec2<double> ballToRobot;
    
    // 这里假设 homoMatBallToWorldAligned 已经被正确赋值
    data->ball.yawToRobot = atan2(data->homoMatBallToWorldAligned(1,3), data->homoMatBallToWorldAligned(0,3));
    data->ball.posToRobot.x = data->homoMatBallToWorldAligned(0,3);
    data->ball.posToRobot.y = data->homoMatBallToWorldAligned(1,3);
    ballToRobot[0] = data->ball.posToRobot.x;
    ballToRobot[1] = data->ball.posToRobot.y;
    auto posToField = dehomoVec(data->homoMatPelvisToField * homoVec(ballToRobot));
    data->ball.posToField.x = posToField[0];
    data->ball.posToField.y = posToField[1];

    double x_T = data->homoMatBallToWorldAligned(0,3);
    double y_T = data->homoMatBallToWorldAligned(1,3);
    double z_T = data->homoMatBallToWorldAligned(2,3);
    data->ball.range = std::sqrt(x_T * x_T + y_T * y_T);
    tree->setEntry<double>("ball_range", data->ball.range);  
    data->ball.pitchToRobot = asin(config->height / data->ball.range);

    if (get_clock()->now().seconds() - data->ball.timePoint.seconds() > config->memoryLength)
    {
        tree->setEntry<bool>("ball_location_known", false);
        data->ballDetected = false;
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

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

void G1Brain::odomCallback(const std::shared_ptr<unitree_go::msg::SportModeState> msg) {
    // 位置
    data->robotPoseToOdom.x = msg->position[0] * config->scale_factor;
    data->robotPoseToOdom.y = msg->position[1] * config->scale_factor;

    // 四元数（从IMUState里取）
    double qw = msg->imu_state.quaternion[0];
    double qx = msg->imu_state.quaternion[1];
    double qy = msg->imu_state.quaternion[2];
    double qz = msg->imu_state.quaternion[3];

    // 四元数转yaw
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    data->robotPoseToOdom.theta = std::atan2(siny_cosp, cosy_cosp);

    // 或者直接用欧拉角yaw
    // data->robotPoseToOdom.theta = msg->imu_state.rpy[2];

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "Odometer information: (%.3f, %.3f, %.3f)",
    //     data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta);

    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "== Final RobotToField: (%.3f, %.3f, %.3f)",
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
}



void G1Brain::lowstateCallback(const std::shared_ptr<robot_interfaces::msg::LowState> msg) {
    data->waist_yaw_angle = msg->motor_state[JointIndex::kWaistYaw].states[0].q;

    data->cur_imu.quaternion [0]= msg->imu_state.quaternion[0];
    data->cur_imu.quaternion [1]= msg->imu_state.quaternion[1];
    data->cur_imu.quaternion [2]= msg->imu_state.quaternion[2];
    data->cur_imu.quaternion [3]= msg->imu_state.quaternion[3];

}

void G1Brain::servoStatesCallback(const std::shared_ptr<robot_interfaces::msg::MotorStates> msg) {

    // 解析头部舵机 yaw/pitch（按照约定：states[0]=yaw, states[1]=pitch，单位为度）

    float yaw_deg = msg->states[0].q   + config->yaw_compensation;
    float pitch_deg = msg->states[1].q + config->pitch_compensation;

    // 转为弧度写入运行时数据
    data->headYaw = deg2rad(static_cast<double>(yaw_deg));
    data->headPitch = deg2rad(static_cast<double>(pitch_deg));

}


void G1Brain::joystickCallback(const std::shared_ptr<unitree_go::msg::WirelessController> msg){

    // RCLCPP_INFO(this->get_logger(), "Wireless controller -- lx: %f; ly: %f; rx: %f; ry: %f; key value: %d",msg->lx, msg->ly, msg->rx, msg->ry, msg->keys);

    uint16_t keys = msg->keys;


    if (keys & 4096) { // 4096 (0x1000) 表示 Up 按钮被按下
        if (keys & 2048) { 
            RCLCPP_INFO(this->get_logger(), "Up and Y buttons state1");
            tree->setEntry<int>("control_state", 1);
            this->client->SetVelocity(0., 0., 0.);
            this->client->moveHead(0., 0.);
        }

        if (keys & 1024) { 
            RCLCPP_INFO(this->get_logger(), "Up and X buttons state2");
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
        }

        if (keys & 256) { 
            RCLCPP_INFO(this->get_logger(), "Up and A buttons state3");
            tree->setEntry<int>("control_state", 3);
        }

        if (keys & 512) { 
            RCLCPP_INFO(this->get_logger(), "Up and B buttons state4");
            tree->setEntry<int>("control_state", 4);
        }
    }

}


void G1Brain::detectionsCallback(const std::shared_ptr<robot_interfaces::msg::DetectionResults> msg) {

    //确定servo height 的数值
    p_eye2base = Pose(0, -config->height, 0, data->headPitch, -data->waist_yaw_angle - data->headYaw, 0);

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
    //ball
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


std::vector<GameObject> G1Brain::getGameObjects(const std::vector<robot_interfaces::msg::DetectionResult>& detection_results, const Pose& p_eye2base, const Pose2D& robotPoseToField)
{
    std::vector<GameObject> gameObjects;

    rclcpp::Time current_time = rclcpp::Clock().now();

    for (const auto &result : detection_results) {
        GameObject gObj;

        gObj.timePoint = current_time;
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

void G1Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    // Parameters
    const double confidenceValve = 0.35;        // If the confidence is lower than this threshold, it is considered not a ball (note that the target confidence passed in by the detection module is currently all > 0.2).
    const double pitchLimit = deg2rad(0);       // When the pitch of the ball relative to the front of the robot (downward is positive) is lower than this value, it is considered not a ball. (Because the ball won't be in the sky.)
    const int timeCountThreshold = 5;           // Only when the ball is detected in consecutive several frames is it considered a ball. This is only used in the ball-finding strategy.
    const unsigned int detectCntThreshold = 3;  // The maximum count. Only when the target is detected in such a number of frames is it considered that the target is truly identified. (Currently only used for ball detection.)
    const unsigned int diffConfidThreshold = 4; // The threshold for the difference times between the tracked ball and the high-confidence ball. After reaching this threshold, the high-confidence ball will be adopted.

    double bestConfidence = 0;
    double minPixDistance = 1.e4;
    int indexRealBall = -1;  // Which ball is considered to be the real one. -1 indicates that no ball has been detected.
    int indexTraceBall = -1; // Track the ball according to the pixel distance. -1 indicates that no target has been tracked.

    // Find the most likely real ball.
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];

        // Judgment: If the confidence is too low, it is considered a false detection.
        if (ballObj.confidence < confidenceValve)
            continue;

        // Prevent the lights in the sky from being recognized as balls.
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 10.0)
            continue;

        // Find the one with the highest confidence among the remaining balls.
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    if (indexRealBall >= 0)
    {
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];

        tree->setEntry<bool>("ball_location_known", true);
    }
    else
    {
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        data->ball.confidence = 0;
    }

    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
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


    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "odomToField information: (%.3f, %.3f, %.3f)",
        data->odomToField.x, data->odomToField.y, data->odomToField.theta);
}
