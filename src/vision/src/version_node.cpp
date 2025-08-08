#include "version_node.h"

// ROS2节点
FootballDetectNode::FootballDetectNode(const rclcpp::NodeOptions & options)
: Node("football_detect")
{
    // 声明参数
    this->declare_parameter<std::string>("engine_file", "/home/unitree/wly666/vision/vision/weight/weight.engine");
    this->declare_parameter<bool>("show_image", false);
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<double>("horizontal_fov", 86.0);
    this->declare_parameter<double>("vertical_fov", 57.0);

    // 获取参数
    std::string engine_file = this->get_parameter("engine_file").as_string();
    bool show_image = this->get_parameter("show_image").as_bool();
    int image_width = this->get_parameter("image_width").as_int();
    int image_height = this->get_parameter("image_height").as_int();
    int fps = this->get_parameter("fps").as_int();
    double horizontal_fov = this->get_parameter("horizontal_fov").as_double();
    double vertical_fov = this->get_parameter("vertical_fov").as_double();

    // 初始化YOLO
    RCLCPP_INFO(this->get_logger(), "Before YOLO init: %s", engine_file.c_str());
    model_ = std::make_unique<YOLO>(engine_file, logger);
    RCLCPP_INFO(this->get_logger(), "After YOLO init");

    // 初始化相机
    camera_ = std::make_unique<CameraController>(image_width, image_height, fps);
    if (!camera_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Camera initialization failed!");
        throw std::runtime_error("Camera initialization failed");
    }
    camera_connected_ = true;

    //创建ROS2发布者
    publisher_ = this->create_publisher<robot_interfaces::msg::DetectionResults>("detection_results", 10);

    //启动线程
    processing_thread_ = std::thread(&FootballDetectNode::processing_loop, this,
                                     show_image, image_width, image_height,
                                     horizontal_fov, vertical_fov);
    reconnect_thread_ = std::thread(&FootballDetectNode::reconnect_monitor, this);
}

FootballDetectNode::~FootballDetectNode(){
    running_ = false;
    if (processing_thread_.joinable()) processing_thread_.join();
    if (reconnect_thread_.joinable()) reconnect_thread_.join();
    camera_->shutdown();
}

void FootballDetectNode::publish_detection_results(const std::vector<Detection> &objects,
                               double image_width, double image_height)
{
    robot_interfaces::msg::DetectionResults msg;
    for (const auto& obj : objects) {
        robot_interfaces::msg::DetectionResult result;
        result.class_id = std::to_string(obj.class_id);
        result.class_name = CLASS_NAMES[obj.class_id];
        result.score = obj.conf;
        result.box = {
            static_cast<float>(obj.bbox.x),
            static_cast<float>(obj.bbox.y),
            static_cast<float>(obj.bbox.x + obj.bbox.width),
            static_cast<float>(obj.bbox.y + obj.bbox.height)
        };
        float u = obj.bbox.x + obj.bbox.width / 2.0f;
        float v = obj.bbox.y + obj.bbox.height / 2.0f;
        float offset_x = u - image_width / 2;
        float offset_y = v - image_height / 2;
        result.xyz = {obj.XYZ.x, obj.XYZ.y, obj.XYZ.z};
        result.offset = {offset_x, offset_y};
        auto yaw_pitch = safe_calculate_angles(offset_x, offset_y, image_width, image_height, 86.0f, 57.0f);
        result.offset_fov = {yaw_pitch.first, yaw_pitch.second};
        msg.results.push_back(result);
    }
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published detection results");
}

void FootballDetectNode::processing_loop(bool show_image, int image_width, int image_height,
                     double horizontal_fov, double vertical_fov)
{
    int specific_class_id = 0;
    float conf_flag = 0.6;
    rs2::align align(RS2_STREAM_COLOR);
    while (running_) {
        if (!camera_connected_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            robot_interfaces::msg::DetectionResults msg;
            robot_interfaces::msg::DetectionResult result;
            result.class_id = std::to_string(-1);
            result.class_name = "";
            result.score = -1.0f;
            result.box = {-1, -1, -1, -1};
            result.xyz = {-1, -1, -1};
            result.offset = {-1, -1};
            result.offset_fov = {-1, -1};
            msg.results.push_back(result);
            publisher_->publish(msg);
            continue;
        }
        auto frameset = camera_->poll_frames();
        if (!frameset) {
            camera_connected_ = false;
            continue;
        }
        auto aligned = align.process(frameset);
        auto color_frame = aligned.get_color_frame();
        auto depth_frame = aligned.get_depth_frame();
        cv::Mat image(cv::Size(image_width, image_height), CV_8UC3,
                      (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(image_width, image_height), CV_16UC1, (void*)depth_frame.get_data());
        if (image.empty()) break;
        std::vector<Detection> objects;
        model_->preprocess(image);
        auto start = std::chrono::system_clock::now();
        model_->infer();
        auto end = std::chrono::system_clock::now();
        model_->postprocess(objects, depth_image, camera_->intrinsics, conf_flag, specific_class_id);
        model_->draw(image, objects);
        publish_detection_results(objects, image.cols, image.rows);
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        RCLCPP_INFO(this->get_logger(), "cost %2.4lf ms", tc);
        if (show_image) {
            cv::imshow("prediction", image);
            if (cv::waitKey(1) == 27) break;
        }
    }
}
void FootballDetectNode::reconnect_monitor() {
    while (running_) {
        if (camera_connected_ || reconnect_attempts_ >= MAX_RECONNECT) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        RCLCPP_WARN(this->get_logger(), "Attempting to reconnect (%d/%d)...",
                    ++reconnect_attempts_, MAX_RECONNECT);
        camera_->shutdown();
        if (camera_->initialize()) {
            reconnect_attempts_ = 0;
            camera_connected_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully! Serial number: %s",
                        camera_->get_serial().c_str());
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
}

