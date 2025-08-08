#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <vector>
#include <cassert>
#include <opencv2/opencv.hpp>
#include "YOLO.h"
#include "common.h"
#include "logging.h"   // 用你自己的 Logger
#include "cameracontrol.h"
#include "safe_angle.h"
// ROS2自定义消息
#include "robot_interfaces/msg/detection_result.hpp"
#include "robot_interfaces/msg/detection_results.hpp"


// ROS2节点
class FootballDetectNode : public rclcpp::Node {
public:
    FootballDetectNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~FootballDetectNode() override;
private:
    std::unique_ptr<YOLO> model_;
    std::unique_ptr<CameraController> camera_;
    rclcpp::Publisher<robot_interfaces::msg::DetectionResults>::SharedPtr publisher_;
    std::atomic<bool> camera_connected_{false};
    std::atomic<int> reconnect_attempts_{0};
    const int MAX_RECONNECT = 500;
    std::atomic<bool> running_{true};
    std::thread processing_thread_;
    std::thread reconnect_thread_;
    void publish_detection_results(const std::vector<Detection> &objects,
                                   double image_width, double image_height);
    void processing_loop(bool show_image, int image_width, int image_height,
                         double horizontal_fov, double vertical_fov);
    void reconnect_monitor();
    Logger logger;
};
