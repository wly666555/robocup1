#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
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
#include "version_node.h"
// ROS2自定义消息
#include "robot_interfaces/msg/detection_result.hpp"
#include "robot_interfaces/msg/detection_results.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<FootballDetectNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Fatal error: Unknown exception!" << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}

