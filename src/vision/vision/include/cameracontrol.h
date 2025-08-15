#pragma once

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
#include "common.h"
#include "librealsense2/rs.hpp"

// Camera Controller
class CameraController {
    rs2::pipeline pipe;
    rs2::config cfg;
    std::string serial;
    const int width, height, fps;
public:
    rs2_intrinsics intrinsics;
    CameraController(int w, int h, int f);
    bool initialize();
    void shutdown();
    rs2::frameset poll_frames(int max_retries = 100) ;
    std::string get_serial() const { return serial; };
};
