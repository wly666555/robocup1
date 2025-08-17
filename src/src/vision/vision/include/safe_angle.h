#pragma once

#include "rclcpp/rclcpp.hpp"
#include <iostream>

// 角度计算
std::pair<float, float> calculate_angles_from_offsets(
    float dx, float dy, int image_width, int image_height,
    float horizontal_fov, float vertical_fov);
std::pair<float, float> safe_calculate_angles(
    float dx, float dy, int image_width, int image_height,
    float horizontal_fov, float vertical_fov);

