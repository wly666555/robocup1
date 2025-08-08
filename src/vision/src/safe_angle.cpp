#include "safe_angle.h"
// 角度计算
std::pair<float, float> calculate_angles_from_offsets(
    float dx, float dy, int image_width, int image_height,
    float horizontal_fov, float vertical_fov)
{
    const float normalized_dx = dx / static_cast<float>(image_width / 2.0f);
    const float normalized_dy = dy / static_cast<float>(image_height / 2.0f);
    const float yaw = normalized_dx * (horizontal_fov / 2.0f);
    const float pitch = -normalized_dy * (vertical_fov / 2.0f);
    return {yaw, pitch};
}
std::pair<float, float> safe_calculate_angles(
    float dx, float dy, int image_width, int image_height,
    float horizontal_fov, float vertical_fov)
{
    if (image_width <= 0 || image_height <= 0)
        throw std::invalid_argument("Invalid image dimensions");
    if (horizontal_fov <= 0 || vertical_fov <= 0)
        throw std::invalid_argument("FOV values must be positive");
    const float clamped_dx = std::clamp(dx, -image_width/2.0f, image_width/2.0f);
    const float clamped_dy = std::clamp(dy, -image_height/2.0f, image_height/2.0f);
    auto [yaw, pitch] = calculate_angles_from_offsets(
        clamped_dx, clamped_dy, image_width, image_height, horizontal_fov, vertical_fov);
    yaw = std::clamp(yaw, -horizontal_fov/2, horizontal_fov/2);
    pitch = std::clamp(pitch, -vertical_fov/2, vertical_fov/2);
    return {yaw, pitch};
}
