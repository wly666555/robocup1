#include "servo_control/utilities.hpp"
#include <cmath>

namespace servo_control {

namespace utilities {

float angle2encoder(float angle, float calibration, const Eigen::Vector2f& limitation, float direction) {
    // 应用校准和方向
    float calibrated_angle = (angle - calibration) * direction;
    
    // 限制角度范围
    float limited_angle = std::clamp(calibrated_angle, limitation(0), limitation(1));
    
    // 转换为编码器值 (假设编码器范围是0-4095)
    float encoder = (limited_angle - limitation(0)) / (limitation(1) - limitation(0)) * 4095.0f;
    
    return encoder;
}

float encoder2angle(uint32_t encoder, float calibration, const Eigen::Vector2f& limitation, float direction) {
    // 编码器值转角度
    float normalized_encoder = static_cast<float>(encoder) / 4095.0f;
    float angle = normalized_encoder * (limitation(1) - limitation(0)) + limitation(0);
    
    // 应用方向和校准
    float calibrated_angle = angle / direction + calibration;
    
    return calibrated_angle;
}

} // namespace utilities

} // namespace servo_control 