#include "servo_control/utilities.hpp"
#include <cmath>
#include <algorithm>

namespace servo_control {

namespace utilities {

float angle2encoder(float angle, float servo_limit_encoder, const Eigen::VectorXf& limitation, float direction) {
    // 限制角度范围
    float desired_angle = std::clamp(angle, limitation(0), limitation(1));
    
    // 使用与g1_comp_servo_service相同的编码器分辨率计算
    float servo_encoder_resolution = 4096.0f / (2.0f * M_PI);
    float joint_range = limitation(1) - limitation(0); 
    float encoder_range = joint_range * (M_PI / 180.0f) * servo_encoder_resolution;
    
    // 应用方向和校准，与g1_comp_servo_service保持一致
    float desired_encoder = direction * (desired_angle - limitation(0)) * (encoder_range / joint_range) + servo_limit_encoder;
    
    return desired_encoder;
}

float encoder2angle(uint32_t encoder, float servo_limit_encoder, const Eigen::VectorXf& limitation, float direction) {
    // 使用与g1_comp_servo_service相同的编码器分辨率计算
    float servo_encoder_resolution = 4096.0f / (2.0f * M_PI);
    float joint_range = limitation(1) - limitation(0); 
    float encoder_range = joint_range * (M_PI / 180.0f) * servo_encoder_resolution;
    
    // 应用方向和校准，与g1_comp_servo_service保持一致
    float desired_encoder = direction * (static_cast<float>(encoder) - servo_limit_encoder) * ((joint_range * (M_PI / 180.0f)) / encoder_range) + limitation(0) * (M_PI / 180.0f);
    desired_encoder *= (180.0f / M_PI);
    
    return desired_encoder;
}

} // namespace utilities

} // namespace servo_control 