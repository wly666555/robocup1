#ifndef SERVO_CONTROL_UTILITIES_HPP
#define SERVO_CONTROL_UTILITIES_HPP

#include <Eigen/Dense>

namespace servo_control {

namespace utilities {

// 角度转编码器值
float angle2encoder(float angle, float servo_limit_encoder, const Eigen::VectorXf& limitation, float direction);

// 编码器值转角度
float encoder2angle(uint32_t encoder, float servo_limit_encoder, const Eigen::VectorXf& limitation, float direction);

} // namespace utilities

} // namespace servo_control

#endif // SERVO_CONTROL_UTILITIES_HPP 