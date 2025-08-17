#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <string>
#include <chrono>
#include <sstream>

/**
 * @brief 弧度转角度
 * @param rad 弧度值
 * @return 角度值
 */
inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

/**
 * @brief 角度转弧度
 * @param deg 角度值
 * @return 弧度值
 */
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

/**
 * @brief 角度归一化到[-π, π]
 * @param angle 原始角度（弧度）
 * @return 归一化后的角度
 */
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief 获取当前时间戳（毫秒）
 * @return 时间戳
 */
inline uint64_t getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

/**
 * @brief 字符串格式化函数
 * @param format 格式化字符串
 * @param args 可变参数
 * @return 格式化后的字符串
 */
template<typename... Args>
std::string string_format(const std::string& format, Args... args) {
    int size = snprintf(nullptr, 0, format.c_str(), args...) + 1;
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1);
}

#endif // UTILS_H
    