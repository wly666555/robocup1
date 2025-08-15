#ifndef SERVO_CONTROL_DXL_CONTROLLER_HPP
#define SERVO_CONTROL_DXL_CONTROLLER_HPP

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <Eigen/Dense>
#include <memory>
#include <array>

namespace servo_control {

class DxlController {
public:
    DxlController(const std::string& device_port, uint32_t baudrate);
    ~DxlController();

    // 初始化设备
    bool init();
    
    // 启用/禁用舵机
    void enable(uint8_t id, uint8_t mode = 1);
    
    // 设置位置
    void setPosition(uint8_t id, int32_t goal_position);
    
    // 获取位置
    void getPosition(uint8_t id, uint32_t* present_position);
    
    // 同步设置位置
    void syncSetPosition(const std::array<int32_t, 2>& goal_positions);
    
    // 同步获取位置
    void syncGetPosition();
    
    // 设置增益
    void setPositionPGain(uint8_t id, uint16_t kp);
    void setPositionDGain(uint8_t id, uint16_t kd);

    // 获取当前位置数组
    const std::array<uint32_t, 2>& getPresentPositions() const { return present_position; }

private:
    // 设备设置
    static constexpr uint8_t PROTOCOL_VERSION = 2.0;
    static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
    static constexpr uint16_t POSITION_D_GAIN = 80;
    static constexpr uint16_t POSITION_P_GAIN = 84;
    static constexpr uint16_t ADDR_GOAL_POSITION = 116;
    static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
    static constexpr uint16_t LEN_PRO_GOAL_POSITION = 4;
    static constexpr uint16_t LEN_PRO_PRESENT_POSITION = 4;

    std::string device_port_;
    uint32_t baudrate_;
    
    // Dynamixel SDK 对象 (使用原始指针，由SDK管理)
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite;
    std::unique_ptr<dynamixel::GroupSyncRead> groupSyncRead;
    
    // 当前位置
    std::array<uint32_t, 2> present_position;
    
    // 私有方法
    int setUint16(uint8_t id, uint16_t address, uint16_t data);
};

} // namespace servo_control

#endif // SERVO_CONTROL_DXL_CONTROLLER_HPP 