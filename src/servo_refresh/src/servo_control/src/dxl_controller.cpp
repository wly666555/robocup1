#include "servo_control/dxl_controller.hpp"
#include <spdlog/spdlog.h>
#include <unistd.h>

namespace servo_control {

DxlController::DxlController(const std::string& device_port, uint32_t baudrate)
    : device_port_(device_port), baudrate_(baudrate) {
    present_position.fill(0);
}

DxlController::~DxlController() = default;

bool DxlController::init() {
    // 创建PortHandler (使用工厂方法)
    portHandler = dynamixel::PortHandler::getPortHandler(device_port_.c_str());
    
    // 创建PacketHandler (使用工厂方法)
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    // 打开端口
    if (!portHandler->openPort()) {
        spdlog::error("Failed to open port: {}", device_port_);
        return false;
    }
    
    // 设置波特率
    if (!portHandler->setBaudRate(baudrate_)) {
        spdlog::error("Failed to set baudrate: {}", baudrate_);
        return false;
    }
    
    // 创建GroupSyncWrite
    groupSyncWrite = std::make_unique<dynamixel::GroupSyncWrite>(
        portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
    
    // 创建GroupSyncRead
    groupSyncRead = std::make_unique<dynamixel::GroupSyncRead>(
        portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    
    // 添加舵机到同步读写组
    for (int i = 0; i < 2; i++) {
        groupSyncRead->addParam(i);
    }
    
    spdlog::info("DXL controller initialized successfully");
    return true;
}

void DxlController::enable(uint8_t id, uint8_t mode) {
    uint8_t dxl_error = 0;
    if (packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, mode, &dxl_error) != COMM_SUCCESS) {
        spdlog::warn("Failed to enable motor {}! Err: {}", id, dxl_error);
    }
}

void DxlController::setPosition(uint8_t id, int32_t goal_position) {
    uint8_t dxl_error = 0;
    if (packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_position, &dxl_error) != COMM_SUCCESS) {
        spdlog::warn("Failed to set position {} for motor {}!", goal_position, id);
    }
}

void DxlController::getPosition(uint8_t id, uint32_t* present_position) {
    uint8_t dxl_error = 0;
    if (packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, present_position, &dxl_error) != COMM_SUCCESS) {
        spdlog::warn("Failed to get position for motor {}!", id);
    }
}

void DxlController::syncSetPosition(const std::array<int32_t, 2>& goal_positions) {
    uint8_t param_goal_position[4];
    for (size_t id = 0; id < goal_positions.size(); id++) {
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_positions[id]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_positions[id]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_positions[id]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_positions[id]));

        if (!groupSyncWrite->addParam(id, param_goal_position)) {
            spdlog::warn("[ID:{}] GroupSyncWrite addparam failed!", id);
        }
    }
    
    if (groupSyncWrite->txPacket() != COMM_SUCCESS) {
        spdlog::warn("GroupSyncWrite txPacket failed!");
    }
    groupSyncWrite->clearParam();
}

void DxlController::syncGetPosition() {
    if (groupSyncRead->txRxPacket() != COMM_SUCCESS) {
        spdlog::warn("GroupSyncRead txRxPacket failed!");
        return;
    }

    for (size_t id = 0; id < present_position.size(); id++) {
        if (!groupSyncRead->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)) {
            spdlog::warn("[ID:{}] Failed to get position!", id);
            continue;
        }
        present_position[id] = groupSyncRead->getData(id, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    }
}

void DxlController::setPositionPGain(uint8_t id, uint16_t kp) {
    while (true) {
        if (setUint16(id, POSITION_P_GAIN, kp) == COMM_SUCCESS) break;
        usleep(50000);
    }
}

void DxlController::setPositionDGain(uint8_t id, uint16_t kd) {
    while (true) {
        if (setUint16(id, POSITION_D_GAIN, kd) == COMM_SUCCESS) break;
        usleep(50000);
    }
}

int DxlController::setUint16(uint8_t id, uint16_t address, uint16_t data) {
    uint8_t dxl_error = 0;
    return packetHandler->write2ByteTxRx(portHandler, id, address, data, &dxl_error);
}

} // namespace servo_control 