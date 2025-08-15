#include "servo_control/servo_control.hpp"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

// ROS2 消息支持
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"

// Unitree SDK2 标准API
#ifdef UNITREE_SDK_AVAILABLE
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/thread/thread.hpp>
#endif

namespace servo_control {

ServoControl::ServoControl() 
    : is_initialized_(false)
    , is_running_(false)
    , joint_enable_(false)
    , simulation_mode_(false)
    , servo0_calibration_(0.0f)
    , servo1_calibration_(0.0f)
    , servo0_limit_encoder_(0.0f)
    , servo1_limit_encoder_(0.0f)
{
    // 初始化Eigen向量
    joint0_limitation_ = Eigen::VectorXf::Zero(2);
    joint1_limitation_ = Eigen::VectorXf::Zero(2);
    direction_ = Eigen::VectorXf::Zero(2);
    servo_angle_ = Eigen::VectorXf::Zero(2);
    target_angle_command_ = Eigen::VectorXf::Zero(2);
    target_encoder_command_ = Eigen::VectorXf::Zero(2);
}

ServoControl::~ServoControl() {
    stop();
}

bool ServoControl::initialize() {
    try {
        spdlog::info("Initializing ServoControl...");
        
        // 加载配置
        loadConfig();
        
        // 初始化DXL控制器
        initDxlController();
        
        // 初始化ROS2通信
        initRos2Communication();
        
        // 初始化DDS通信
        initDdsCommunication();
        
        is_initialized_ = true;
        spdlog::info("ServoControl initialized successfully");
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to initialize ServoControl: {}", e.what());
        return false;
    }
}

void ServoControl::initRos2Communication() {
    try {
        spdlog::info("Initializing ROS2 communication...");
        
        // 创建ROS2节点
        ros2_node_ = std::make_shared<rclcpp::Node>("servo_control_node");
        
        // 创建发布者
        motor_state_pub_ = ros2_node_->create_publisher<robot_interfaces::msg::MotorStates>(
            "servo/motor_states", 10);
        spdlog::info("ROS2 motor state publisher initialized");
        
        // 创建订阅者
        motor_cmd_sub_ = ros2_node_->create_subscription<robot_interfaces::msg::MotorCmd>(
            "servo/motor_cmd", 10,
            std::bind(&ServoControl::motorCmdsCallback, this, std::placeholders::_1));
        spdlog::info("ROS2 motor command subscriber initialized");
        
        spdlog::info("ROS2 communication initialized successfully");
    } catch (const std::exception& e) {
        spdlog::error("Failed to initialize ROS2 communication: {}", e.what());
        ros2_node_.reset();
        motor_state_pub_.reset();
        motor_cmd_sub_.reset();
    }
}

void ServoControl::initDdsCommunication() {
    #ifdef UNITREE_SDK_AVAILABLE
    // 暂时禁用DDS功能，专注于舵机控制
    spdlog::info("DDS communication temporarily disabled - focusing on servo control");
    /*
    try {
        spdlog::info("Initializing DDS communication...");
        
        // 创建发布者 - 使用Unitree SDK2标准API
        motor_state_pub_dds_ = std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorStates_>>("rt/g1_comp_servo/state");
        if (!motor_state_pub_dds_) {
            spdlog::error("Failed to create motor state publisher");
            return;
        }
        
        // InitChannel()返回void，不需要检查返回值
        motor_state_pub_dds_->InitChannel();
        spdlog::info("Motor state publisher initialized");
        
        // 创建订阅者 - 使用Unitree SDK2标准API
        motor_cmd_sub_dds_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorCmds_>>("rt/g1_comp_servo/cmd");
        if (!motor_cmd_sub_dds_) {
            spdlog::error("Failed to create motor command subscriber");
            return;
        }
        
        // InitChannel()返回void，不需要检查返回值
        motor_cmd_sub_dds_->InitChannel([this](const void* msg) {
            try {
                const auto* motor_cmds = static_cast<const unitree_go::msg::dds_::MotorCmds_*>(msg);
                motorCmdsCallbackDds(*motor_cmds);
            } catch (const std::exception& e) {
                spdlog::error("Error in DDS callback: {}", e.what());
            }
        });
        spdlog::info("Motor command subscriber initialized");
        
        spdlog::info("DDS communication initialized successfully");
    } catch (const std::exception& e) {
        spdlog::error("Failed to initialize DDS communication: {}", e.what());
        // 清理资源
        motor_state_pub_dds_.reset();
        motor_cmd_sub_dds_.reset();
    }
    */
    #else
    spdlog::info("DDS communication disabled - Unitree SDK not available");
    #endif
}

void ServoControl::motorCmdsCallback(const robot_interfaces::msg::MotorCmd::SharedPtr msg) {
    try {
        // 处理ROS2消息
        target_angle_command_(0) = msg->q;
        target_angle_command_(1) = msg->q;  // 假设两个舵机使用相同的目标位置
        
        // 检查舵机启用状态
        if (msg->mode == 1) {
            joint_enable_ = true;
        } else {
            joint_enable_ = false;
        }
        
        spdlog::debug("Received ROS2 motor command: q={}, mode={}, enable={}", 
                     msg->q, msg->mode, joint_enable_);
    } catch (const std::exception& e) {
        spdlog::error("Error in ROS2 motorCmdsCallback: {}", e.what());
    }
}

#ifdef UNITREE_SDK_AVAILABLE
void ServoControl::motorCmdsCallbackDds(const unitree_go::msg::dds_::MotorCmds_& msg) {
    // 暂时禁用DDS回调
    spdlog::debug("DDS callback disabled");
    /*
    try {
        if (msg.cmds().size() >= 2) {
            for (int i = 0; i < 2; ++i) {
                const auto& cmd = msg.cmds()[i];
                target_angle_command_(i) = cmd.q();
                
                // 检查舵机启用状态
                if (cmd.mode() == 1) {
                    joint_enable_ = true;
                } else {
                    joint_enable_ = false;
                }
            }
            spdlog::debug("Received motor commands: q0={}, q1={}, enable={}", 
                         target_angle_command_(0), target_angle_command_(1), joint_enable_);
        }
    } catch (const std::exception& e) {
        spdlog::error("Error in motorCmdsCallback: {}", e.what());
    }
    */
}
#endif

void ServoControl::publishMotorStates() {
    try {
        if (!motor_state_pub_) {
            return;
        }
        
        // 创建ROS2状态消息
        auto motor_states = std::make_unique<robot_interfaces::msg::MotorStates>();
        motor_states->mode = joint_enable_ ? 1 : 0;
        motor_states->q = servo_angle_(0);  // 使用第一个舵机的角度
        motor_states->dq = 0.0f;  // 速度暂时设为0
        motor_states->ddq = 0.0f; // 加速度暂时设为0
        motor_states->tau_est = 0.0f; // 扭矩估计暂时设为0
        motor_states->temperature = 0; // 温度暂时设为0
        motor_states->lost = 0; // 丢失状态暂时设为0
        
        // 发布状态
        motor_state_pub_->publish(*motor_states);
        spdlog::debug("Published ROS2 motor states: q={}, mode={}, enable={}", 
                     motor_states->q, motor_states->mode, joint_enable_);
    } catch (const std::exception& e) {
        spdlog::error("Error in publishMotorStates: {}", e.what());
    }
}

#ifdef UNITREE_SDK_AVAILABLE
void ServoControl::publishMotorStatesDds() {
    // 暂时禁用DDS发布
    spdlog::debug("DDS publish disabled");
    /*
    try {
        if (!motor_state_pub_dds_) {
            return;
        }
        
        // 创建状态消息
        unitree_go::msg::dds_::MotorStates_ motor_states;
        motor_states.states().resize(2);
        
        for (int i = 0; i < 2; ++i) {
            auto& state = motor_states.states()[i];
            state.q(servo_angle_(i));
            state.dq(0.0f);  // 速度暂时设为0
            state.tau_est(0.0f); // 使用tau_est而不是tau
            state.mode(joint_enable_ ? 1 : 0);
        }
        
        // 发布状态
        motor_state_pub_dds_->Write(motor_states);
        spdlog::debug("Published motor states: q0={}, q1={}, enable={}", 
                     servo_angle_(0), servo_angle_(1), joint_enable_);
    } catch (const std::exception& e) {
        spdlog::error("Error in publishMotorStates: {}", e.what());
    }
    */
}
#endif

void ServoControl::initDxlController() {
    try {
        spdlog::info("Initializing DXL controller...");
        
        // 检查硬件连接
        if (!std::filesystem::exists("/dev/ttyUSB0")) {
            spdlog::warn("DXL device /dev/ttyUSB0 not found. Running in simulation mode.");
            simulation_mode_ = true;
        } else {
            simulation_mode_ = false;
        }
        
        // 创建DXL控制器
        dxl_controller_ = std::make_unique<DxlController>("/dev/ttyUSB0", 1000000);
        
        if (!simulation_mode_) {
            if (!dxl_controller_->init()) {
                spdlog::error("Failed to initialize DXL controller");
                simulation_mode_ = true;
            } else {
                spdlog::info("DXL controller initialized successfully");
                
                // 设置增益参数
                for (int i = 0; i < 2; ++i) {
                    dxl_controller_->setPositionPGain(i, 500);
                    dxl_controller_->setPositionDGain(i, 300);
                }
                spdlog::info("Set gain parameters successfully");
            }
        }
    } catch (const std::exception& e) {
        spdlog::error("Error initializing DXL controller: {}", e.what());
        simulation_mode_ = true;
    }
}

void ServoControl::updateServoPositions() {
    if (simulation_mode_) {
        // 模拟模式：直接使用目标角度作为当前角度
        servo_angle_ = target_angle_command_;
        spdlog::debug("Simulation mode: servo_angle = target_angle_command");
        return;
    }
    
    if (!dxl_controller_ || !joint_enable_) {
        return;
    }
    
    try {
        // 设置舵机位置
        std::array<int32_t, 2> goal_positions;
        for (int i = 0; i < 2; ++i) {
            float servo_encoder_position;
            if (i == 0) {
                servo_encoder_position = utilities::angle2encoder(
                    target_angle_command_(i), servo0_limit_encoder_, joint0_limitation_, direction_(0));
            } else {
                servo_encoder_position = utilities::angle2encoder(
                    target_angle_command_(i), servo1_limit_encoder_, joint1_limitation_, direction_(1));
            }
            
            goal_positions[i] = static_cast<int32_t>(servo_encoder_position);
        }
        
        // 同步设置位置
        dxl_controller_->syncSetPosition(goal_positions);
        
        // 读取当前位置
        dxl_controller_->syncGetPosition();
        
        // 获取当前位置并转换编码器位置为角度
        const auto& present_positions = dxl_controller_->getPresentPositions();
        servo_angle_(0) = utilities::encoder2angle(
            present_positions[0], servo0_limit_encoder_, joint0_limitation_, direction_(0));
        servo_angle_(1) = utilities::encoder2angle(
            present_positions[1], servo1_limit_encoder_, joint1_limitation_, direction_(1));
        
        spdlog::debug("Updated servo positions: q0={}, q1={}", servo_angle_(0), servo_angle_(1));
    } catch (const std::exception& e) {
        spdlog::error("Error updating servo positions: {}", e.what());
    }
}

void ServoControl::checkMotorEnable() {
    // 检查舵机是否启用 - 简化版本
    joint_enable_ = true; // 总是启用
}

void ServoControl::controlLoop() {
    checkMotorEnable();
    updateServoPositions();
    publishMotorStates();  // 总是发布ROS2状态消息
}

void ServoControl::run() {
    if (!is_initialized_) {
        spdlog::error("ServoControl not initialized");
        return;
    }
    
    is_running_ = true;
    
    // 使用标准线程进行控制循环，避免RecurrentThread的问题
    std::thread control_thread([this]() {
        spdlog::info("Control thread started");
        while (is_running_) {
            try {
                controlLoop();
                std::this_thread::sleep_for(std::chrono::milliseconds(15)); // 15ms
            } catch (const std::exception& e) {
                spdlog::error("Error in control loop: {}", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 出错时等待更长时间
            }
        }
        spdlog::info("Control thread stopped");
    });
    control_thread.detach();
    
    spdlog::info("ServoControl started");
}

void ServoControl::stop() {
    spdlog::info("Stopping ServoControl...");
    is_running_ = false;
    
    // 等待一段时间让线程自然结束
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    spdlog::info("ServoControl stopped");
}

void ServoControl::loadConfig() {
    try {
        // 获取可执行文件路径
        std::filesystem::path exe_path = std::filesystem::current_path();
        
        // 尝试多个可能的配置文件路径
        std::vector<std::string> possible_paths = {
            "config/config.yaml",
            "../config/config.yaml",
            "../src/servo_control/config/config.yaml",
            "../../src/servo_control/config/config.yaml"
        };
        
        std::string config_path;
        bool config_found = false;
        
        // 首先尝试环境变量
        const char* env_path = std::getenv("SERVO_CONFIG_PATH");
        if (env_path) {
            config_path = env_path;
            if (std::filesystem::exists(config_path)) {
                config_found = true;
                spdlog::info("Using config from environment variable: {}", config_path);
            }
        }
        
        // 如果环境变量不存在，尝试可能的路径
        if (!config_found) {
            for (const auto& path : possible_paths) {
                if (std::filesystem::exists(path)) {
                    config_path = path;
                    config_found = true;
                    spdlog::info("Found config file: {}", config_path);
                    break;
                }
            }
        }
        
        if (!config_found) {
            throw std::runtime_error("Config file not found in any of the expected locations");
        }
        
        // 加载YAML配置
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 读取配置参数
        servo0_calibration_ = config["servo0_calibration"].as<float>();
        servo1_calibration_ = config["servo1_calibration"].as<float>();
        
        // 读取关节限制
        if (config["joint0"] && config["joint0"].IsSequence()) {
            joint0_limitation_(0) = config["joint0"][0].as<float>();
            joint0_limitation_(1) = config["joint0"][1].as<float>();
        }
        
        if (config["joint1"] && config["joint1"].IsSequence()) {
            joint1_limitation_(0) = config["joint1"][0].as<float>();
            joint1_limitation_(1) = config["joint1"][1].as<float>();
        }
        
        // 读取方向参数
        if (config["direction"] && config["direction"].IsSequence()) {
            direction_(0) = config["direction"][0].as<float>();
            direction_(1) = config["direction"][1].as<float>();
        }
        
        // 设置编码器限制
        servo0_limit_encoder_ = servo0_calibration_;
        servo1_limit_encoder_ = servo1_calibration_;
        
        spdlog::info("Configuration loaded successfully from: {}", config_path);
        spdlog::info("servo0_calibration: {}", servo0_calibration_);
        spdlog::info("servo1_calibration: {}", servo1_calibration_);
        spdlog::info("joint0_limitation: [{}, {}]", joint0_limitation_(0), joint0_limitation_(1));
        spdlog::info("joint1_limitation: [{}, {}]", joint1_limitation_(0), joint1_limitation_(1));
        spdlog::info("direction: [{}, {}]", direction_(0), direction_(1));
        
    } catch (const std::exception& e) {
        spdlog::error("Failed to load config: {}", e.what());
        throw;
    }
}

} // namespace servo_control 