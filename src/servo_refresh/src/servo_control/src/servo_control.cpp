#include "servo_control/servo_control.hpp"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

// ROS2 消息支持
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/motor_cmds.hpp"
#include "robot_interfaces/msg/motor_states.hpp"
#include "robot_interfaces/msg/motor_state.hpp"

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
            "servo/motor_state", 10);
        spdlog::info("ROS2 motor state publisher initialized");
        
        // 创建订阅者
        motor_cmd_sub_ = ros2_node_->create_subscription<robot_interfaces::msg::MotorCmds>(
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

void ServoControl::motorCmdsCallback(const robot_interfaces::msg::MotorCmds::SharedPtr msg) {
    // 处理ROS2舵机命令 - 总是包含两个关节：yaw和pitch
    if (msg->states.size() >= 2) {
        const auto &cmd_yaw = msg->states[0];
        const auto &cmd_pitch = msg->states[1];

        target_angle_command_(0) = cmd_yaw.q;      // yaw (servo0)
        target_angle_command_(1) = cmd_pitch.q;    // pitch (servo1)

        joint_enable_ = (cmd_yaw.mode == 1) || (cmd_pitch.mode == 1);

        spdlog::debug("Received MotorCmds: yaw={}, pitch={}, enable={}",
                      target_angle_command_(0), target_angle_command_(1), joint_enable_);
    }
}

void ServoControl::publishMotorStates() {
    try {
        if (!motor_state_pub_) {
            return;
        }
        
        // 创建ROS2状态消息（两通道）
        robot_interfaces::msg::MotorStates motor_states_msg;
        motor_states_msg.states.resize(2);

        motor_states_msg.states[0].mode = joint_enable_ ? 1 : 0;
        motor_states_msg.states[0].q = servo_angle_(0);
        motor_states_msg.states[0].dq = 0.0f;
        motor_states_msg.states[0].ddq = 0.0f;
        motor_states_msg.states[0].tau_est = 0.0f;
        motor_states_msg.states[0].temperature = 0;
        motor_states_msg.states[0].lost = 0;

        motor_states_msg.states[1].mode = joint_enable_ ? 1 : 0;
        motor_states_msg.states[1].q = servo_angle_(1);
        motor_states_msg.states[1].dq = 0.0f;
        motor_states_msg.states[1].ddq = 0.0f;
        motor_states_msg.states[1].tau_est = 0.0f;
        motor_states_msg.states[1].temperature = 0;
        motor_states_msg.states[1].lost = 0;

        // 发布状态
        motor_state_pub_->publish(motor_states_msg);
        spdlog::debug("Published MotorStates: q0={}, q1={}, enable={}",
                      motor_states_msg.states[0].q, motor_states_msg.states[1].q, joint_enable_);
    } catch (const std::exception& e) {
        spdlog::error("Error in publishMotorStates: {}", e.what());
    }
}

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
    // 检查舵机是否启用
    static bool last_joint_enable = false;
    
    if (joint_enable_ != last_joint_enable) {
        if (joint_enable_) {
            // 启用舵机
            if (dxl_controller_ && !simulation_mode_) {
                dxl_controller_->enable(0, 1);
                dxl_controller_->enable(1, 1);
                spdlog::info("Enable arm.");
            }
        } else {
            // 禁用舵机
            if (dxl_controller_ && !simulation_mode_) {
                dxl_controller_->enable(0, 0);
                dxl_controller_->enable(1, 0);
                spdlog::info("Release arm.");
            }
        }
        last_joint_enable = joint_enable_;
    }
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
        // 获取当前工作目录
        std::filesystem::path current_path = std::filesystem::current_path();
        spdlog::info("Current working directory: {}", current_path.string());
        
        // 尝试多个可能的配置文件路径
        std::vector<std::string> possible_paths = {
            "config/servo_config.yaml",
            "config/config.yaml",
            "../config/servo_config.yaml",
            "../config/config.yaml",
            "../src/servo_control/config/servo_config.yaml",
            "../src/servo_control/config/config.yaml",
            "../../src/servo_control/config/servo_config.yaml",
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
            } else {
                spdlog::warn("Environment variable SERVO_CONFIG_PATH points to non-existent file: {}", config_path);
            }
        }
        
        // 如果环境变量不存在，尝试可能的路径
        if (!config_found) {
            spdlog::info("Searching for config files in possible paths...");
            for (const auto& path : possible_paths) {
                spdlog::debug("Checking path: {}", path);
                if (std::filesystem::exists(path)) {
                    config_path = path;
                    config_found = true;
                    spdlog::info("Found config file: {}", config_path);
                    break;
                }
            }
        }
        
        if (!config_found) {
            spdlog::warn("Config file not found in any of the expected locations. Using default values.");
            // 使用默认配置值
            servo0_calibration_ = 1440.0f;
            servo1_calibration_ = 2348.0f;
            joint0_limitation_(0) = -50.0f;
            joint0_limitation_(1) = 50.0f;
            joint1_limitation_(0) = -20.0f;
            joint1_limitation_(1) = 85.0f;
            direction_(0) = 1.0f;
            direction_(1) = -1.0f;
            
            // 设置编码器限制
            servo0_limit_encoder_ = servo0_calibration_;
            servo1_limit_encoder_ = servo1_calibration_;
            
            spdlog::info("Using default configuration values");
            return;
        }
        
        // 加载YAML配置
        spdlog::info("Loading config from: {}", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 检查配置文件结构
        if (config["servo_control"] && config["servo_control"]["ros__parameters"]) {
            // 使用ROS2参数格式
            config = config["servo_control"]["ros__parameters"];
            spdlog::info("Using ROS2 parameter format");
        }
        
        // 读取配置参数
        if (config["servo0_calibration"]) {
            servo0_calibration_ = config["servo0_calibration"].as<float>();
            spdlog::info("Loaded servo0_calibration: {}", servo0_calibration_);
        } else {
            spdlog::warn("servo0_calibration not found in config, using default: 1440");
            servo0_calibration_ = 1440.0f;
        }
        
        if (config["servo1_calibration"]) {
            servo1_calibration_ = config["servo1_calibration"].as<float>();
            spdlog::info("Loaded servo1_calibration: {}", servo1_calibration_);
        } else {
            spdlog::warn("servo1_calibration not found in config, using default: 2348");
            servo1_calibration_ = 2348.0f;
        }
        
        // 读取关节限制
        if (config["joint0"] && config["joint0"].IsSequence()) {
            joint0_limitation_(0) = config["joint0"][0].as<float>();
            joint0_limitation_(1) = config["joint0"][1].as<float>();
            spdlog::info("Loaded joint0_limitation: [{}, {}]", joint0_limitation_(0), joint0_limitation_(1));
        } else {
            spdlog::warn("joint0 not found in config, using default: [-50, 50]");
            joint0_limitation_(0) = -50.0f;
            joint0_limitation_(1) = 50.0f;
        }
        
        if (config["joint1"] && config["joint1"].IsSequence()) {
            joint1_limitation_(0) = config["joint1"][0].as<float>();
            joint1_limitation_(1) = config["joint1"][1].as<float>();
            spdlog::info("Loaded joint1_limitation: [{}, {}]", joint1_limitation_(0), joint1_limitation_(1));
        } else {
            spdlog::warn("joint1 not found in config, using default: [-20, 85]");
            joint1_limitation_(0) = -20.0f;
            joint1_limitation_(1) = 85.0f;
        }
        
        // 读取方向参数
        if (config["direction"] && config["direction"].IsSequence()) {
            direction_(0) = config["direction"][0].as<float>();
            direction_(1) = config["direction"][1].as<float>();
            spdlog::info("Loaded direction: [{}, {}]", direction_(0), direction_(1));
        } else {
            spdlog::warn("direction not found in config, using default: [1, -1]");
            direction_(0) = 1.0f;
            direction_(1) = -1.0f;
        }
        
        // 设置编码器限制
        servo0_limit_encoder_ = servo0_calibration_;
        servo1_limit_encoder_ = servo1_calibration_;
        
        spdlog::info("Configuration loaded successfully from: {}", config_path);
        spdlog::info("Final values - servo0_calibration: {}, servo1_calibration: {}", servo0_calibration_, servo1_calibration_);
        spdlog::info("Final values - joint0_limitation: [{}, {}]", joint0_limitation_(0), joint0_limitation_(1));
        spdlog::info("Final values - joint1_limitation: [{}, {}]", joint1_limitation_(0), joint1_limitation_(1));
        spdlog::info("Final values - direction: [{}, {}]", direction_(0), direction_(1));
        spdlog::info("Final values - servo0_limit_encoder: {}, servo1_limit_encoder: {}", servo0_limit_encoder_, servo1_limit_encoder_);
        
    } catch (const std::exception& e) {
        spdlog::error("Failed to load config: {}", e.what());
        throw;
    }
}

} // namespace servo_control 