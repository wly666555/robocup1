#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <iostream>
#include <chrono>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

namespace po = boost::program_options;

inline std::string device;
inline std::string ns;
inline uint16_t damping;
inline std::string arm_direction;


/* ---------- 命令行参数 ---------- */
po::variables_map helper(int argc, char** argv)
{
#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#else
    spdlog::set_level(spdlog::level::info);
#endif

    po::options_description desc("Unitree Aloha Serial to DDS");
    desc.add_options()
        ("help,h", "produce help message")
        ("serial,s", po::value<std::string>(&device)->default_value("/dev/ttyUSB0"), "serial port")
        ("ns", po::value<std::string>(&ns)->default_value("aloha"), "dds namespace")
        ("damping", po::value<uint16_t>(&damping)->default_value(100), "aloha mini damping")
        ("direction,d", po::value<std::string>(&arm_direction)->default_value("default"), "arm direction")
        ;


    // Parse command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

/* ---------- config.yaml ---------- */
inline struct {
    float dt;
    float pos_limit;

    std::vector<float> kp;
    std::vector<float> kd;

    struct {
        std::string network;
    } dds;
} cfg;
inline YAML::Node config;
void parse_config()
{
    // 获取执行程序的路径
    auto get_exe_path = []() -> std::string {
        std::vector<char> path(1024);
        ssize_t len = readlink("/proc/self/exe", &path[0], path.size());
        if (len != -1) {
            path[len] = '\0';  // Null-terminate the result
            return std::string(&path[0]);
        } else {
            spdlog::error("Failed to get executable path.");
            exit(1);
        }
    };

    std::filesystem::path exe_path = std::filesystem::path(get_exe_path());
    std::string config_path = exe_path.parent_path().string() + "/config.yaml";

    // 如果config.yaml不存在(并非部署时)，则是正在开发阶段，使用默认路径
    if (!std::filesystem::exists(config_path)) {
        config_path = exe_path.parent_path().parent_path().string() + "/config/config.yaml";
    }
    spdlog::debug("Config Path: {}", config_path);

    try {
        config = YAML::LoadFile(config_path);

        cfg.dt = config["dt"].as<float>();
        cfg.pos_limit = config["vel_limit"].as<float>() * cfg.dt; // 直接在此处转为单次角度限幅
        cfg.dds.network = (config["dds"]["network"]) ? config["dds"]["network"].as<std::string>() : "";
        cfg.kp = config["kp"].as<std::vector<float>>();
        cfg.kd = config["kd"].as<std::vector<float>>();

    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config.yaml: {}", e.what());
        exit(1);
    }
}

}

#endif // PARAM_H