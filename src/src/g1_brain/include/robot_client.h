#pragma once

#include <cstdint>
#include <limits>
#include <rclcpp/node.hpp>

#include <memory>
#include <iostream>
#include <string>
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "robot_interfaces/msg/motor_states.hpp"
#include "robot_interfaces/msg/motor_state.hpp"
#include "robot_interfaces/msg/motor_cmds.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_api/msg/request.hpp"
#include "locator.h"
#include <behaviortree_cpp/contrib/json.hpp>

#include "common/base_client.hpp"
#include "common/ut_errror.hpp"
#include "common/patch.hpp"



using namespace std;

class G1Brain; // 类相互依赖，向前声明



class RobotClient {
public:
    RobotClient(G1Brain* argBrain);
    std::shared_ptr<rclcpp::Node> node_; 

    void init();
    void moveHead(double pitch, double yaw);
    void moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);
    // void Move(float vx, float vy, float vyaw);

    void SetVelocity(float vx, float vy, float omega, float duration = 86200.F);
    
    void StartVelocityStream(double rate_hz, double hold_sec) {
        hold_sec_ = hold_sec;
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));
        timer_ = node_->create_wall_timer(period,std::bind(&RobotClient::SendOne, this));
    };

private:
    G1Brain *brain;  // 指向父节点
    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr cmd_puber_;
    BaseClient base_client_;  // 用于请求-响应处理

    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex cmd_mtx_;
    float target_vx_, target_vy_, target_wz_;
    double hold_sec_;
    bool has_cmd_;


    void SendOne() {
        float vx, vy, wz;
        {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            if (!has_cmd_) return;
            vx = target_vx_;
            vy = target_vy_;
            wz = target_wz_;
        }

        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;

        nlohmann::json js;
        js["velocity"] = {vx, vy, wz};
        js["duration"] = hold_sec_; // 持续时间 > 发布周期，作为看门狗
        req.parameter = js.dump();

        // 异步发送，不阻塞
        auto future = base_client_.AsyncCall(req);
    };


};



