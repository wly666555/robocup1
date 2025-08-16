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
#include "nlohmann/json.hpp"

#include <nlohmann/detail/exceptions.hpp>
#include "common/base_client.hpp"
#include "common/ut_errror.hpp"
#include "common/patch.hpp"



using namespace std;

class G1Brain; // 类相互依赖，向前声明


class RobotClient {
public:
    RobotClient(G1Brain* argBrain) : brain(argBrain) {
        base_client_(brain, "/api/sport/request", "/api/sport/response")
    }
    void init();
    void moveHead(double pitch, double yaw);

    void Move(float vx, float vy, float vyaw) ;

    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);
    
    void SetFsmId(int fsm_id) 
    {
        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
        nlohmann::json js;
        js["data"] = fsm_id;
        req.parameter = js.dump();
        return base_client_.Call(req);
    }
    void SetVelocity(float vx, float vy, float omega, float duration = 1.F) ;
    void StandUp() ;
    void Move(float vx, float vy, float vyaw, bool continous_move) ;    

private:

    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr cmd_puber_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;

    BaseClient base_client_;
    G1Brain *brain;
    bool continous_move_ = false;

};


