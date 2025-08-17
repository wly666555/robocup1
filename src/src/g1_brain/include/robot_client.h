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

    int32_t Move(float vx, float vy, float vyaw) ;

    int32_t moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);
    
    int32_t SetFsmId(int fsm_id) 
    {
        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
        nlohmann::json js;
        js["data"] = fsm_id;
        req.parameter = js.dump();
        return base_client_.Call(req);
    }
    int32_t SetVelocity(float vx, float vy, float omega, float duration ) ;
    int32_t StandUp() ;
    int32_t Move(float vx, float vy, float vyaw, bool continous_move) ;    

private:

    rclcpp::Publisher<robot_interfaces::msg::MotorCmds>::SharedPtr cmd_puber_;

    BaseClient base_client_;
    G1Brain *brain;
    bool continous_move_ = false;

};

