#ifndef NODE_H
#define NODE_H


#include <iostream>

#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"

#include "dds/Publisher.h"
#include "dds/Subscription.h"

#include <unitree/common/thread/thread.hpp>
#include "control/interface.h"

#include "common/mathTools.h"
#include "common/mathTypes.h"

#include "common/types.h"



using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::g1;

#include <stdint.h>




template <typename T>
void registerNode(BT::BehaviorTreeFactory& factory, const std::string& id, Interface* interface)
{
    factory.registerBuilder<T>(id, [interface](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<T>(name, config, interface);
    });
}




using Vec2f = Eigen::Vector2f;

struct InterpolationPhase {
    Vec2f target;
    float duration;
    float progress = 0.0f;

    InterpolationPhase(const Vec2f& t, float d) : target(t), duration(d) {}
};

class MultiStageInterpolator {
public:
    void reset(const Vec2f& init) {
        phases_.clear();
        currentPhase_ = 0;
        initAngle_ = init;
    }

    void addPhase(const Vec2f& target, float duration) {
        phases_.emplace_back(target, duration);
    }

    bool interpolate(Vec2f& outAngle) {
        if (currentPhase_ >= phases_.size()) return false;

        auto& phase = phases_[currentPhase_];
        phase.progress += 1.0f / phase.duration;
        if (phase.progress > 1.0f) phase.progress = 1.0f;

        Vec2f from = (currentPhase_ == 0) ? initAngle_ : phases_[currentPhase_ - 1].target;
        outAngle = (1.0f - phase.progress) * from + phase.progress * phase.target;

        if (phase.progress >= 1.0f) currentPhase_++;
        return true;
    }

private:
    Vec2f initAngle_;
    std::vector<InterpolationPhase> phases_;
    size_t currentPhase_ = 0;
};

class camFindBall : public BT::SyncActionNode
{
public:
    camFindBall(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
    Vec2f initAngle;
    Vec2f targetAngle;

    MultiStageInterpolator interpolator;
    const std::vector<std::pair<Vec2f, float>> predefinedPhases = {
        {Vec2f(0, 0),     200},
        {Vec2f(35, -15),  500},
        {Vec2f(35, 15),   500},
        {Vec2f(-35, -15), 500},
        {Vec2f(-35, 15),  500},
        {Vec2f(0, 0),     500}
    };
    bool firstRun = true;
};

class camTrackBall : public BT::SyncActionNode
{
public:
    camTrackBall(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {
        counter = 0;
    }

    BT::NodeStatus tick() override;

private:
    Interface *_interface;
    double yaw_angle_add;
    double pitch_angle_add;
    double counter = 0;
};



class MoveToPoseOnField : public BT::SyncActionNode
{
public:
    MoveToPoseOnField(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "目标 x 坐标, Field 坐标系"),
            InputPort<double>("y", 0, "目标 y 坐标, Field 坐标系"),
            InputPort<double>("theta", 0, "目标最终朝向, Field 坐标系"),
            InputPort<double>("long_range_threshold", 1.5, "目标点的距离超过这个值时, 优先走过去, 而不是细调位置和方向"),
            InputPort<double>("turn_threshold", 0.4, "长距离时, 目标点的方向超这个数值时, 先转向目标点"),
            InputPort<double>("x_tolerance", 0.2, "x 容差"),
            InputPort<double>("y_tolerance", 0.2, "y 容差"),
            InputPort<double>("theta_tolerance", 0.1, "theta 容差"),
        };
    }
    BT::NodeStatus tick() override;

private:
    Interface *_interface;
};

class Chase : public BT::SyncActionNode
{
public:
    Chase(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("dist", 1.0, "追球的目标是球后面多少距离"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Interface *_interface;

    double _dir = 1.0;  //+为右

    string _state;
};

class kick : public BT::StatefulActionNode
{
public:
    kick(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::StatefulActionNode(name, config), _interface(interface){}
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    Interface *_interface;
    int _msecKick = 1000;

    std::chrono::steady_clock::time_point _startTime;
};


class StrikerDecide : public BT::SyncActionNode
{
public:
    StrikerDecide(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}
    static BT::PortsList providedPorts()
        {
            return {
                InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
                InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
                OutputPort<string>("decision_out")};
        }

    BT::NodeStatus tick() override;
private:
    Interface *_interface;
};



class AbnormalCondition : public BT::SyncActionNode
{
public:
    AbnormalCondition(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    BT::NodeStatus tick() override;

private:
    Interface *_interface;

    RotMat<double> B2G_RotMat, G2B_RotMat;
};

// ===================== Goalie Nodes Begin =====================


class GoalieDecide : public BT::SyncActionNode
{
public:
    GoalieDecide(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    static BT::PortsList providedPorts()
        {
            return {
                InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
                InputPort<double>("adjust_angle_tolerance", 0.1, "小于这个角度, 认为 adjust 已经成功"),
                InputPort<double>("adjust_y_tolerance", 0.1, "y 方向偏移小于这个值, 认为 y 方向 adjust 成功"),
                InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
                OutputPort<string>("decision_out")};
        }

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};



class Adjust : public BT::SyncActionNode
{
public:
    Adjust(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};

class RobotFindBall : public BT::StatefulActionNode
{
public:
    RobotFindBall(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::StatefulActionNode(name, config), _interface(interface){}
   
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    Interface *_interface;

    double turn_dir;
};
// ===================== Goalie Nodes End =====================

#endif