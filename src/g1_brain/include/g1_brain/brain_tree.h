#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

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

extern bool goal_wly;

template <typename T>
void registerNode(BT::BehaviorTreeFactory& factory, const std::string& id, Interface* interface)
{
    factory.registerBuilder<T>(id, [interface](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<T>(name, config, interface);
    });
}

class BrainTree {
public:
    void init();
    void initEntry();
    void tick();


    template <typename T>//
    void setEntry(const std::string& key, const T& value);//
    
    template <typename T>//
    T getEntry(const std::string& key);//




private:
    void setEntry(const std::string& key, const auto& value) {
        // Implementation would be here
    }
    template<typename T>
    T getEntry(const std::string& key) {
        // Implementation would be here
        return T();
    }

     Brain* brain;  // 添加Brain指针成员
    BT::Tree tree; // 添加行为树对象
};


class selfLocate : public BT::SyncActionNode {
public:
    selfLocate(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};


class Adjust : public BT::SyncActionNode {
public:
    Adjust(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};

class CamFindBall : public BT::SyncActionNode {
public:
    CamFindBall(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

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

class CamTrackBall : public BT::SyncActionNode {
public:
    CamTrackBall(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
    double yaw_angle_add = 0;
    double pitch_angle_add = 0;
};

class robotTrackPelvis : public BT::SyncActionNode {
public:
    robotTrackPelvis(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};

class robotTrackField : public BT::SyncActionNode {
public:
    robotTrackField(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};

class Kick : public BT::SyncActionNode {
public:
    Kick(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}
   
    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};

class PrintMsg : public BT::SyncActionNode {
public:
    PrintMsg(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("msg", "Message to print")
        };
    }

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};

class SetVelocity : public BT::SyncActionNode {
public:
    SetVelocity(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("x", "X velocity"),
            BT::InputPort<double>("y", "Y velocity"),
            BT::InputPort<double>("theta", "Angular velocity")
        };
    }

    BT::NodeStatus tick() override;

private:
    Interface* _interface;
};

// 保留原有的MultiStageInterpolator定义
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

#endif