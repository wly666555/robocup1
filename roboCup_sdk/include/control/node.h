#ifndef NODE_H
#define NODE_H

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

template <typename T>
void registerNode(BT::BehaviorTreeFactory& factory, const std::string& id, Interface* interface)
{
    factory.registerBuilder<T>(id, [interface](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<T>(name, config, interface);
    });
}

class camToPosition : public BT::SyncActionNode
{
public:
    camToPosition(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("joint0_angle", 0.0, "Joint 0 desired angle"),
            BT::InputPort<double>("joint1_angle", 0.0, "Joint 1 desired angle"),
            BT::InputPort<double>("duration", 500.0, "duration"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Interface *_interface;

    float percent = 0;   
    Vec2<float> initAngle;
    Vec2<float> targetAngle;
    bool firstRun = false;
};


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


class robotTrackPelvis : public BT::SyncActionNode
{
public:
    robotTrackPelvis(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    BT::NodeStatus tick() override;

private:
    Interface *_interface;
};


class BackToPosition : public BT::SyncActionNode
{
public:
    BackToPosition(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    BT::NodeStatus tick() override;

private:
    Interface *_interface;
};

class robotTrackField : public BT::SyncActionNode
{
public:
    robotTrackField(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    BT::NodeStatus tick() override;

private:
    Interface *_interface;
};

class kick : public BT::SyncActionNode
{
public:
    kick(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}
   
    BT::NodeStatus tick() override;

private:
    Interface *_interface;

    double counter = 0;
};


class playerDecision : public BT::SyncActionNode
{
public:
    playerDecision(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}
    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("decision", "decision string")
        };
    }

    BT::NodeStatus tick() override;
private:
    Interface *_interface;
};

class WristControl : public BT::SyncActionNode
{
public:
    WristControl(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("wrist_angle", 0.0, "Joint 0 desired angle"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Interface *_interface;

    float _duration_1 = 1000;  
    float _percent_1 = 0;   
    bool hasSubWristState = false;
    float init_angle;
    float target_angle;
    int counter = 0;
    const int amplitude = 30; // Amplitude of the sine wave
    const int frequency = 4; // Frequency of the sine wave

    float weight = 0.f;
    float kp = 60.f;
    float kd = 1.5f;
};

class Speak : public BT::SyncActionNode
{
public:
    Speak(const std::string& name, const BT::NodeConfig& config, Interface* interface)
        : BT::SyncActionNode(name, config), _interface(interface)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("text", "Say this text")
        };
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

#endif