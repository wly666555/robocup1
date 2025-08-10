#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense> // <--- 加上这个
#include "nav_msgs/msg/odometry.hpp"
#include "robot_interfaces/msg/low_state.hpp"
#include "robot_interfaces/msg/motor_cmd.hpp"
#include "locate/math_utils.h"
#include "locate/misc.h"
#include <stdint.h>
#include "locate/types.h"
#include "brain_data.h"
#include "brain.h"
#include "client.hpp"

// using namespace std; // 建议去掉
using namespace BT;

// =================== 行为树主类 ===================
class BrainTree {
public:
    BrainTree(G1Brain *argBrain) : brain(argBrain) {}

    void init();
    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const std::string &key)
    {
        T value = T();
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const std::string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    G1Brain *brain;
    BrainConfig config_;
    void initEntry();
};
// =================== MultiStageInterpolator 定义 ===================
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

// =================== 行为树节点类 ===================
class SelfLocate : public BT::SyncActionNode {
public:
    SelfLocate(const std::string &name, const BT::NodeConfiguration& config, G1Brain *_brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

    
    bool isOdomCalibrated() const { return odomCalibrated; }
    BT::NodeStatus tick() override;

private:
    YamlParser yamlparser;
    std::shared_ptr<BrainData> data;
    G1Brain* brain;
    BrainConfig config;
    bool odomCalibrated = false;
    std::chrono::high_resolution_clock::time_point lastSuccessfulLocalizeTime;
};

class Adjust : public BT::SyncActionNode {
public:
    Adjust(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<BrainData> data;
    G1Brain *brain;
    BrainConfig config;
};

class CamFindBall : public BT::SyncActionNode {
public:
    CamFindBall(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}
    void setClient(std::shared_ptr<RobotClient> c) { client = c; }
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<RobotClient> client;
    G1Brain *brain;
    BrainConfig config;
    using Vec2f = Eigen::Vector2f;
    Vec2f initAngle;
    Vec2f targetAngle;
    MultiStageInterpolator interpolator; // <--- 只写类型名
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
    CamTrackBall(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    BrainConfig config;
    double yaw_angle_add = 0;
    double pitch_angle_add = 0;
};

class robotTrackField : public BT::SyncActionNode {
public:
    robotTrackField(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    BrainConfig config;
};

class Kick : public BT::SyncActionNode {
public:
    Kick(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    BrainConfig config;
};

class PrintMsg : public BT::SyncActionNode {
public:
    PrintMsg(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("msg", "Message to print")
        };
    }

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    BrainConfig config;
};

class SetVelocity : public BT::SyncActionNode {
public:
    SetVelocity(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

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
    G1Brain *brain;
    BrainConfig config;
};



// =================== playerDecision 节点 ===================
class playerDecision : public BT::SyncActionNode
{
public:
    playerDecision(const std::string& name, const BT::NodeConfiguration& config, G1Brain* _brain, const BrainConfig& brain_config)
        : SyncActionNode(name, config), brain(_brain), config(brain_config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("decision", "decision string")
        };
    }

    BT::NodeStatus tick() override;
private:
    G1Brain *brain;
    BrainConfig config;
    bool goalSignal;
};

#endif
