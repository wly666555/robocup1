#pragma once

#include <cmath>
#include <algorithm>
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <Eigen/Dense> // <--- 加上这个
#include <stdint.h>
#include "brain.h"


class G1Brain;

using namespace std; 
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
    SelfLocate(const string &name, const NodeConfiguration &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    
    BT::NodeStatus tick() override;

private:
    BrainData *data;
    YamlParser yamlparser;
    G1Brain* brain;
    rclcpp::Time lastSuccessfulLocalizeTime;
};

class Adjust : public BT::SyncActionNode {
public:
    Adjust(const string &name, const NodeConfiguration &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    BrainData *data;
};

class CamFindBall : public SyncActionNode {
public:
    CamFindBall(const std::string& name, const NodeConfiguration& config, G1Brain* brain);
 
    NodeStatus tick() override;
 
private:
    using Vec2f = Eigen::Vector2f;
    MultiStageInterpolator interpolator_;
    std::vector<std::pair<Vec2f, float>> predefinedPhases_;
    bool firstRun_ = true;
    G1Brain* brain;
};

class CamTrackBall : public BT::SyncActionNode {
public:
    CamTrackBall(const string &name, const NodeConfiguration &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    
    double yaw_angle_add = 0;
    double pitch_angle_add = 0;
};

class Chase : public BT::SyncActionNode {
public:
    Chase(const string &name, const NodeConfiguration &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    BT::NodeStatus tick() override;

private:
    BrainData *data;
    G1Brain *brain; 
};

class Kick : public BT::SyncActionNode {
public:
    Kick(const string &name, const NodeConfiguration &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    
};

class PrintMsg : public BT::SyncActionNode {
public:
    PrintMsg(const string &name, const NodeConfiguration &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("msg", "Message to print")
        };
    }

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
    
};


// =================== playerDecision 节点 ===================
class playerDecision : public BT::SyncActionNode
{
public:
    playerDecision(const string &name, const NodeConfiguration &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("decision", "decision string")
        };
    }

    BT::NodeStatus tick() override;
private:
    BrainData *data;
    G1Brain *brain;
};
