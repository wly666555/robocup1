#pragma once

#include <cmath>
#include <algorithm>
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
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
    SelfLocate(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    
    BT::NodeStatus tick() override;
    
    static PortsList providedPorts()
    {
        return {
            InputPort<string>("mode", "enter_field", "must be one of [enter_field, trust_direction, trust_position, trust_nothing, face_forward]"),
            // enter_field: 上场时使用，此时必然在已方半场，且可以根据已方球门的位置进一步缩小方向范围
            // trust_direction: 正常情况下使用，此时 odom 信息大体上是准确的（未摔倒过）
            // trust_position: 使用于摔倒后，此时 x,y 可信，但方向不可信。（注意，如果此时在中线附近，则因为球场的对称性，需认为位置也不可信）
            // trust_nothing: 极限情况，认为 x,y 也不可信，需要先通过标志物辨别方向。
            // face_forward: 面向对方球门的方向, 主要用于测试
        };
    };

private:
    G1Brain* brain;
};

class Adjust : public BT::SyncActionNode {
public:
    Adjust(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
};

class CamFindBall : public SyncActionNode {
public:
    CamFindBall(const std::string& name, const NodeConfig& config, G1Brain* brain);
 
    NodeStatus tick() override;
 
private:
    double _cmdSequence[6][2];    // The sequence of actions for finding the ball, in which the robot looks towards these positions in order.
    rclcpp::Time _timeLastCmd;    // The time of the last command execution, used to ensure there is a time interval between commands.
    int _cmdIndex;                // The current step in the cmdSequence that is being executed.
    long _cmdIntervalMSec;        // The time interval (in milliseconds) between executing actions in the sequence.
    long _cmdRestartIntervalMSec; // If the time since the last execution exceeds this value, the sequence will restart from step 0.
    G1Brain* brain;
};

class CamTrackBall : public BT::SyncActionNode {
public:
    CamTrackBall(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
};

class Chase : public BT::SyncActionNode {
public:
    Chase(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("dist", 1.0, "追球的目标是球后面多少距离"),
        };
    }
    BT::NodeStatus tick() override;

private:
    G1Brain *brain; 
    string _state;
    double _dir = 1.0;  //+为右

};

class Kick : public StatefulActionNode {
public:
    Kick(const string &name, const NodeConfig &config, G1Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    G1Brain *brain;
    rclcpp::Time _startTime;
    int _msecKick = 1000;
};

class PrintMsg : public BT::SyncActionNode {
public:
    PrintMsg(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

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

class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    G1Brain *brain;
};
// =================== playerDecision 节点 ===================
class StrikerDecide : public BT::SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            OutputPort<std::string>("decision", "decision string")};
    }

    BT::NodeStatus tick() override;
private:
    G1Brain *brain;
};


class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

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
    G1Brain *brain;
};

class MoveToPoseOnField : public BT::SyncActionNode{
public:
    MoveToPoseOnField(const string &name, const NodeConfig &config, G1Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    BT::NodeStatus tick() override;

private:
    G1Brain *brain;
};

class Rotate : public BT::StatefulActionNode{
public:
    Rotate(const string &name, const NodeConfig &config, G1Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    G1Brain *brain;
    double turn_dir;
};
