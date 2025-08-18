#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include "control/node.h"
#include "control/interface.h"

using namespace std;
using namespace BT;

int main(int argc, char const *argv[])
{
  unitree::robot::ChannelFactory::Instance()->Init(0,argv[1]);
  std::shared_ptr<BT::Blackboard> blackboard = std::make_shared<BT::Blackboard>(); // 创建黑板实例

  
  BehaviorTreeFactory factory;
  registerNode<camFindBall>(factory, "camFindBall", &interface);
  registerNode<Adjust>(factory, "Adjust", &interface);
  registerNode<Chase>(factory, "Chase", &interface);
  registerNode<Kick>(factory, "Kick", &interface);
  registerNode<GoalieDecide>(factory, "GoalieDecide", &interface);
  registerNode<CamTrackBall>(factory, "CamTrackBall", &interface);
  registerNode<SetVelocity>(factory, "SetVelocity", &interface);
  registerNode<RobotFindBall>(factory, "RobotFindBall", &interface);
  factory.registerNodeType<CheckDecision>("CheckDecision");
  factory.registerBehaviorTreeFromFile("/home/unitree/robocup/roboCup_sdk/test/striker.xml");

  Interface interface(blackboard);

  auto tree = factory.createTree("MainTree");
  // 设置黑板默认值
  auto blackboard = tree.blackboard(); // 黑板指针


  blackboard->setEntry<bool>("ball_location_known", false);
  blackboard->setEntry<std::string>("decision", "");
  blackboard->setEntry<string>("player_role", "striker");
  blackboard->setEntry<double>("ball_range", 0.0);



  
  while(true)
  {
    tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
  }
  return 0;
}
