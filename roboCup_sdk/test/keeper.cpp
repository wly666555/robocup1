#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include "control/node.h"
#include "control/interface.h"

using namespace BT;

int main(int argc, char const *argv[])
{
  unitree::robot::ChannelFactory::Instance()->Init(0,argv[1]);

  Interface interface;

  BehaviorTreeFactory factory;
  registerNode<robotTrackField>(factory, "robotTrackField", &interface);
  registerNode<kick>(factory, "kick", &interface);
  registerNode<GoalieBackToHome>(factory, "GoalieBackToHome", &interface);
  registerNode<camFindBall>(factory, "camFindBall", &interface);
  registerNode<GoalieDecision>(factory, "GoalieDecision", &interface);
  registerNode<camTrackBall>(factory, "camTrackBall", &interface);
  factory.registerNodeType<CheckDecision>("CheckDecision");
  factory.registerBehaviorTreeFromFile("/home/unitree/robocup/roboCup_sdk/test/keeper.xml");
  auto tree = factory.createTree("MainTree");
  
  while(true)
  {
    tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
  }
  return 0;
}
