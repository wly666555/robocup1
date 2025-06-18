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
  registerNode<playerDecision>(factory, "playerDecision", &interface);
  registerNode<camFindBall>(factory, "camFindBall", &interface);
  registerNode<camTrackBall>(factory, "camTrackBall", &interface);
  registerNode<robotTrackField>(factory, "robotTrackField", &interface);
  registerNode<kick>(factory, "kick", &interface);
  factory.registerBehaviorTreeFromFile("/home/unitree/roboCup_sdk/test/halfFieldKickExample.xml");
  auto tree = factory.createTree("MainTree");
  
  while(true)
  {
    tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
  }
  return 0;
}