
#include <string>
#include <iostream>

#include "multi_robot/behavior_tree_nodes/OpenGripper.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace multi_robot
{

OpenGripper::OpenGripper(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
OpenGripper::halt()
{
  std::cout << "OpenGripper halt" << std::endl;
}

BT::NodeStatus
OpenGripper::tick()
{
  std::cout << "OpenGripper tick " << counter_ << std::endl;

  if (counter_++ < 5) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace multi_robot

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multi_robot::OpenGripper>("OpenGripper");
}

