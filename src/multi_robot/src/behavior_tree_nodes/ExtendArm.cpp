
#include <string>
#include <iostream>

#include "multi_robot/behavior_tree_nodes/ExtendArm.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace multi_robot
{

ExtendArm::ExtendArm(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
ExtendArm::halt()
{
  std::cout << "ExtendArm halt" << std::endl;
}

BT::NodeStatus
ExtendArm::tick()
{
  std::cout << "ExtendArm tick " << counter_ << std::endl;

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
  factory.registerNodeType<multi_robot::ExtendArm>("ExtendArm");
}

