

#ifndef MULTI_ROBOT__BEHAVIOR_TREE_NODES__CLOSEGRIPPER_HPP_
#define MULTI_ROBOT__BEHAVIOR_TREE_NODES__CLOSEGRIPPER_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace multi_robot
{

class CloseGripper : public BT::ActionNodeBase
{
public:
  explicit CloseGripper(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  int counter_;
};

}  // namespace multi_robot

#endif  // MULTI_ROBOT__BEHAVIOR_TREE_NODES__CLOSEGRIPPER_HPP_

