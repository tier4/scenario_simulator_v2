#include <simulation_controller/behavior/action_node.hpp>

namespace entity_behavior
{

ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{}

BT::NodeStatus ActionNode::executeTick()
{
  return BT::ActionNodeBase::executeTick();
}
}
