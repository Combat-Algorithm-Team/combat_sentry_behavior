#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_FIELD_EVENT_CONDITION_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_FIELD_EVENT_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "combat_rm_interfaces/msg/event_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace combat_sentry_behavior
{

class CheckFieldEventCondition : public BT::SimpleConditionNode
{
public:
  CheckFieldEventCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkEvent();

  rclcpp::Logger logger_ = rclcpp::get_logger("CheckFieldEventCondition");
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_FIELD_EVENT_CONDITION_HPP_