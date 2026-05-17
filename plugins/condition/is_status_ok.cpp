// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "combat_sentry_behavior/plugins/condition/is_status_ok.hpp"

namespace combat_sentry_behavior
{

IsStatusOKCondition::IsStatusOKCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusOKCondition::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsStatusOKCondition::checkRobotStatus()
{
  auto msg = getInput<combat_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int hp_min = 200;
  int hp_max = 400;
  if (!getInput("hp_min", hp_min) || !getInput("hp_max", hp_max)) {
    RCLCPP_ERROR(logger_, "Failed to read hp_min or hp_max");
    return BT::NodeStatus::FAILURE;
  }

  if (hp_min > hp_max) {
    RCLCPP_ERROR(logger_, "Invalid HP range: hp_min(%d) > hp_max(%d)", hp_min, hp_max);
    return BT::NodeStatus::FAILURE;
  }

  if (msg->current_hp < hp_min) {
    need_recover_ = true;
    return BT::NodeStatus::FAILURE;
  }

  if (msg->current_hp >= hp_max) {
    need_recover_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return need_recover_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::PortsList IsStatusOKCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>("hp_min", 200, "Minimum HP. NOTE: Sentry init/max HP is 400"),
    BT::InputPort<int>("hp_max", 400, "Recovery exit HP. NOTE: Sentry init/max HP is 400")};
}
}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::IsStatusOKCondition>("IsStatusOK");
}
