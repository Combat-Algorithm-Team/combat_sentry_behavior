// Copyright 2026 Jieliang Li
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

#include "combat_sentry_behavior/plugins/condition/is_base_hp_low.hpp"

#include <functional>

namespace combat_sentry_behavior
{

IsBaseHpLowCondition::IsBaseHpLowCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsBaseHpLowCondition::checkBaseHp, this), config)
{
}

BT::NodeStatus IsBaseHpLowCondition::checkBaseHp()
{
  const auto msg = getInput<combat_rm_interfaces::msg::GameRobotHp>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameRobotHp message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int hp_threshold = 4000;
  const auto hp_threshold_result = getInput("hp_threshold", hp_threshold);
  if (!hp_threshold_result) {
    RCLCPP_ERROR(
      logger_, "Failed to read [hp_threshold]: %s", hp_threshold_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (hp_threshold < 0) {
    RCLCPP_WARN(logger_, "hp_threshold should not be negative, clamp to 0");
    hp_threshold = 0;
  }

  return static_cast<int>(msg->ally_base_hp) < hp_threshold ? BT::NodeStatus::SUCCESS
                                                           : BT::NodeStatus::FAILURE;
}

BT::PortsList IsBaseHpLowCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::GameRobotHp>(
      "key_port", "{@referee_gameRobotHp}", "GameRobotHp port on blackboard"),
    BT::InputPort<int>(
      "hp_threshold", 600, "Return SUCCESS when ally_base_hp is lower than this value"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::IsBaseHpLowCondition>("IsBaseHpLow");
}
