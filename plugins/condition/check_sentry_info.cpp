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

#include "combat_sentry_behavior/plugins/condition/check_sentry_info.hpp"

namespace combat_sentry_behavior
{

CheckSentryInfoCondition::CheckSentryInfoCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&CheckSentryInfoCondition::checkSentryInfo, this), config)
{
}

BT::NodeStatus CheckSentryInfoCondition::checkSentryInfo()
{
  auto msg = getInput<combat_rm_interfaces::msg::SentryInfo>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "SentryInfo message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int current_state = -1;
  int ally_power_rune_state = -1;
  int disengaged_state = -1;
  if (
    !readExpectedValue("current_state", current_state) ||
    !readExpectedValue("ally_power_rune_state", ally_power_rune_state) ||
    !readExpectedValue("disengaged_state", disengaged_state)) {
    return BT::NodeStatus::FAILURE;
  }

  const auto & sentry_info = msg.value();
  auto check = [](int expected, int actual) { return expected == -1 || expected == actual; };

  const bool match = check(current_state, sentry_info.current_state) &&
                     check(ally_power_rune_state, sentry_info.ally_power_rune_state ? 1 : 0) &&
                     check(disengaged_state, sentry_info.disengaged_state ? 1 : 0);

  return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

bool CheckSentryInfoCondition::readExpectedValue(const char * port_name, int & value)
{
  const auto input = getInput<int>(port_name);
  if (!input) {
    RCLCPP_ERROR(logger_, "Failed to read [%s]: %s", port_name, input.error().c_str());
    value = -1;
    return false;
  }

  value = input.value();
  return true;
}

BT::PortsList CheckSentryInfoCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::SentryInfo>(
      "key_port", "{@referee_sentryInfo}", "SentryInfo port on blackboard"),
    BT::InputPort<int>(
      "current_state", -1,
      "Expected SentryInfo.current_state. Use -1 to ignore, 3 means moving posture"),
    BT::InputPort<int>(
      "ally_power_rune_state", -1,
      "Expected ally_power_rune_state. Use -1 to ignore, 1 means currently activatable"),
    BT::InputPort<int>(
      "disengaged_state", -1, "Expected disengaged_state. Use -1 to ignore, 1 means disengaged"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::CheckSentryInfoCondition>("CheckSentryInfo");
}
