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

#include "combat_sentry_behavior/plugins/condition/is_rfid_detected.hpp"

namespace combat_sentry_behavior
{

IsRfidDetectedCondition::IsRfidDetectedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRfidDetectedCondition::checkRfidStatus, this), config)
{
}

BT::NodeStatus IsRfidDetectedCondition::checkRfidStatus()
{
  bool ally_supply_point_non_exchange, center_gain_point;
  auto msg = getInput<combat_rm_interfaces::msg::RfidStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RfidStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("ally_supply_point_non_exchange", ally_supply_point_non_exchange);
  getInput("center_gain_point", center_gain_point);

  if ((ally_supply_point_non_exchange && msg->ally_supply_point_non_exchange == msg->DETECTED) ||
    (center_gain_point && msg->center_gain_point == msg->DETECTED)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList IsRfidDetectedCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::RfidStatus>(
      "key_port", "{@referee_rfidStatus}", "RfidStatus port on blackboard"),
    BT::InputPort<bool>(
      "ally_supply_point_non_exchange", false, "己方与兑换区不重叠的补给区 / RMUL 补给区"),
    BT::InputPort<bool>("center_gain_point", false, "中心增益点（仅 RMUL 适用）"),
  };
}
}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::IsRfidDetectedCondition>("IsRfidDetected");
}
