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
  auto msg = getInput<combat_rm_interfaces::msg::RfidStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RfidStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  if (!readExpectedValues()) {
    return BT::NodeStatus::FAILURE;
  }

  const auto & rfid_status = msg.value();
  if ((ally_base_gain_point_ && rfid_status.ally_base_gain_point) ||
    (ally_central_highland_gain_point_ && rfid_status.ally_central_highland_gain_point) ||
    (ally_fortress_gain_point_ && rfid_status.ally_fortress_gain_point) ||
    (ally_outpost_gain_point_ && rfid_status.ally_outpost_gain_point) ||
    (ally_supply_point_non_exchange_ && rfid_status.ally_supply_point_non_exchange) ||
    (ally_supply_point_exchange_ && rfid_status.ally_supply_point_exchange) ||
    (enemy_central_highland_gain_point_ && rfid_status.enemy_central_highland_gain_point) ||
    (enemy_fortress_gain_point_ && rfid_status.enemy_fortress_gain_point) ||
    (enemy_outpost_gain_point_ && rfid_status.enemy_outpost_gain_point) ||
    (center_gain_point_ && rfid_status.center_gain_point))
  {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

bool IsRfidDetectedCondition::readExpectedValues()
{
  return readExpectedValue("ally_base_gain_point", ally_base_gain_point_) &&
         readExpectedValue("ally_central_highland_gain_point", ally_central_highland_gain_point_) &&
         readExpectedValue("ally_fortress_gain_point", ally_fortress_gain_point_) &&
         readExpectedValue("ally_outpost_gain_point", ally_outpost_gain_point_) &&
         readExpectedValue("ally_supply_point_non_exchange", ally_supply_point_non_exchange_) &&
         readExpectedValue("ally_supply_point_exchange", ally_supply_point_exchange_) &&
         readExpectedValue("enemy_central_highland_gain_point", enemy_central_highland_gain_point_) &&
         readExpectedValue("enemy_fortress_gain_point", enemy_fortress_gain_point_) &&
         readExpectedValue("enemy_outpost_gain_point", enemy_outpost_gain_point_) &&
         readExpectedValue("center_gain_point", center_gain_point_);
}

bool IsRfidDetectedCondition::readExpectedValue(const char * port_name, bool & value)
{
  const auto input = getInput<bool>(port_name);
  if (!input) {
    RCLCPP_ERROR(logger_, "Failed to read [%s]: %s", port_name, input.error().c_str());
    value = false;
    return false;
  }

  value = input.value();
  return true;
}

BT::PortsList IsRfidDetectedCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::RfidStatus>(
      "key_port", "{@referee_rfidStatus}", "RfidStatus port on blackboard"),
    BT::InputPort<bool>("ally_base_gain_point", false, "己方基地增益点"),
    BT::InputPort<bool>("ally_central_highland_gain_point", false, "己方中央高地增益点"),
    BT::InputPort<bool>("ally_fortress_gain_point", false, "己方堡垒增益点"),
    BT::InputPort<bool>("ally_outpost_gain_point", false, "己方前哨站增益点"),
    BT::InputPort<bool>(
      "ally_supply_point_non_exchange", false, "己方与兑换区不重叠的补给区 / RMUL 补给区"),
    BT::InputPort<bool>("ally_supply_point_exchange", false, "己方与兑换区重叠的补给区"),
    BT::InputPort<bool>("enemy_central_highland_gain_point", false, "对方中央高地增益点"),
    BT::InputPort<bool>("enemy_fortress_gain_point", false, "对方堡垒增益点"),
    BT::InputPort<bool>("enemy_outpost_gain_point", false, "对方前哨站增益点"),
    BT::InputPort<bool>("center_gain_point", false, "中心增益点（仅 RMUL 适用）"),
  };
}
}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::IsRfidDetectedCondition>("IsRfidDetected");
}
