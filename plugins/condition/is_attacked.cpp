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

#include "combat_sentry_behavior/plugins/condition/is_attacked.hpp"

namespace combat_sentry_behavior
{

IsAttackedCondition::IsAttackedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedCondition::checkIsAttacked, this), config)
{
}

BT::NodeStatus IsAttackedCondition::checkIsAttacked()
{
  auto robotstatus_msg = getInput<combat_rm_interfaces::msg::RobotStatus>("robotstatus_port");
  auto hurt_msg = getInput<combat_rm_interfaces::msg::HurtData>("hurtdata_port");
  if (!robotstatus_msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }
  if (!hurt_msg) {
    RCLCPP_ERROR(logger_, "HurtData message is not available");
    return BT::NodeStatus::FAILURE;
  }

  bool is_hp_deduced = false;
  if (last_hp_ - robotstatus_msg->current_hp >= 20) {
    is_hp_deduced = true;
  } else {
    is_hp_deduced = false;
  }
  last_hp_ = robotstatus_msg->current_hp;

  const bool is_attacked = is_hp_deduced && hurt_msg->hp_deduction_reason == hurt_msg->ARMOR_HIT;

  // RCLCPP_WARN(logger_, "cur hp = %d; last_hp = %d", robotstatus_msg->current_hp, last_hp_);

  // RCLCPP_WARN(
  //   logger_, "is_hp_deduced = %d; hp_deduction_reason = %d", is_hp_deduced,
  //   hurt_msg->hp_deduction_reason);

  // RCLCPP_WARN(logger_, "is attacked : %d; ", is_attacked);

  return is_attacked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsAttackedCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::RobotStatus>(
      "robotstatus_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<combat_rm_interfaces::msg::HurtData>(
      "hurtdata_port", "{@referee_hurtdata}", "HurtData port on blackboard")};
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::IsAttackedCondition>("IsAttacked");
}
