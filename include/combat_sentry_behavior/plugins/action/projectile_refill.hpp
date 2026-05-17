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

#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PROJECTILE_REFILL_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PROJECTILE_REFILL_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "combat_rm_interfaces/msg/game_status.hpp"
#include "combat_rm_interfaces/msg/sentry_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace combat_sentry_behavior
{

class CheckProjectileRefillTime : public BT::ConditionNode
{
public:
  CheckProjectileRefillTime(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("CheckProjectileRefillTime");
};

class CheckProjectileAllowance : public BT::ConditionNode
{
public:
  CheckProjectileAllowance(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("CheckProjectileAllowance");
};

class SetProjectileRefillIndex : public BT::SyncActionNode
{
public:
  SetProjectileRefillIndex(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("SetProjectileRefillIndex");
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PROJECTILE_REFILL_HPP_
