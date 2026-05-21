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

#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_OUTPOST_HP_LOW_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_OUTPOST_HP_LOW_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "combat_rm_interfaces/msg/game_robot_hp.hpp"
#include "rclcpp/rclcpp.hpp"

namespace combat_sentry_behavior
{

class IsOutpostHpLowCondition : public BT::SimpleConditionNode
{
public:
  IsOutpostHpLowCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkOutpostHp();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsOutpostHpLowCondition");
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_OUTPOST_HP_LOW_HPP_
