// Copyright 2026 Lijie Liang
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

#include "combat_sentry_behavior/plugins/condition/is_detect_enemy.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

namespace combat_sentry_behavior
{

IsDetectEnemyCondition::IsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemy, this), config)
{
}

BT::PortsList IsDetectEnemyCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::Target>(
      "key_port", "{@tracker_target}", "Vision tracker target port on blackboard"),
    BT::InputPort<std::vector<int>>(
      "armor_id", "1;2;3;4;5;7",
      "Expected target id. Multiple numbers should be separated by the character `;` in Groot2"),
    BT::InputPort<float>("max_distance", 8.0, "Distance to enemy target"),
  };
}

BT::NodeStatus IsDetectEnemyCondition::checkEnemy()
{
  std::vector<int> expected_armor_ids;
  float max_distance;
  auto msg = getInput<combat_rm_interfaces::msg::Target>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "Tracker target message is not available");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("armor_id", expected_armor_ids)) {
    RCLCPP_ERROR(logger_, "Failed to read [armor_id]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("max_distance", max_distance)) {
    RCLCPP_ERROR(logger_, "Failed to read [max_distance]");
    return BT::NodeStatus::FAILURE;
  }

  if (!msg->tracking || msg->id.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  int target_id;
  try {
    target_id = std::stoi(msg->id);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger_, "Invalid tracker target id [%s]: %s", msg->id.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const bool is_id_match =
    std::find(expected_armor_ids.begin(), expected_armor_ids.end(), target_id) !=
    expected_armor_ids.end();
  const float distance_to_enemy = std::hypot(msg->position.x, msg->position.y);
  const bool is_within_distance = distance_to_enemy <= max_distance;

  return is_id_match && is_within_distance ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::IsDetectEnemyCondition>("IsDetectEnemy");
}
