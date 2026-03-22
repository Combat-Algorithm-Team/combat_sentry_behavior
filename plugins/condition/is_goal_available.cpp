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

#include "combat_sentry_behavior/plugins/condition/is_goal_available.hpp"

#include "combat_sentry_behavior/custom_types.hpp"

namespace combat_sentry_behavior
{

IsGoalAvailableCondition::IsGoalAvailableCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsGoalAvailableCondition::checkIsGoalAvailable, this), config)
{
}

BT::NodeStatus IsGoalAvailableCondition::checkIsGoalAvailable()
{
  Pose3D goal{};
  getInput("goal", goal);

  float max_x;
  getInput<float>("max_x", max_x);

  float max_y;
  getInput<float>("max_y", max_y);

  bool is_goal_available = false;
  if (goal.x < max_x && goal.y < max_y) {
    is_goal_available = true;
  }

  return is_goal_available ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsGoalAvailableCondition::providedPorts()
{
  return {
    BT::InputPort<float>("max_x", "8.0", "the maximum x coordinate of the goal position"),
    BT::InputPort<float>("max_y", "8.0", "the maximum y coordinate of the goal position"),
    BT::InputPort<Pose3D>(
      "goal", "{candidate_pose}",
      "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::IsGoalAvailableCondition>("IsGoalAvailable");
}
