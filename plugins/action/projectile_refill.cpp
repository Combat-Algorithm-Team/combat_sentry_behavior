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

#include "combat_sentry_behavior/plugins/action/projectile_refill.hpp"

#include <algorithm>
#include <string>

namespace combat_sentry_behavior
{
namespace
{

bool readStringPort(
  BT::TreeNode & node, rclcpp::Logger logger, const char * port, std::string & value)
{
  const auto input = node.getInput<std::string>(port);
  if (!input || input.value().empty()) {
    RCLCPP_ERROR(
      logger, "Failed to read non-empty [%s]: %s", port,
      input ? "empty value" : input.error().c_str());
    return false;
  }

  value = input.value();
  return true;
}

bool readIntPort(BT::TreeNode & node, rclcpp::Logger logger, const char * port, int & value)
{
  const auto input = node.getInput<int>(port);
  if (!input) {
    RCLCPP_ERROR(logger, "Failed to read [%s]: %s", port, input.error().c_str());
    return false;
  }

  value = input.value();
  return true;
}

int readBlackboardCounter(
  const BT::Blackboard::Ptr & blackboard, const std::string & key, int default_value)
{
  int value = default_value;
  if (!blackboard->get<int>(key, value)) {
    return default_value;
  }
  return value;
}

bool computeRefillIndex(
  const combat_rm_interfaces::msg::GameStatus & game_status, int expected_game_progress,
  int match_duration, int refill_period, int max_refill_index, int & refill_index)
{
  if (game_status.game_progress != expected_game_progress) {
    return false;
  }
  if (match_duration <= 0 || refill_period <= 0 || max_refill_index <= 0) {
    return false;
  }

  const int elapsed_time = std::max(0, match_duration - game_status.stage_remain_time);
  refill_index = std::min(elapsed_time / refill_period, max_refill_index);
  return true;
}

bool readRefillTimingPorts(
  BT::TreeNode & node, rclcpp::Logger logger, int & expected_game_progress, int & match_duration,
  int & refill_period, int & max_refill_index)
{
  return readIntPort(node, logger, "expected_game_progress", expected_game_progress) &&
         readIntPort(node, logger, "match_duration", match_duration) &&
         readIntPort(node, logger, "refill_period", refill_period) &&
         readIntPort(node, logger, "max_refill_index", max_refill_index);
}

}  // namespace

CheckProjectileRefillTime::CheckProjectileRefillTime(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckProjectileRefillTime::tick()
{
  if (!config().blackboard) {
    RCLCPP_ERROR(logger_, "Blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  const auto game_status = getInput<combat_rm_interfaces::msg::GameStatus>("game_status_port");
  if (!game_status) {
    RCLCPP_ERROR(logger_, "GameStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string refill_index_key;
  int expected_game_progress = 4;
  int match_duration = 420;
  int refill_period = 60;
  int max_refill_index = 6;
  int default_refill_index = 0;
  if (
    !readStringPort(*this, logger_, "refill_index_key", refill_index_key) ||
    !readRefillTimingPorts(
      *this, logger_, expected_game_progress, match_duration, refill_period, max_refill_index) ||
    !readIntPort(*this, logger_, "default_refill_index", default_refill_index)) {
    return BT::NodeStatus::FAILURE;
  }

  int current_refill_index = 0;
  if (
    !computeRefillIndex(
      game_status.value(), expected_game_progress, match_duration, refill_period, max_refill_index,
      current_refill_index) ||
    current_refill_index <= 0)
  {
    return BT::NodeStatus::FAILURE;
  }

  const int last_refill_index =
    readBlackboardCounter(config().blackboard, refill_index_key, default_refill_index);
  return current_refill_index > last_refill_index ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
}

BT::PortsList CheckProjectileRefillTime::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::GameStatus>(
      "game_status_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<std::string>(
      "refill_index_key", "rmuc_projectile_refill_index",
      "Blackboard key storing the latest handled projectile refill interval"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected running game progress"),
    BT::InputPort<int>("match_duration", 420, "Running stage duration in seconds"),
    BT::InputPort<int>("refill_period", 60, "Projectile refill period in seconds"),
    BT::InputPort<int>(
      "max_refill_index", 6, "Maximum refill interval index during the match"),
    BT::InputPort<int>(
      "default_refill_index", 0, "Value used when the refill index is not initialized"),
  };
}

CheckProjectileAllowance::CheckProjectileAllowance(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckProjectileAllowance::tick()
{
  const auto sentry_info = getInput<combat_rm_interfaces::msg::SentryInfo>("sentry_info_port");
  if (!sentry_info) {
    RCLCPP_ERROR(logger_, "SentryInfo message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int projectile_allowance_17mm_below = -1;
  int projectile_allowance_17mm_above = -1;
  if (
    !readIntPort(
      *this, logger_, "projectile_allowance_17mm_below", projectile_allowance_17mm_below) ||
    !readIntPort(
      *this, logger_, "projectile_allowance_17mm_above", projectile_allowance_17mm_above)) {
    return BT::NodeStatus::FAILURE;
  }

  const int projectile_allowance_17mm =
    static_cast<int>(sentry_info->projectile_allowance_17mm);
  const bool is_below_match = projectile_allowance_17mm_below < 0 ||
                              projectile_allowance_17mm < projectile_allowance_17mm_below;
  const bool is_above_match = projectile_allowance_17mm_above < 0 ||
                              projectile_allowance_17mm > projectile_allowance_17mm_above;

  return (is_below_match && is_above_match) ? BT::NodeStatus::SUCCESS
                                            : BT::NodeStatus::FAILURE;
}

BT::PortsList CheckProjectileAllowance::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::SentryInfo>(
      "sentry_info_port", "{@referee_sentryInfo}", "SentryInfo port on blackboard"),
    BT::InputPort<int>(
      "projectile_allowance_17mm_below", -1,
      "Return SUCCESS when 17mm projectile allowance is below this value. Negative value ignores"),
    BT::InputPort<int>(
      "projectile_allowance_17mm_above", -1,
      "Return SUCCESS when 17mm projectile allowance is above this value. Negative value ignores"),
  };
}

SetProjectileRefillIndex::SetProjectileRefillIndex(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus SetProjectileRefillIndex::tick()
{
  if (!config().blackboard) {
    RCLCPP_ERROR(logger_, "Blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  const auto game_status = getInput<combat_rm_interfaces::msg::GameStatus>("game_status_port");
  if (!game_status) {
    RCLCPP_ERROR(logger_, "GameStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string refill_index_key;
  int expected_game_progress = 4;
  int match_duration = 420;
  int refill_period = 60;
  int max_refill_index = 6;
  int default_refill_index = 0;
  if (
    !readStringPort(*this, logger_, "refill_index_key", refill_index_key) ||
    !readRefillTimingPorts(
      *this, logger_, expected_game_progress, match_duration, refill_period, max_refill_index) ||
    !readIntPort(*this, logger_, "default_refill_index", default_refill_index)) {
    return BT::NodeStatus::FAILURE;
  }

  int current_refill_index = 0;
  if (
    !computeRefillIndex(
      game_status.value(), expected_game_progress, match_duration, refill_period, max_refill_index,
      current_refill_index) ||
    current_refill_index <= 0)
  {
    return BT::NodeStatus::FAILURE;
  }

  const int last_refill_index =
    readBlackboardCounter(config().blackboard, refill_index_key, default_refill_index);
  config().blackboard->set(refill_index_key, std::max(last_refill_index, current_refill_index));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetProjectileRefillIndex::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::GameStatus>(
      "game_status_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<std::string>(
      "refill_index_key", "rmuc_projectile_refill_index",
      "Blackboard key storing the latest handled projectile refill interval"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected running game progress"),
    BT::InputPort<int>("match_duration", 420, "Running stage duration in seconds"),
    BT::InputPort<int>("refill_period", 60, "Projectile refill period in seconds"),
    BT::InputPort<int>(
      "max_refill_index", 6, "Maximum refill interval index during the match"),
    BT::InputPort<int>(
      "default_refill_index", 0, "Value used when the refill index is not initialized"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::CheckProjectileRefillTime>(
    "CheckProjectileRefillTime");
  factory.registerNodeType<combat_sentry_behavior::CheckProjectileAllowance>(
    "CheckProjectileAllowance");
  factory.registerNodeType<combat_sentry_behavior::SetProjectileRefillIndex>(
    "SetProjectileRefillIndex");
}
