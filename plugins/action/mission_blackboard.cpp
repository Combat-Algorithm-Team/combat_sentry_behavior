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

#include "combat_sentry_behavior/plugins/action/mission_blackboard.hpp"

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

bool readBoolPort(BT::TreeNode & node, rclcpp::Logger logger, const char * port, bool & value)
{
  const auto input = node.getInput<bool>(port);
  if (!input) {
    RCLCPP_ERROR(logger, "Failed to read [%s]: %s", port, input.error().c_str());
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

bool readBlackboardFlag(
  const BT::Blackboard & blackboard, const std::string & key, bool default_value)
{
  bool value = default_value;
  if (!blackboard.get<bool>(key, value)) {
    return default_value;
  }
  return value;
}

int readBlackboardCounter(
  const BT::Blackboard & blackboard, const std::string & key, int default_value)
{
  int value = default_value;
  if (!blackboard.get<int>(key, value)) {
    return default_value;
  }
  return value;
}

}  // namespace

CheckMissionFlag::CheckMissionFlag(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckMissionFlag::tick()
{
  if (!config().blackboard) {
    RCLCPP_ERROR(logger_, "Blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string flag;
  bool expected = true;
  bool default_value = false;
  if (
    !readStringPort(*this, logger_, "flag", flag) ||
    !readBoolPort(*this, logger_, "expected", expected) ||
    !readBoolPort(*this, logger_, "default_value", default_value)) {
    return BT::NodeStatus::FAILURE;
  }

  const bool value =
    readBlackboardFlag(*config().blackboard->rootBlackboard(), flag, default_value);
  return value == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList CheckMissionFlag::providedPorts()
{
  return {
    BT::InputPort<std::string>("flag", "Mission flag key"),
    BT::InputPort<bool>("expected", true, "Expected flag value"),
    BT::InputPort<bool>("default_value", false, "Value used when the flag is not initialized"),
  };
}

SetMissionFlag::SetMissionFlag(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus SetMissionFlag::tick()
{
  if (!config().blackboard) {
    RCLCPP_ERROR(logger_, "Blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string flag;
  bool value = true;
  if (
    !readStringPort(*this, logger_, "flag", flag) ||
    !readBoolPort(*this, logger_, "value", value)) {
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->rootBlackboard()->set(flag, value);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetMissionFlag::providedPorts()
{
  return {
    BT::InputPort<std::string>("flag", "Mission flag key"),
    BT::InputPort<bool>("value", true, "Flag value to set"),
  };
}

CheckMissionCounter::CheckMissionCounter(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckMissionCounter::tick()
{
  if (!config().blackboard) {
    RCLCPP_ERROR(logger_, "Blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string counter;
  int max_count = 1;
  int default_value = 0;
  if (
    !readStringPort(*this, logger_, "counter", counter) ||
    !readIntPort(*this, logger_, "max_count", max_count) ||
    !readIntPort(*this, logger_, "default_value", default_value)) {
    return BT::NodeStatus::FAILURE;
  }

  const int value =
    readBlackboardCounter(*config().blackboard->rootBlackboard(), counter, default_value);
  return value < max_count ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList CheckMissionCounter::providedPorts()
{
  return {
    BT::InputPort<std::string>("counter", "Mission counter key"),
    BT::InputPort<int>("max_count", 1, "Return SUCCESS when current counter is less than this"),
    BT::InputPort<int>("default_value", 0, "Value used when the counter is not initialized"),
  };
}

SetMissionCounter::SetMissionCounter(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus SetMissionCounter::tick()
{
  if (!config().blackboard) {
    RCLCPP_ERROR(logger_, "Blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string counter;
  int value = 0;
  if (
    !readStringPort(*this, logger_, "counter", counter) ||
    !readIntPort(*this, logger_, "value", value)) {
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->rootBlackboard()->set(counter, value);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetMissionCounter::providedPorts()
{
  return {
    BT::InputPort<std::string>("counter", "Mission counter key"),
    BT::InputPort<int>("value", 0, "Counter value to set"),
  };
}

IncrementMissionCounter::IncrementMissionCounter(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus IncrementMissionCounter::tick()
{
  if (!config().blackboard) {
    RCLCPP_ERROR(logger_, "Blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string counter;
  int step = 1;
  int max_count = -1;
  if (
    !readStringPort(*this, logger_, "counter", counter) ||
    !readIntPort(*this, logger_, "step", step) ||
    !readIntPort(*this, logger_, "max_count", max_count)) {
    return BT::NodeStatus::FAILURE;
  }

  int value = readBlackboardCounter(*config().blackboard->rootBlackboard(), counter, 0) + step;
  if (max_count >= 0 && value > max_count) {
    value = max_count;
  }
  config().blackboard->rootBlackboard()->set(counter, value);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList IncrementMissionCounter::providedPorts()
{
  return {
    BT::InputPort<std::string>("counter", "Mission counter key"),
    BT::InputPort<int>("step", 1, "Counter increment"),
    BT::InputPort<int>("max_count", -1, "Optional upper clamp. Negative value disables clamp"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::CheckMissionFlag>("CheckMissionFlag");
  factory.registerNodeType<combat_sentry_behavior::SetMissionFlag>("SetMissionFlag");
  factory.registerNodeType<combat_sentry_behavior::CheckMissionCounter>("CheckMissionCounter");
  factory.registerNodeType<combat_sentry_behavior::SetMissionCounter>("SetMissionCounter");
  factory.registerNodeType<combat_sentry_behavior::IncrementMissionCounter>(
    "IncrementMissionCounter");
}
