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

#include "combat_sentry_behavior/plugins/condition/wait_for_target_lost_stable.hpp"

#include <exception>

namespace combat_sentry_behavior
{

WaitForTargetLostStable::WaitForTargetLostStable(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus WaitForTargetLostStable::onStart()
{
  if (!getInput("target_id", target_id_)) {
    RCLCPP_ERROR(logger_, "Failed to read [target_id]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("lost_duration", lost_duration_ms_)) {
    RCLCPP_ERROR(logger_, "Failed to read [lost_duration]");
    return BT::NodeStatus::FAILURE;
  }
  if (lost_duration_ms_ < 0.0) {
    RCLCPP_WARN(logger_, "lost_duration should not be negative, clamp to 0");
    lost_duration_ms_ = 0.0;
  }

  target_seen_once_ = false;
  loss_observed_ = false;
  reported_missing_msg_ = false;
  return checkTarget();
}

BT::NodeStatus WaitForTargetLostStable::onRunning()
{
  return checkTarget();
}

void WaitForTargetLostStable::onHalted()
{
  target_seen_once_ = false;
  loss_observed_ = false;
  reported_missing_msg_ = false;
}

BT::NodeStatus WaitForTargetLostStable::checkTarget()
{
  const auto msg = getInput<combat_rm_interfaces::msg::Target>("key_port");
  if (!msg) {
    if (!reported_missing_msg_) {
      RCLCPP_WARN(logger_, "Target message is not available: %s", msg.error().c_str());
      reported_missing_msg_ = true;
    }
    loss_observed_ = false;
    return BT::NodeStatus::RUNNING;
  }
  reported_missing_msg_ = false;

  const bool target_id_match = isTargetId(msg->id);
  if (target_id_match && msg->tracking) {
    target_seen_once_ = true;
    loss_observed_ = false;
    return BT::NodeStatus::RUNNING;
  }

  if (!target_seen_once_) {
    return BT::NodeStatus::RUNNING;
  }

  if (!msg->tracking) {
    if (!loss_observed_) {
      loss_observed_ = true;
      loss_start_time_ = SteadyClock::now();
    }
    return isStableLost() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

bool WaitForTargetLostStable::isTargetId(const std::string & id) const
{
  if (id.empty()) {
    return false;
  }

  try {
    return std::stoi(id) == target_id_;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger_, "Invalid tracker target id [%s]: %s", id.c_str(), ex.what());
    return false;
  }
}

bool WaitForTargetLostStable::isStableLost() const
{
  const auto elapsed_ms =
    std::chrono::duration<double, std::milli>(SteadyClock::now() - loss_start_time_).count();
  return elapsed_ms >= lost_duration_ms_;
}

BT::PortsList WaitForTargetLostStable::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::Target>(
      "key_port", "{@tracker_target}", "Vision tracker target port on blackboard"),
    BT::InputPort<int>("target_id", 8, "Target id to observe"),
    BT::InputPort<double>(
      "lost_duration", 5000.0, "Required continuous lost duration in milliseconds"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::WaitForTargetLostStable>(
    "WaitForTargetLostStable");
}
