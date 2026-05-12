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

#include "combat_sentry_behavior/plugins/condition/wait_for_sentry_info_state.hpp"

namespace combat_sentry_behavior
{
namespace
{
constexpr uint8_t kMovingState = 3;
constexpr double kDefaultTimeoutMs = 5000.0;
}  // namespace

WaitForSentryInfoState::WaitForSentryInfoState(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus WaitForSentryInfoState::onStart()
{
  const auto expected_state = getInput<uint8_t>("expected_state");
  if (!expected_state) {
    RCLCPP_ERROR(logger_, "Failed to read [expected_state]: %s", expected_state.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  expected_state_ = expected_state.value();

  const auto timeout = getInput<double>("timeout");
  if (!timeout) {
    RCLCPP_ERROR(logger_, "Failed to read [timeout]: %s", timeout.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  timeout_ms_ = timeout.value();
  if (timeout_ms_ < 0.0) {
    RCLCPP_WARN(logger_, "timeout should not be negative, clamp to 0");
    timeout_ms_ = 0.0;
  }

  reported_missing_msg_ = false;
  start_time_ = SteadyClock::now();
  return checkState();
}

BT::NodeStatus WaitForSentryInfoState::onRunning()
{
  return checkState();
}

void WaitForSentryInfoState::onHalted()
{
  reported_missing_msg_ = false;
}

BT::NodeStatus WaitForSentryInfoState::checkState()
{
  const auto msg = getInput<combat_rm_interfaces::msg::SentryInfo>("key_port");
  if (!msg) {
    if (!reported_missing_msg_) {
      RCLCPP_WARN(logger_, "SentryInfo message is not available: %s", msg.error().c_str());
      reported_missing_msg_ = true;
    }
    return isTimedOut() ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
  }
  reported_missing_msg_ = false;

  if (msg->current_state == expected_state_) {
    return BT::NodeStatus::SUCCESS;
  }

  return isTimedOut() ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
}

bool WaitForSentryInfoState::isTimedOut() const
{
  const auto elapsed_ms =
    std::chrono::duration<double, std::milli>(SteadyClock::now() - start_time_).count();
  return elapsed_ms >= timeout_ms_;
}

BT::PortsList WaitForSentryInfoState::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::SentryInfo>(
      "key_port", "{@referee_sentryInfo}", "SentryInfo port on blackboard"),
    BT::InputPort<uint8_t>("expected_state", kMovingState, "Expected SentryInfo.current_state"),
    BT::InputPort<double>(
      "timeout", kDefaultTimeoutMs,
      "Timeout in milliseconds. Return SUCCESS once the expected state is observed"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::WaitForSentryInfoState>(
    "WaitForSentryInfoState");
}
