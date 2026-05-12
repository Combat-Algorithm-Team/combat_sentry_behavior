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

#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__WAIT_FOR_SENTRY_INFO_STATE_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__WAIT_FOR_SENTRY_INFO_STATE_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "combat_rm_interfaces/msg/sentry_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace combat_sentry_behavior
{

class WaitForSentryInfoState : public BT::StatefulActionNode
{
public:
  WaitForSentryInfoState(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  using SteadyClock = std::chrono::steady_clock;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  BT::NodeStatus checkState();
  bool isTimedOut() const;

  rclcpp::Logger logger_ = rclcpp::get_logger("WaitForSentryInfoState");
  uint8_t expected_state_{3};
  double timeout_ms_{5000.0};
  bool reported_missing_msg_{false};
  SteadyClock::time_point start_time_;
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__WAIT_FOR_SENTRY_INFO_STATE_HPP_
