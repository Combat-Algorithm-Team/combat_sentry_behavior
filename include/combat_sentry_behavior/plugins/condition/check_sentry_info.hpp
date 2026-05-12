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

#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_SENTRY_INFO_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_SENTRY_INFO_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "combat_rm_interfaces/msg/sentry_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace combat_sentry_behavior
{

class CheckSentryInfoCondition : public BT::SimpleConditionNode
{
public:
  CheckSentryInfoCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkSentryInfo();
  bool readExpectedValue(const char * port_name, int & value);

  rclcpp::Logger logger_ = rclcpp::get_logger("CheckSentryInfoCondition");
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_SENTRY_INFO_HPP_
