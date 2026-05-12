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

#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__MISSION_BLACKBOARD_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__MISSION_BLACKBOARD_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace combat_sentry_behavior
{

class CheckMissionFlag : public BT::ConditionNode
{
public:
  CheckMissionFlag(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("CheckMissionFlag");
};

class SetMissionFlag : public BT::SyncActionNode
{
public:
  SetMissionFlag(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("SetMissionFlag");
};

class CheckMissionCounter : public BT::ConditionNode
{
public:
  CheckMissionCounter(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("CheckMissionCounter");
};

class SetMissionCounter : public BT::SyncActionNode
{
public:
  SetMissionCounter(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("SetMissionCounter");
};

class IncrementMissionCounter : public BT::SyncActionNode
{
public:
  IncrementMissionCounter(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("IncrementMissionCounter");
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__MISSION_BLACKBOARD_HPP_
