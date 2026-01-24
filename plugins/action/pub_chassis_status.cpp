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

#include "combat_sentry_behavior/plugins/action/pub_chassis_status.hpp"

namespace combat_sentry_behavior
{

PublishChassisStatusAction::PublishChassisStatusAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishChassisStatusAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<uint8_t>("chassis_status", 0, "Chassis status (0-5)"),
  });
}

bool PublishChassisStatusAction::setMessage(example_interfaces::msg::UInt8 & msg)
{
  uint8_t chassis_status = 0;
  getInput("chassis_status", chassis_status);

  msg.data = static_cast<uint8_t>(chassis_status);

  return true;
}

bool PublishChassisStatusAction::setHaltMessage(example_interfaces::msg::UInt8 & msg)
{
  msg.data = 0;
  return true;
}

}  // namespace combat_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(combat_sentry_behavior::PublishChassisStatusAction, "PublishChassisStatus");