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

#include "combat_sentry_behavior/plugins/action/pub_nav2_goal.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace combat_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<Pose3D>(
      "goal", "0;0;0", "Goal pose published to Nav2 in x;y;yaw format"),
  });
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  Pose3D goal;
  const auto goal_result = getInput("goal", goal);
  if (!goal_result) {
    RCLCPP_ERROR(
      logger(), "Invalid or missing required input [goal]: %s", goal_result.error().c_str());
    return false;
  }

  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = goal.x;
  msg.pose.position.y = goal.y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, goal.yaw);
  quaternion.normalize();
  msg.pose.orientation = tf2::toMsg(quaternion);

  RCLCPP_INFO(
    logger(), "Publishing nav2 goal: (x=%.3f, y=%.3f, yaw=%.3f)", goal.x, goal.y, goal.yaw);
  return true;
}

}  // namespace combat_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(combat_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
