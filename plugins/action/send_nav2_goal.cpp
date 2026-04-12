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

#include "combat_sentry_behavior/plugins/action/send_nav2_goal.hpp"

#include <cmath>

#include "combat_sentry_behavior/custom_types.hpp"

namespace combat_sentry_behavior
{

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  Pose3D received_goal{};
  if (!getInput("goal", received_goal)) {
    RCLCPP_ERROR(logger(), "Invalid or missing required input [goal]");
    return false;
  }

  // Normalize yaw and snap tiny numeric noise to exact zero for stable origin returns.
  constexpr double kSnapEps = 1e-6;
  received_goal.x = std::abs(received_goal.x) < kSnapEps ? 0.0 : received_goal.x;
  received_goal.y = std::abs(received_goal.y) < kSnapEps ? 0.0 : received_goal.y;
  const double normalized_yaw = std::atan2(std::sin(received_goal.yaw), std::cos(received_goal.yaw));
  const double snapped_yaw = std::abs(normalized_yaw) < kSnapEps ? 0.0 : normalized_yaw;

  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();

  goal.pose.pose.position.x = received_goal.x;
  goal.pose.pose.position.y = received_goal.y;
  goal.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, snapped_yaw);
  q.normalize();
  goal.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(
    logger(), "Setting nav2 goal to (x=%.6f, y=%.6f, yaw=%.6f)", received_goal.x, received_goal.y,
    snapped_yaw);

  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Navigation succeeded!");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation canceled");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown navigation result code: %d", static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Distance remaining: %f", feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::onHalt()
{
  RCLCPP_INFO(
    logger(),
    "SendNav2GoalAction halted. Goal cancellation is handled by RosActionNode::halt().");
}

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SendNav2GoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<Pose3D>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace combat_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(combat_sentry_behavior::SendNav2GoalAction, "SendNav2Goal");
