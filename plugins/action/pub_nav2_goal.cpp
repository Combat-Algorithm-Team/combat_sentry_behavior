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

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "combat_sentry_behavior/custom_types.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

double computePathLength(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1].pose.position;
    const auto & current = path.poses[i].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

}  // namespace

namespace combat_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
  resetOutputs();
}

template <typename T>
bool PubNav2GoalAction::setOutputOrThrow(const std::string & key, const T & value)
{
  const auto result = setOutput(key, value);
  if (!result) {
    throw BT::RuntimeError(result.error());
  }
  return true;
}

bool PubNav2GoalAction::ensurePublisher()
{
  std::string topic_name;
  const auto topic_result = getInput("topic_name", topic_name);
  if (!topic_result || topic_name.empty() || topic_name == "__default__placeholder__") {
    topic_name = prev_topic_name_;
  }

  if (topic_name.empty()) {
    RCLCPP_ERROR(logger(), "PubNav2GoalAction requires a non-empty topic_name");
    return false;
  }

  if (!publisher_ || current_topic_name_ != topic_name) {
    publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::QoS(10));
    current_topic_name_ = topic_name;
  }
  return true;
}

bool PubNav2GoalAction::publishGoal()
{
  if (!ensurePublisher()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped message;
  if (!setMessage(message)) {
    return false;
  }

  goal_sent_ros_nanoseconds_ = rclcpp::Time(message.header.stamp).nanoseconds();
  publisher_->publish(message);
  last_republish_time_ = SteadyClock::now();
  return true;
}

void PubNav2GoalAction::resetOutputs()
{
  last_distance_remaining_ = std::numeric_limits<double>::infinity();
  last_path_remaining_ = std::numeric_limits<double>::infinity();
  last_status_code_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
}

void PubNav2GoalAction::publishOutputs()
{
  setOutputOrThrow("distance_remaining", last_distance_remaining_);
  setOutputOrThrow("path_remaining", last_path_remaining_);
  setOutputOrThrow("nav_status_code", last_status_code_);
}

bool PubNav2GoalAction::isFresh(
  const BT::Timestamp & stamp, const std::chrono::milliseconds & timeout) const
{
  const auto now = SteadyClock::now().time_since_epoch();
  return now >= stamp.time && (now - stamp.time) <= timeout;
}

bool PubNav2GoalAction::isFromCurrentRun(const BT::Timestamp & stamp) const
{
  return stamp.time >= start_time_.time_since_epoch();
}

std::optional<int> PubNav2GoalAction::latestStatusCodeAfterGoal(
  const action_msgs::msg::GoalStatusArray & status_array) const
{
  if (status_array.status_list.empty()) {
    return std::nullopt;
  }

  const action_msgs::msg::GoalStatus * latest_status = nullptr;
  int64_t latest_stamp_nanoseconds = 0;

  for (const auto & status : status_array.status_list) {
    const auto current_stamp_nanoseconds = rclcpp::Time(status.goal_info.stamp).nanoseconds();
    if (current_stamp_nanoseconds < goal_sent_ros_nanoseconds_) {
      continue;
    }
    if (latest_status == nullptr || current_stamp_nanoseconds > latest_stamp_nanoseconds) {
      latest_status = &status;
      latest_stamp_nanoseconds = current_stamp_nanoseconds;
    }
  }

  if (latest_status == nullptr) {
    return std::nullopt;
  }

  return latest_status->status;
}

BT::NodeStatus PubNav2GoalAction::onStart()
{
  const auto goal_result = getInput("goal", current_goal_);
  if (!goal_result) {
    RCLCPP_ERROR(
      logger(), "Invalid or missing required input [goal]: %s", goal_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  resetOutputs();
  start_time_ = SteadyClock::now();
  last_republish_time_ = start_time_;
  publishOutputs();

  if (!publishGoal()) {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    logger(), "Publishing nav2 goal to %s: (x=%.3f, y=%.3f, yaw=%.3f)",
    current_topic_name_.c_str(), current_goal_.x, current_goal_.y, current_goal_.yaw);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PubNav2GoalAction::onRunning()
{
  std::chrono::milliseconds republish_period(0);
  const auto republish_result = getInput("goal_republish_period", republish_period);
  if (!republish_result) {
    RCLCPP_ERROR(
      logger(), "Failed to read [goal_republish_period]: %s", republish_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::chrono::milliseconds observation_timeout(2000);
  const auto timeout_result = getInput("nav_observation_timeout", observation_timeout);
  if (!timeout_result) {
    RCLCPP_ERROR(
      logger(), "Failed to read [nav_observation_timeout]: %s", timeout_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  double success_distance_tolerance = 0.15;
  const auto tolerance_result =
    getInput("success_distance_tolerance", success_distance_tolerance);
  if (!tolerance_result) {
    RCLCPP_ERROR(
      logger(), "Failed to read [success_distance_tolerance]: %s",
      tolerance_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto now_steady = SteadyClock::now();
  if (republish_period.count() > 0 &&
    (now_steady - last_republish_time_) >= republish_period && !publishGoal())
  {
    return BT::NodeStatus::FAILURE;
  }

  bool has_current_goal_status = false;
  bool has_fresh_distance_remaining = false;
  bool has_fresh_path_remaining = false;

  if (const auto stamped_feedback =
        getInputStamped<nav2_msgs::action::NavigateToPose::Feedback>("feedback_port")) {
    if (isFresh(stamped_feedback->stamp, observation_timeout) &&
      isFromCurrentRun(stamped_feedback->stamp))
    {
      last_distance_remaining_ = stamped_feedback->value.distance_remaining;
      has_fresh_distance_remaining = true;
    }
  }

  if (const auto stamped_plan = getInputStamped<nav_msgs::msg::Path>("plan_port")) {
    if (isFresh(stamped_plan->stamp, observation_timeout) &&
      isFromCurrentRun(stamped_plan->stamp))
    {
      last_path_remaining_ = computePathLength(stamped_plan->value);
      has_fresh_path_remaining = true;
    }
  }

  if (const auto stamped_status =
        getInputStamped<action_msgs::msg::GoalStatusArray>("status_port")) {
    if (isFresh(stamped_status->stamp, observation_timeout) &&
      isFromCurrentRun(stamped_status->stamp))
    {
      const auto current_goal_status = latestStatusCodeAfterGoal(stamped_status->value);
      if (current_goal_status) {
        last_status_code_ = *current_goal_status;
        has_current_goal_status = true;
      } else {
        last_status_code_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
      }
    }
  }

  if (!has_current_goal_status) {
    last_status_code_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  }

  publishOutputs();

  switch (last_status_code_) {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      return BT::NodeStatus::FAILURE;

    default:
      break;
  }

  if (has_current_goal_status && has_fresh_distance_remaining &&
    last_distance_remaining_ <= success_distance_tolerance)
  {
    return BT::NodeStatus::SUCCESS;
  }

  if (has_current_goal_status && !has_fresh_distance_remaining && has_fresh_path_remaining &&
    last_path_remaining_ <= success_distance_tolerance)
  {
    return BT::NodeStatus::SUCCESS;
  }

  if (!has_current_goal_status && (now_steady - start_time_) > observation_timeout) {
    publishOutputs();
    RCLCPP_WARN(
      logger(),
      "PubNav2GoalAction timed out waiting for current nav2 goal status for goal "
      "(%.3f, %.3f, %.3f)",
      current_goal_.x, current_goal_.y, current_goal_.yaw);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void PubNav2GoalAction::onHalted()
{
  RCLCPP_DEBUG(
    logger(),
    "PubNav2GoalAction halted without canceling nav2 goal (x=%.3f, y=%.3f, yaw=%.3f)",
    current_goal_.x, current_goal_.y, current_goal_.yaw);
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose.position.x = current_goal_.x;
  msg.pose.position.y = current_goal_.y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, current_goal_.yaw);
  quaternion.normalize();
  msg.pose.orientation = tf2::toMsg(quaternion);

  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "__default__placeholder__", "Topic name"),
    BT::InputPort<Pose3D>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<action_msgs::msg::GoalStatusArray>(
      "status_port", "{@nav2_navigate_to_pose_status}", "Latest nav2 action status list"),
    BT::InputPort<nav2_msgs::action::NavigateToPose::Feedback>(
      "feedback_port", "{@nav2_navigate_to_pose_feedback}", "Latest nav2 feedback"),
    BT::InputPort<nav_msgs::msg::Path>(
      "plan_port", "{@nav2_plan}", "Latest nav2 global plan"),
    BT::InputPort<double>(
      "success_distance_tolerance", 0.15, "Distance threshold that can finish navigation"),
    BT::InputPort<std::chrono::milliseconds>(
      "goal_republish_period", std::chrono::milliseconds(0),
      "How often to republish the same goal while running. Use 0 to publish only on start"),
    BT::InputPort<std::chrono::milliseconds>(
      "nav_observation_timeout", std::chrono::milliseconds(2000),
      "How long to wait for fresh nav2 status, feedback, or plan before failing"),
    BT::OutputPort<double>(
      "distance_remaining", "{nav_distance_remaining}", "Latest nav2 distance remaining"),
    BT::OutputPort<double>(
      "path_remaining", "{nav_path_remaining}", "Latest computed remaining path length"),
    BT::OutputPort<int>(
      "nav_status_code", "{nav_status_code}", "Latest nav2 action status code"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(combat_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
