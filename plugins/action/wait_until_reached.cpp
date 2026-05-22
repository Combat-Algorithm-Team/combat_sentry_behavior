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

#include "combat_sentry_behavior/plugins/action/wait_until_reached.hpp"

#include <cmath>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace combat_sentry_behavior
{

WaitUntilReached::WaitUntilReached(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf), node_(params.nh)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList WaitUntilReached::providedPorts()
{
  return {
    BT::InputPort<Pose3D>("target", "0;0;0", "Target position in x;y;yaw format"),
    BT::InputPort<double>("tolerance", 0.1, "Distance tolerance to consider reached"),
    BT::InputPort<std::string>("target_frame", "map", "Frame where the target pose is defined"),
    BT::InputPort<std::string>("robot_frame", "base_yaw_odom", "Robot frame used for TF lookup"),
    BT::InputPort<double>("transform_timeout", 0.05, "TF lookup timeout in seconds"),
  };
}

BT::NodeStatus WaitUntilReached::onStart()
{
  const auto target_result = getInput<Pose3D>("target", target_);
  if (!target_result) {
    RCLCPP_ERROR(logger(), "Failed to read [target]: %s", target_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto tolerance_result = getInput<double>("tolerance", tolerance_);
  if (!tolerance_result) {
    RCLCPP_ERROR(logger(), "Failed to read [tolerance]: %s", tolerance_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (tolerance_ < 0.0) {
    RCLCPP_WARN(logger(), "tolerance should not be negative, clamp to 0");
    tolerance_ = 0.0;
  }

  const auto target_frame_result = getInput<std::string>("target_frame", target_frame_);
  if (!target_frame_result || target_frame_.empty()) {
    RCLCPP_ERROR(logger(), "Failed to read a non-empty [target_frame]");
    return BT::NodeStatus::FAILURE;
  }

  const auto robot_frame_result = getInput<std::string>("robot_frame", robot_frame_);
  if (!robot_frame_result || robot_frame_.empty()) {
    RCLCPP_ERROR(logger(), "Failed to read a non-empty [robot_frame]");
    return BT::NodeStatus::FAILURE;
  }

  const auto transform_timeout_result = getInput<double>("transform_timeout", transform_timeout_);
  if (!transform_timeout_result) {
    RCLCPP_ERROR(
      logger(), "Failed to read [transform_timeout]: %s", transform_timeout_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (transform_timeout_ < 0.0) {
    RCLCPP_WARN(logger(), "transform_timeout should not be negative, clamp to 0");
    transform_timeout_ = 0.0;
  }

  RCLCPP_INFO(
    logger(),
    "WaitUntilReached started: target=(%.3f, %.3f, %.3f), tolerance=%.3f, "
    "frames=%s<- %s",
    target_.x, target_.y, target_.yaw, tolerance_, target_frame_.c_str(), robot_frame_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilReached::onRunning()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(transform_timeout_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger(), *node_->get_clock(), 2000, "TF lookup failed: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double dx = transform.transform.translation.x - target_.x;
  const double dy = transform.transform.translation.y - target_.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= tolerance_) {
    RCLCPP_INFO(
      logger(), "WaitUntilReached: SUCCESS (distance=%.3f <= tolerance=%.3f)",
      distance, tolerance_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitUntilReached::onHalted()
{
  RCLCPP_DEBUG(logger(), "WaitUntilReached halted");
}

}  // namespace combat_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<combat_sentry_behavior::WaitUntilReached>("WaitUntilReached", params);
  factory.registerNodeType<combat_sentry_behavior::WaitUntilReached>("WaitUntilReach", params);
}
