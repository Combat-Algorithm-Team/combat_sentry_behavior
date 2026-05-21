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

#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "combat_sentry_behavior/custom_types.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace combat_sentry_behavior
{
class PubNav2GoalAction
: public BT::RosTopicPubStatefulActionNode<geometry_msgs::msg::PoseStamped>
{
public:
  PubNav2GoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  bool setMessage(geometry_msgs::msg::PoseStamped & goal) override;

private:
  using SteadyClock = std::chrono::steady_clock;

  template <typename T>
  bool setOutputOrThrow(const std::string & key, const T & value);

  bool ensurePublisher();

  bool publishGoal();

  void resetOutputs();

  void publishOutputs();

  bool isFresh(const BT::Timestamp & stamp, const std::chrono::milliseconds & timeout) const;

  bool isFromCurrentRun(const BT::Timestamp & stamp) const;

  bool matchesCurrentGoalPose(
    const geometry_msgs::msg::PoseStamped & goal_pose, double position_tolerance) const;

  bool pathEndsAtCurrentGoal(const nav_msgs::msg::Path & path, double position_tolerance) const;

  bool updateObservedGoalPose(
    const std::chrono::milliseconds & timeout, double position_tolerance);

  bool isAtOrAfterGoalBindDelay(
    const BT::Timestamp & stamp, const std::chrono::milliseconds & bind_delay) const;

  bool hasObservedCurrentPlan() const;

  double elapsedSinceObservedGoalPoseMs() const;

  std::optional<action_msgs::msg::GoalStatus> latestBindableStatusAfterObservedGoal(
    const action_msgs::msg::GoalStatusArray & status_array) const;

  std::optional<action_msgs::msg::GoalStatus> findStatusForGoal(
    const action_msgs::msg::GoalStatusArray & status_array,
    const unique_identifier_msgs::msg::UUID & goal_id) const;

  std::optional<action_msgs::msg::GoalStatus> selectCurrentGoalStatus(
    const action_msgs::msg::GoalStatusArray & status_array, bool allow_unbound_goal_binding);

  rclcpp::Logger logger() { return node_->get_logger(); }
  rclcpp::Time now() { return node_->now(); }

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> publisher_;
  std::string current_topic_name_;
  Pose3D current_goal_{};
  std::optional<unique_identifier_msgs::msg::UUID> current_goal_id_;
  bool observed_current_goal_pose_{false};
  bool observed_current_plan_{false};
  SteadyClock::duration observed_goal_pose_time_{};
  int64_t observed_goal_pose_ros_nanoseconds_{0};
  SteadyClock::time_point start_time_{};
  SteadyClock::time_point last_republish_time_{};
  int64_t goal_sent_ros_nanoseconds_{0};
  double last_distance_remaining_{0.0};
  double last_path_remaining_{0.0};
  int last_status_code_{0};
};
}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_
