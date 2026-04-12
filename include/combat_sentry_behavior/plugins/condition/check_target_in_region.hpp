#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_TARGET_IN_REGION_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_TARGET_IN_REGION_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "combat_rm_interfaces/msg/target.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using PointStamped = geometry_msgs::msg::PointStamped;

namespace combat_sentry_behavior
{

class CheckTargetInRegion : public BT::RosTopicPubNode<std_msgs::msg::Bool>
{
public:
  CheckTargetInRegion(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(std_msgs::msg::Bool & msg) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  struct Params
  {
    double min_x, max_x;
    double min_y, max_y;
    double transform_tolerance;
    std::string target_frame;
  } params_;

  bool transformPoseInTargetFrame(
    const PointStamped & input_pose,
    PointStamped & transformed_pose,
    tf2_ros::Buffer & tf_buffer,
    const std::string target_frame,
    const double transform_timeout);

  void createVisualizationMarkers(
  visualization_msgs::msg::MarkerArray & msg);
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_TARGET_IN_REGION_HPP_
