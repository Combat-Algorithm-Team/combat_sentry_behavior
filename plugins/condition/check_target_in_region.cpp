#include "combat_sentry_behavior/plugins/action/check_target_in_region.hpp"

#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace combat_sentry_behavior
{

CheckTargetInRegion::CheckTargetInRegion(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: RosTopicPubNode<std_msgs::msg::Bool>(name, config, params)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
  name + "/region_marker", 10);

  declare_parameter_if_not_declared(node_, name + ".min_x", rclcpp::ParameterValue(-5.0));
  declare_parameter_if_not_declared(node_, name + ".max_x", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(node_, name + ".min_y", rclcpp::ParameterValue(-5.0));
  declare_parameter_if_not_declared(node_, name + ".max_y", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node_, name + ".target_frame", rclcpp::ParameterValue("map"));
  declare_parameter_if_not_declared(
    node_, name + ".transform_tolerance", rclcpp::ParameterValue(0.5));

  node_->get_parameter(name + ".min_x", params_.min_x);
  node_->get_parameter(name + ".max_x", params_.max_x);
  node_->get_parameter(name + ".min_y", params_.min_y);
  node_->get_parameter(name + ".max_y", params_.max_y);
  node_->get_parameter(name + ".target_frame", params_.target_frame);
  node_->get_parameter(name + ".transform_tolerance", params_.transform_tolerance);
}

BT::PortsList CheckTargetInRegion::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<rm_interfaces::msg::Target>(
      "tracker_port", "{@tracker_target}", "Vision target from blackboard")
  });
}

bool CheckTargetInRegion::setMessage(std_msgs::msg::Bool & msg)
{
  auto tracker_target = getInput<rm_interfaces::msg::Target>("tracker_port");

  if (!tracker_target) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: tracker_port");
    msg.data = false;
    return true;
  }

  PointStamped enemy_point;
  enemy_point.header = tracker_target->header;
  enemy_point.point = tracker_target->position;

  PointStamped enemy_in_target_frame;

  if (!transformPoseInTargetFrame(
        enemy_point, enemy_in_target_frame,
        *tf_buffer_, params_.target_frame,
        params_.transform_tolerance))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to transform target");
    msg.data = false;
    return true;
  }

  const double x = enemy_in_target_frame.point.x;
  const double y = enemy_in_target_frame.point.y;

  // 判断是否在矩形区域内
  const bool in_region =
    (x >= params_.min_x && x <= params_.max_x &&
     y >= params_.min_y && y <= params_.max_y);

  msg.data = in_region;

  RCLCPP_INFO(
    node_->get_logger(),
    "Target (%.2f, %.2f) in region: %s",
    x, y, in_region ? "true" : "false");

  visualization_msgs::msg::MarkerArray marker_msg;
  createVisualizationMarkers(marker_msg);
  marker_pub_->publish(marker_msg);

  return true;
}

bool CheckTargetInRegion::transformPoseInTargetFrame(
  const PointStamped & input_pose,
  PointStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer,
  const std::string target_frame,
  const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    transformed_pose =
      tf_buffer.transform(input_pose, target_frame,
        tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(logger, "LookupException: %s", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(logger, "ConnectivityException: %s", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger, "ExtrapolationException: %s", ex.what());
  } catch (tf2::TimeoutException &) {
    RCLCPP_ERROR(logger, "Transform timeout");
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "TransformException: %s", ex.what());
  }

  return false;
}

void CheckTargetInRegion::createVisualizationMarkers(
  visualization_msgs::msg::MarkerArray & msg)
{
  msg.markers.clear();

  // ===== 区域矩形 =====
  visualization_msgs::msg::Marker region;

  region.header.frame_id = params_.target_frame;
  region.header.stamp = node_->now();

  region.ns = "region";
  region.id = 0;
  region.type = visualization_msgs::msg::Marker::LINE_STRIP;

  region.scale.x = 0.05;

  region.color.g = 1.0;
  region.color.a = 1.0;

  geometry_msgs::msg::Point p;

  // 左下
  p.x = params_.min_x;
  p.y = params_.min_y;
  p.z = 0.0;
  region.points.push_back(p);

  // 右下
  p.x = params_.max_x;
  p.y = params_.min_y;
  region.points.push_back(p);

  // 右上
  p.x = params_.max_x;
  p.y = params_.max_y;
  region.points.push_back(p);

  // 左上
  p.x = params_.min_x;
  p.y = params_.max_y;
  region.points.push_back(p);

  // 闭环
  p.x = params_.min_x;
  p.y = params_.min_y;
  region.points.push_back(p);

  msg.markers.push_back(region);
}

}  // namespace combat_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(
  combat_sentry_behavior::CheckTargetInRegion,
  "CheckTargetInRegion");