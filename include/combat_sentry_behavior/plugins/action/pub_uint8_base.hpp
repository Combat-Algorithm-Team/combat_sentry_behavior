#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "example_interfaces/msg/u_int8.hpp"

namespace combat_sentry_behavior
{

class PubUInt8Base : public BT::StatefulActionNode
{
public:
  PubUInt8Base(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params,
    const std::string & port_name)
  : BT::StatefulActionNode(name, config),
    node_(params.nh),
    default_topic_name_(params.default_port_value),
    port_name_(port_name)
  {
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("topic_name", "__default__placeholder__", "Topic name"),
      BT::InputPort<std::chrono::milliseconds>(
        "duration", std::chrono::milliseconds(0), "Publish then sleep duration in milliseconds"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  BT::NodeStatus onStart() override
  {
    std::string topic_name;
    if (!resolveTopicName(topic_name) || !ensurePublisher(topic_name)) {
      return BT::NodeStatus::FAILURE;
    }

    example_interfaces::msg::UInt8 msg;
    if (!setMessage(msg)) {
      return BT::NodeStatus::FAILURE;
    }
    publisher_->publish(msg);

    if (!getInput("duration", duration_)) {
      duration_ = std::chrono::milliseconds(0);
    }

    start_time_ = node_->now();
    return duration_ <= std::chrono::milliseconds(0) ? BT::NodeStatus::SUCCESS :
           BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto elapsed = node_->now() - start_time_;
    const auto elapsed_ms = elapsed.to_chrono<std::chrono::milliseconds>();
    return elapsed_ms < duration_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    if (!publisher_) {
      return;
    }

    example_interfaces::msg::UInt8 halt_msg;
    if (setHaltMessage(halt_msg)) {
      publisher_->publish(halt_msg);
    }
  }

protected:
  virtual bool setMessage(example_interfaces::msg::UInt8 & msg)
  {
    uint8_t value = 0;
    if (!getInput(port_name_, value)) {
      return false;
    }
    msg.data = value;
    return true;
  }

  virtual bool setHaltMessage(example_interfaces::msg::UInt8 & msg)
  {
    (void)msg;
    return false;
  }

private:
  bool resolveTopicName(std::string & topic_name)
  {
    const auto topic_result = getInput("topic_name", topic_name);
    if (!topic_result || topic_name.empty() || topic_name == "__default__placeholder__") {
      topic_name = default_topic_name_;
    }
    return !topic_name.empty();
  }

  bool ensurePublisher(const std::string & topic_name)
  {
    if (publisher_ && current_topic_name_ == topic_name) {
      return true;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    publisher_ = node_->create_publisher<example_interfaces::msg::UInt8>(topic_name, qos);
    current_topic_name_ = topic_name;
    return true;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::UInt8>> publisher_;
  std::string default_topic_name_;
  std::string current_topic_name_;
  std::string port_name_;
  std::chrono::milliseconds duration_{0};
  rclcpp::Time start_time_;
};

}  // namespace combat_sentry_behavior
