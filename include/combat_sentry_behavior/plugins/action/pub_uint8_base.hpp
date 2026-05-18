#pragma once

#include <chrono>
#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "example_interfaces/msg/u_int8.hpp"

namespace combat_sentry_behavior
{

class PubUInt8Base : public BT::RosTopicPubStatefulActionNode<example_interfaces::msg::UInt8>
{
public:
  PubUInt8Base(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params,
    const std::string & port_name)
  : BT::RosTopicPubStatefulActionNode<example_interfaces::msg::UInt8>(name, config, params),
    port_name_(port_name)
  {
  }

  BT::NodeStatus onStart() override
  {
    const auto status =
      BT::RosTopicPubStatefulActionNode<example_interfaces::msg::UInt8>::onStart();
    if (status != BT::NodeStatus::RUNNING) {
      return status;
    }

    std::chrono::milliseconds duration{0};
    if (!getInput("duration", duration)) {
      duration = std::chrono::milliseconds{0};
    }

    return duration <= std::chrono::milliseconds{0} ? BT::NodeStatus::SUCCESS : status;
  }

protected:
  bool setMessage(example_interfaces::msg::UInt8 & msg) override
  {
    uint8_t value = 0;
    if (!getInput(port_name_, value)) {
      return false;
    }
    msg.data = value;
    return true;
  }

  bool setHaltMessage(example_interfaces::msg::UInt8 & msg) override
  {
    (void)msg;
    return false;
  }

private:
  std::string port_name_;
};

}  // namespace combat_sentry_behavior
