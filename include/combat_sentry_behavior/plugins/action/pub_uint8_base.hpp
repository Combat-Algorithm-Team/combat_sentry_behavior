#pragma once

#include <string>
#include <chrono>
#include "example_interfaces/msg/u_int8.hpp"
#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"

namespace combat_sentry_behavior
{

class PubUInt8Base : public BT::RosTopicPubStatefulActionNode<example_interfaces::msg::UInt8>
{
public:
  PubUInt8Base(const std::string & name, const BT::NodeConfig & config, 
               const BT::RosNodeParams & params, const std::string & port_name)
  : BT::RosTopicPubStatefulActionNode<example_interfaces::msg::UInt8>(name, config, params), 
    port_name_(port_name)
  {
  }

protected:
  bool setMessage(example_interfaces::msg::UInt8 & msg) override
  {
    uint8_t value = 0;
    // 根据子类传进来的端口名字动态读取数据
    getInput(port_name_, value);
    msg.data = value;
    return true;
  }

  bool setHaltMessage(example_interfaces::msg::UInt8 & msg) override
  {
    msg.data = 0; // 默认 halt 值为 0
    return true;
  }

private:
  std::string port_name_;
};

}  // namespace combat_sentry_behavior