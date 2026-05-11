#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_FIELD_EVENT_CONDITION_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_FIELD_EVENT_CONDITION_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "combat_rm_interfaces/msg/event_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace combat_sentry_behavior
{

class CheckFieldEventCondition : public BT::StatefulActionNode
{
public:
  CheckFieldEventCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  bool readExpectedValues();
  bool readExpectedValue(const char * port_name, int & value);
  bool checkEvent();

  rclcpp::Logger logger_ = rclcpp::get_logger("CheckFieldEventCondition");
  std::chrono::steady_clock::time_point start_time_;
  double wait_time_ms_{0.0};

  int ally_supply_zone_non_exchange_{-1};
  int ally_supply_zone_exchange_{-1};
  int ally_supply_zone_{-1};
  int ally_small_power_rune_{-1};
  int ally_big_power_rune_{-1};
  int central_highland_{-1};
  int trapezoidal_highland_{-1};
  int center_gain_point_{-1};
  int ally_fortress_gain_point_{-1};
  int ally_outpost_gain_point_{-1};
  int base_gain_point_{-1};
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CHECK_FIELD_EVENT_CONDITION_HPP_
