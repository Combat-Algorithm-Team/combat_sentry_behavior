#include "combat_sentry_behavior/plugins/condition/check_field_event.hpp"

namespace combat_sentry_behavior
{

CheckFieldEventCondition::CheckFieldEventCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus CheckFieldEventCondition::onStart()
{
  wait_time_ms_ = 0.0;
  if (!getInput("wait_time", wait_time_ms_)) {
    RCLCPP_ERROR(logger_, "Failed to read [wait_time]");
    return BT::NodeStatus::FAILURE;
  }
  if (wait_time_ms_ < 0.0) {
    RCLCPP_WARN(logger_, "wait_time should not be negative, clamp to 0");
    wait_time_ms_ = 0.0;
  }

  if (!readExpectedValues()) {
    return BT::NodeStatus::FAILURE;
  }

  if (checkEvent()) {
    return BT::NodeStatus::SUCCESS;
  }

  if (wait_time_ms_ <= 0.0) {
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckFieldEventCondition::onRunning()
{
  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time_);

  if (elapsed.count() >= wait_time_ms_) {
    return BT::NodeStatus::FAILURE;
  }

  return checkEvent() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void CheckFieldEventCondition::onHalted()
{
  wait_time_ms_ = 0.0;
}

bool CheckFieldEventCondition::readExpectedValues()
{
  return readExpectedValue("ally_supply_zone_non_exchange", ally_supply_zone_non_exchange_) &&
         readExpectedValue("ally_supply_zone_exchange", ally_supply_zone_exchange_) &&
         readExpectedValue("ally_supply_zone", ally_supply_zone_) &&
         readExpectedValue("ally_small_power_rune", ally_small_power_rune_) &&
         readExpectedValue("ally_big_power_rune", ally_big_power_rune_) &&
         readExpectedValue("central_highland", central_highland_) &&
         readExpectedValue("trapezoidal_highland", trapezoidal_highland_) &&
         readExpectedValue("center_gain_point", center_gain_point_) &&
         readExpectedValue("ally_fortress_gain_point", ally_fortress_gain_point_) &&
         readExpectedValue("ally_outpost_gain_point", ally_outpost_gain_point_) &&
         readExpectedValue("base_gain_point", base_gain_point_);
}

bool CheckFieldEventCondition::readExpectedValue(const char * port_name, int & value)
{
  const auto input = getInput<int>(port_name);
  if (!input) {
    RCLCPP_ERROR(logger_, "Failed to read [%s]: %s", port_name, input.error().c_str());
    value = -1;
    return false;
  }

  value = input.value();
  return true;
}

bool CheckFieldEventCondition::checkEvent()
{
  auto msg = getInput<combat_rm_interfaces::msg::EventData>("key_port");

  if (!msg) {
    RCLCPP_ERROR(logger_, "EventData message is not available");
    return false;
  }

  const auto & event_data = msg.value();
  auto check = [](int expected, int actual) { return expected == -1 || expected == actual; };

  return check(ally_supply_zone_non_exchange_, event_data.ally_supply_zone_non_exchange) &&
         check(ally_supply_zone_exchange_, event_data.ally_supply_zone_exchange) &&
         check(ally_supply_zone_, event_data.ally_supply_zone) &&
         check(ally_small_power_rune_, event_data.ally_small_power_rune) &&
         check(ally_big_power_rune_, event_data.ally_big_power_rune) &&
         check(central_highland_, event_data.central_highland) &&
         check(trapezoidal_highland_, event_data.trapezoidal_highland) &&
         check(center_gain_point_, event_data.center_gain_point) &&
         check(ally_fortress_gain_point_, event_data.ally_fortress_gain_point) &&
         check(ally_outpost_gain_point_, event_data.ally_outpost_gain_point) &&
         check(base_gain_point_, event_data.base_gain_point);
}

BT::PortsList CheckFieldEventCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::EventData>(
      "key_port", "{@referee_eventData}", "EventData message"),
    BT::InputPort<double>(
      "wait_time", 0.0, "Wait timeout in milliseconds. 0 means immediate check"),

    BT::InputPort<int>("ally_supply_zone_non_exchange", -1, "补给区(非兑换区)"),
    BT::InputPort<int>("ally_supply_zone_exchange", -1, "补给区(兑换区)"),
    BT::InputPort<int>("ally_supply_zone", -1, "补给区(RMUL)"),

    BT::InputPort<int>("ally_small_power_rune", -1, "小能量机关"),
    BT::InputPort<int>("ally_big_power_rune", -1, "大能量机关"),

    BT::InputPort<int>("central_highland", -1, "中央高地"),
    BT::InputPort<int>("trapezoidal_highland", -1, "梯形高地"),

    BT::InputPort<int>("center_gain_point", -1, "中心增益点"),
    BT::InputPort<int>("ally_fortress_gain_point", -1, "堡垒增益点"),
    BT::InputPort<int>("ally_outpost_gain_point", -1, "前哨站增益点"),
    BT::InputPort<int>("base_gain_point", -1, "基地增益点"),
  };
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::CheckFieldEventCondition>("CheckFieldEvent");
}
