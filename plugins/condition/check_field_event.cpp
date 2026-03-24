#include "combat_sentry_behavior/plugins/condition/check_field_event.hpp"

namespace combat_sentry_behavior
{

CheckFieldEventCondition::CheckFieldEventCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&CheckFieldEventCondition::checkEvent, this), config)
{
}

BT::NodeStatus CheckFieldEventCondition::checkEvent()
{
  auto msg = getInput<combat_rm_interfaces::msg::FieldEvent>("key_port");

  if (!msg) {
    RCLCPP_ERROR(logger_, "FieldEvent message is not available");
    return BT::NodeStatus::FAILURE;
  }

  // 默认 -1 表示忽略
  int ally_supply_zone_non_exchange = -1;
  int ally_supply_zone_exchange = -1;
  int ally_supply_zone = -1;

  int ally_small_power_rune = -1;
  int ally_big_power_rune = -1;

  int central_highland = -1;
  int trapezoidal_highland = -1;

  int center_gain_point = -1;
  int ally_fortress_gain_point = -1;
  int ally_outpost_gain_point = -1;
  int base_gain_point = -1;

  // 读取输入
  getInput("ally_supply_zone_non_exchange", ally_supply_zone_non_exchange);
  getInput("ally_supply_zone_exchange", ally_supply_zone_exchange);
  getInput("ally_supply_zone", ally_supply_zone);

  getInput("ally_small_power_rune", ally_small_power_rune);
  getInput("ally_big_power_rune", ally_big_power_rune);

  getInput("central_highland", central_highland);
  getInput("trapezoidal_highland", trapezoidal_highland);

  getInput("center_gain_point", center_gain_point);
  getInput("ally_fortress_gain_point", ally_fortress_gain_point);
  getInput("ally_outpost_gain_point", ally_outpost_gain_point);
  getInput("base_gain_point", base_gain_point);

  // 统一检查函数
  auto check = [](int input, int actual) { return (input == -1) || (input == actual); };

  bool result = check(ally_supply_zone_non_exchange, msg->ally_supply_zone_non_exchange) &&
                check(ally_supply_zone_exchange, msg->ally_supply_zone_exchange) &&
                check(ally_supply_zone, msg->ally_supply_zone) &&

                check(ally_small_power_rune, msg->ally_small_power_rune) &&
                check(ally_big_power_rune, msg->ally_big_power_rune) &&

                check(central_highland, msg->central_highland) &&
                check(trapezoidal_highland, msg->trapezoidal_highland) &&

                check(center_gain_point, msg->center_gain_point) &&
                check(ally_fortress_gain_point, msg->ally_fortress_gain_point) &&
                check(ally_outpost_gain_point, msg->ally_outpost_gain_point) &&
                check(base_gain_point, msg->base_gain_point);

  return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList CheckFieldEventCondition::providedPorts()
{
  return {
    BT::InputPort<combat_rm_interfaces::msg::FieldEvent>(
      "key_port", "{@field_event}", "FieldEvent message"),

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