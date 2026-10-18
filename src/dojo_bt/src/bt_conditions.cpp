/**
 * Custom BehaviorTree condition nodes for RoboDojo autonomous navigation.
 * These plugins are loaded by nav2_bt_navigator as a shared library.
 */

#include "dojo_bt/bt_conditions.hpp"

#include <string>
#include <limits>
#include <jsoncpp/json/json.h>

#include "behaviortree_cpp/bt_factory.h"

namespace dojo_bt
{

// ─── IsMissionActive ───────────────────────────────────────

IsMissionActive::IsMissionActive(const std::string & name,
                                 const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList IsMissionActive::providedPorts()
{
  return {
    BT::InputPort<bool>("expected", true, "Expected value of mission active flag")
  };
}

BT::NodeStatus IsMissionActive::tick()
{
  // Read from blackboard; the autonomous_mission_node writes state JSON here
  std::string state_json;
  if (config().blackboard->get("mission_state", state_json)) {
    try {
      Json::Value root;
      Json::Reader reader;
      if (reader.parse(state_json, root)) {
        bool active = root.get("mission_active", false).asBool();
        bool expected;
        getInput("expected", expected);
        return (active == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
    } catch (...) {}
  }
  return BT::NodeStatus::FAILURE;
}

// ─── IsMissionType ─────────────────────────────────────────

IsMissionType::IsMissionType(const std::string & name,
                             const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList IsMissionType::providedPorts()
{
  return {
    BT::InputPort<std::string>("mission_type", "NONE",
                               "Expected mission type: EXPLORE, NAV_TO_GOAL, WAYPOINT_RUN, DOCK")
  };
}

BT::NodeStatus IsMissionType::tick()
{
  std::string state_json;
  if (config().blackboard->get("mission_state", state_json)) {
    try {
      Json::Value root;
      Json::Reader reader;
      if (reader.parse(state_json, root)) {
        std::string current = root.get("mission_type", "NONE").asString();
        std::string expected;
        getInput("mission_type", expected);
        return (current == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
    } catch (...) {}
  }
  return BT::NodeStatus::FAILURE;
}

// ─── IsCollisionImminent ───────────────────────────────────

IsCollisionImminent::IsCollisionImminent(const std::string & name,
                                         const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (node_) {
    state_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/dojo_mission/state", 10,
      std::bind(&IsCollisionImminent::state_callback, this, std::placeholders::_1));
  }
}

BT::PortsList IsCollisionImminent::providedPorts()
{
  return {};
}

void IsCollisionImminent::state_callback(const std_msgs::msg::String & msg)
{
  try {
    Json::Value root;
    Json::Reader reader;
    if (reader.parse(msg.data, root)) {
      min_scan_range_ = root.get("min_scan_range", std::numeric_limits<double>::infinity()).asDouble();
    }
  } catch (...) {}
}

BT::NodeStatus IsCollisionImminent::tick()
{
  return (min_scan_range_ < collision_threshold_)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── IsStuck ───────────────────────────────────────────────

IsStuck::IsStuck(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList IsStuck::providedPorts()
{
  return {
    BT::InputPort<double>("stuck_threshold_sec", 5.0,
                           "Seconds without significant movement before declaring stuck")
  };
}

BT::NodeStatus IsStuck::tick()
{
  std::string state_json;
  if (config().blackboard->get("mission_state", state_json)) {
    try {
      Json::Value root;
      Json::Reader reader;
      if (reader.parse(state_json, root)) {
        bool stuck = root.get("is_stuck", false).asBool();
        return stuck ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
    } catch (...) {}
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace dojo_bt

