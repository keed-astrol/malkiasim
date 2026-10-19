/**
 * Custom BehaviorTree condition nodes for RoboDojo autonomous navigation.
 * These plugins are loaded by nav2_bt_navigator as a shared library.
 */

#include "dojo_bt/bt_conditions.hpp"

#include <functional>
#include <limits>
#include <string>
#include <jsoncpp/json/json.h>

#include "behaviortree_cpp/bt_factory.h"

namespace dojo_bt
{

// ─── MissionStateListener ──────────────────────────────────

void MissionStateListener::init_mission_state_listener(const BT::NodeConfig & config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (node_) {
    state_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/dojo_mission/state", 10,
      std::bind(&MissionStateListener::state_callback, this, std::placeholders::_1));
  }
}

void MissionStateListener::state_callback(const std_msgs::msg::String & msg)
{
  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(msg.data, root)) {
    return;
  }
  has_mission_state_ = true;
  mission_active_ = root.get("mission_active", false).asBool();
  mission_type_ = root.get("mission_type", "NONE").asString();
  is_stuck_ = root.get("is_stuck", false).asBool();

  // The mission node publishes "min_scan_range": null when there is no usable
  // scan (or the range is infinite). jsoncpp's asDouble() would report that
  // null as 0.0 — i.e. a collision — so treat null/missing as "no obstacle".
  if (root.isMember("min_scan_range") && !root["min_scan_range"].isNull()) {
    min_scan_range_ = root["min_scan_range"].asDouble();
  } else {
    min_scan_range_ = std::numeric_limits<double>::infinity();
  }
}

// ─── IsMissionActive ───────────────────────────────────────

IsMissionActive::IsMissionActive(const std::string & name,
                                 const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
  init_mission_state_listener(config);
}

BT::PortsList IsMissionActive::providedPorts()
{
  return {
    BT::InputPort<bool>("expected", true, "Required value of the mission_active flag")
  };
}

BT::NodeStatus IsMissionActive::tick()
{
  bool expected = true;
  if (!getInput("expected", expected)) {
    return BT::NodeStatus::FAILURE;
  }
  return (mission_active() == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── IsMissionType ─────────────────────────────────────────

IsMissionType::IsMissionType(const std::string & name,
                             const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
  init_mission_state_listener(config);
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
  std::string expected;
  if (!getInput("mission_type", expected)) {
    return BT::NodeStatus::FAILURE;
  }
  return (mission_type() == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── IsCollisionImminent ───────────────────────────────────

IsCollisionImminent::IsCollisionImminent(const std::string & name,
                                         const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
  init_mission_state_listener(config);
}

BT::PortsList IsCollisionImminent::providedPorts()
{
  return {};
}

BT::NodeStatus IsCollisionImminent::tick()
{
  return (min_scan_range() < kCollisionThreshold)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── MissionStuck ──────────────────────────────────────────
// (registered as "MissionStuck" to avoid colliding with Nav2's stock IsStuck)

MissionStuck::MissionStuck(const std::string & name,
                           const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
{
  init_mission_state_listener(config);
}

BT::PortsList MissionStuck::providedPorts()
{
  // No input ports: the stuck threshold is applied by the mission node when it
  // computes "is_stuck".
  return {};
}

BT::NodeStatus MissionStuck::tick()
{
  return is_stuck() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace dojo_bt
