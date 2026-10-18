#ifndef DOJO_BT__BT_CONDITIONS_HPP_
#define DOJO_BT__BT_CONDITIONS_HPP_

#include <string>
#include <memory>
#include <limits>

#include "behaviortree_cpp/condition_node.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace dojo_bt
{

/**
 * Shared /dojo_mission/state subscriber.
 *
 * The autonomous mission node publishes a JSON snapshot of its state machine on
 * /dojo_mission/state at 1 Hz. The behaviour tree runs inside bt_navigator (a
 * different process), so BT nodes cannot read the mission node's blackboard:
 * they subscribe to that topic and cache the most recent sample instead.
 *
 * Derive from this alongside a BT node type and call
 * init_mission_state_listener(config) from the node's constructor.
 */
class MissionStateListener
{
public:
  void init_mission_state_listener(const BT::NodeConfig & config);

  bool has_mission_state() const {return has_mission_state_;}
  bool mission_active() const {return mission_active_;}
  const std::string & mission_type() const {return mission_type_;}
  bool is_stuck() const {return is_stuck_;}
  double min_scan_range() const {return min_scan_range_;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  bool has_mission_state_{false};
  bool mission_active_{false};
  std::string mission_type_{"NONE"};
  bool is_stuck_{false};
  double min_scan_range_{std::numeric_limits<double>::infinity()};

  void state_callback(const std_msgs::msg::String & msg);
};

/**
 * Checks whether the robot is in an active mission.
 * Port: expected => the required value of the mission_active flag.
 */
class IsMissionActive : public BT::ConditionNode, public MissionStateListener
{
public:
  IsMissionActive(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

/**
 * Checks whether the current mission type matches the expected type.
 * Port: mission_type => "EXPLORE", "NAV_TO_GOAL", "WAYPOINT_RUN", "DOCK"
 */
class IsMissionType : public BT::ConditionNode, public MissionStateListener
{
public:
  IsMissionType(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

/**
 * Succeeds while the mission node reports an obstacle inside the emergency-stop
 * radius of the cone the robot is travelling into (its JSON "min_scan_range"
 * field, in metres, measured over the forward sector of the 360 deg lidar).
 */
class IsCollisionImminent : public BT::ConditionNode, public MissionStateListener
{
public:
  IsCollisionImminent(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

  /// Obstacle distance (m) below which a collision counts as imminent.
  static constexpr double kCollisionThreshold = 0.25;
};

/**
 * Succeeds while the mission node reports the robot as stuck.
 *
 * NOTE: registered as "MissionStuck", NOT "IsStuck" — Nav2 already ships a
 * stock IsStuck node (nav2_is_stuck_condition_bt_node) and BehaviorTree.CPP
 * throws on duplicate registration IDs.
 */
class MissionStuck : public BT::ConditionNode, public MissionStateListener
{
public:
  MissionStuck(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace dojo_bt

#endif  // DOJO_BT__BT_CONDITIONS_HPP_
