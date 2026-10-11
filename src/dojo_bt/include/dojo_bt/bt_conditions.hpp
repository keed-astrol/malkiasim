#ifndef DOJO_BT__BT_CONDITIONS_HPP_
#define DOJO_BT__BT_CONDITIONS_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/condition_node.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace dojo_bt
{

/**
 * Checks whether the robot is in an active mission.
 * Reads from the /dojo_mission/state topic blackboard port.
 */
class IsMissionActive : public BT::ConditionNode
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
class IsMissionType : public BT::ConditionNode
{
public:
  IsMissionType(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

/**
 * Checks if a collision is imminent based on LIDAR scan.
 * Reads min_scan_range from the blackboard.
 */
class IsCollisionImminent : public BT::ConditionNode
{
public:
  IsCollisionImminent(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  double min_scan_range_{std::numeric_limits<double>::infinity()};
  double collision_threshold_{0.25};

  void state_callback(const std_msgs::msg::String & msg);
};

/**
 * Checks if the robot is stuck (hasn't moved significantly in N seconds).
 */
class IsStuck : public BT::ConditionNode
{
public:
  IsStuck(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace dojo_bt

#endif  // DOJO_BT__BT_CONDITIONS_HPP_
