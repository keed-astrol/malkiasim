#ifndef DOJO_BT__BT_ACTIONS_HPP_
#define DOJO_BT__BT_ACTIONS_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace dojo_bt
{

/**
 * Emergency stop: publish zero cmd_vel immediately.
 */
class EmergencyStop : public BT::SyncActionNode
{
public:
  EmergencyStop(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

/**
 * Idle state: hold robot stationary and wait for a mission.
 */
class IdleState : public BT::StatefulActionNode
{
public:
  IdleState(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

/**
 * Compute unexplored frontiers from the current map and scan for autonomous exploration.
 * Selects the best frontier and sets it as the next goal via blackboard.
 */
class ComputeFrontiers : public BT::SyncActionNode
{
public:
  ComputeFrontiers(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

/**
 * Dock robot: final low-speed approach to a dock marker.
 */
class DockRobot : public BT::SyncActionNode
{
public:
  DockRobot(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace dojo_bt

#endif  // DOJO_BT__BT_ACTIONS_HPP_
