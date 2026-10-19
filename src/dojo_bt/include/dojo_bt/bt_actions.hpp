#ifndef DOJO_BT__BT_ACTIONS_HPP_
#define DOJO_BT__BT_ACTIONS_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "dojo_bt/bt_conditions.hpp"

namespace dojo_bt
{

/**
 * Emergency stop: publish zero velocity on /cmd_vel_emergency.
 *
 * Available to behaviour trees, but the stock RoboDojo tree does not use it:
 * while a mission runs, autonomous_mission_node is the single writer of
 * /cmd_vel_emergency (it publishes the stop *and* the escape manoeuvre). Two
 * publishers on that high-priority mux input would fight each other, so the
 * mission node keeps ownership and the tree only withholds controller commands
 * via IsCollisionImminent.
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
 * Idle state: hold robot stationary until the mission node reports that a
 * mission has been commanded. Reads /dojo_mission/state via
 * MissionStateListener.
 */
class IdleState : public BT::StatefulActionNode, public MissionStateListener
{
public:
  IdleState(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
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
 *
 * NOTE: registered as "MissionDock", NOT "DockRobot" — Nav2 ships a DockRobot
 * behaviour (nav2_docking) and BehaviorTree.CPP throws on duplicate IDs.
 */
class MissionDock : public BT::SyncActionNode
{
public:
  MissionDock(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace dojo_bt

#endif  // DOJO_BT__BT_ACTIONS_HPP_
