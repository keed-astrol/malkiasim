/**
 * Custom BehaviorTree action nodes for RoboDojo autonomous navigation.
 * These plugins are loaded by nav2_bt_navigator as a shared library.
 */

#include "dojo_bt/bt_actions.hpp"
#include "dojo_bt/bt_conditions.hpp"

#include <string>
#include <cmath>

#include "behaviortree_cpp/bt_factory.h"

namespace dojo_bt
{

// ─── EmergencyStop ─────────────────────────────────────────

EmergencyStop::EmergencyStop(const std::string & name,
                             const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (node_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_emergency", 10);
  }
}

BT::PortsList EmergencyStop::providedPorts()
{
  return {};
}

BT::NodeStatus EmergencyStop::tick()
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }
  return BT::NodeStatus::SUCCESS;
}

// ─── IdleState ─────────────────────────────────────────────

IdleState::IdleState(const std::string & name,
                     const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config)
{
  init_mission_state_listener(config);
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (node) {
    cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_emergency", 10);
  }
}

BT::PortsList IdleState::providedPorts()
{
  return {};
}

BT::NodeStatus IdleState::onStart()
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus IdleState::onRunning()
{
  // Hold still until the mission node reports that a mission is active.
  if (mission_active()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }
  return BT::NodeStatus::RUNNING;
}

void IdleState::onHalted()
{
  // Nothing to clean up
}

// ─── ComputeFrontiers ──────────────────────────────────────

ComputeFrontiers::ComputeFrontiers(const std::string & name,
                                   const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ComputeFrontiers::providedPorts()
{
  return {
    BT::InputPort<std::string>("scan_topic", "/scan", "LIDAR scan topic"),
    BT::InputPort<std::string>("map_topic", "/map", "Occupancy grid map topic"),
    BT::InputPort<std::string>("global_frame", "map", "Global frame for frontier poses"),
  };
}

BT::NodeStatus ComputeFrontiers::tick()
{
  // Stub: frontier computation would happen here in a full implementation.
  // For now, this returns SUCCESS to allow the tree to proceed.
  // In a full implementation, this would:
  //   1. Subscribe to /map (OccupancyGrid)
  //   2. Run edge detection to find boundaries between known free (0) and unknown (-1)
  //   3. Cluster frontier cells into candidate regions
  //   4. Select the best frontier (largest/closest)
  //   5. Compute a goal pose facing that frontier
  //   6. Write the pose to the blackboard for NavigateToFrontier
  return BT::NodeStatus::SUCCESS;
}

// ─── MissionDock ───────────────────────────────────────────
// (registered as "MissionDock" to avoid colliding with Nav2's DockRobot)

MissionDock::MissionDock(const std::string & name,
                         const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList MissionDock::providedPorts()
{
  return {};
}

BT::NodeStatus MissionDock::tick()
{
  // Stub: final docking approach would happen here.
  // In a full implementation, this would:
  //   1. Switch to a dock-specific controller
  //   2. Use a short-range sensor (IR/ultrasonic or visual marker)
  //   3. Slowly approach until alignment threshold is met
  //   4. Confirm dock engagement
  //
  // For the initial implementation, NavigateToPose handles the approach
  // and this node acts as a terminal confirmation.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace dojo_bt

// ─── Plugin registration ───────────────────────────────────
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<dojo_bt::EmergencyStop>("EmergencyStop");
  factory.registerNodeType<dojo_bt::IdleState>("IdleState");
  factory.registerNodeType<dojo_bt::ComputeFrontiers>("ComputeFrontiers");
  factory.registerNodeType<dojo_bt::MissionDock>("MissionDock");
  factory.registerNodeType<dojo_bt::IsMissionActive>("IsMissionActive");
  factory.registerNodeType<dojo_bt::IsMissionType>("IsMissionType");
  factory.registerNodeType<dojo_bt::IsCollisionImminent>("IsCollisionImminent");
  factory.registerNodeType<dojo_bt::MissionStuck>("MissionStuck");
}
