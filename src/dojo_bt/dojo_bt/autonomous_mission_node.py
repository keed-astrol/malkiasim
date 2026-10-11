#!/usr/bin/env python3
"""
Autonomous Mission Node for RoboDojo maze navigation.

Integrates:
  - LIDAR /scan        → collision risk, obstacle mapping
  - Wheel odom /odom   → stuck detection, displacement tracking
  - IMU /imu           → heading verification (raw angular velocity + orientation)
  - EKF /odometry/filtered → fused pose for Nav2
  - /map               → frontier computation during EXPLORE missions
  - /tf                → coordinate transforms

Commands accepted on /dojo_mission (JSON string)
and translated to Nav2 action goals (NavigateToPose, NavigateThroughPoses).

The state machine lives in MissionStateMachine; this node is the ROS2 interface
that subscribes to sensors, publishes diagnostics, and drives Nav2 actions.
"""

import json
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .mission_state_machine import (
    MissionStateMachine,
    MissionState,
    MissionType,
)


class AutonomousMissionNode(Node):
    """
    ROS2 node that manages autonomous maze navigation via a state machine.

    Publishers:
      /dojo_mission/state     (String)    - current state as JSON
      /dojo_mission/status    (String)    - human-readable status
      /cmd_vel_emergency      (Twist)     - emergency stop override

    Subscribers:
      /scan                   (LaserScan) - LIDAR for collision/frontiers
      /odom                   (Odometry)  - wheel odometry for stuck detection
      /imu                    (Imu)       - orientation + angular velocity
      /dojo_mission           (String)    - mission commands as JSON

    Action Clients:
      /navigate_to_pose       (NavigateToPose)
      /navigate_through_poses (NavigateThroughPoses)
    """

    COLLISION_MIN_RANGE = 0.25       # meters
    COLLISION_WARN_RANGE = 0.50      # meters
    COLLISION_CLEAR_RANGE = 0.40     # meters: hysteresis to exit EMERGENCY

    def __init__(self):
        super().__init__('autonomous_mission_node')

        # ─── State machine ───────────────────────────────────
        self.sm = MissionStateMachine()

        # ─── Callback groups for thread safety ────────────────
        self._sensor_cb = MutuallyExclusiveCallbackGroup()
        self._cmd_cb = MutuallyExclusiveCallbackGroup()
        self._action_cb = ReentrantCallbackGroup()

        # ─── Subscribers (sensor stream) ─────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self._on_scan, 10,
            callback_group=self._sensor_cb)

        self._odom_sub = self.create_subscription(
            Odometry, '/odom',
            self._on_odom, 10,
            callback_group=self._sensor_cb)

        self._imu_sub = self.create_subscription(
            Imu, '/imu',
            self._on_imu, 10,
            callback_group=self._sensor_cb)

        self._odom_filt_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self._on_filtered_odom, 10,
            callback_group=self._sensor_cb)

        # ─── Subscribers (command input) ─────────────────────
        self._cmd_sub = self.create_subscription(
            String, '/dojo_mission',
            self._on_mission_command, 10,
            callback_group=self._cmd_cb)

        # ─── Publishers ──────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/dojo_mission/state', 10)
        self._status_pub = self.create_publisher(String, '/dojo_mission/status', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_emergency', 10)

        # ─── Action clients ──────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self._action_cb)
        self._wp_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses',
            callback_group=self._action_cb)

        # ─── TF2 ─────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ─── Cached sensor data ──────────────────────────────
        self._latest_scan: Optional[LaserScan] = None
        self._latest_odom: Optional[Odometry] = None
        self._latest_imu: Optional[Imu] = None
        self._min_scan_range: float = float('inf')
        self._current_pose: Optional[PoseStamped] = None
        self._current_yaw: float = 0.0

        # ─── Recovery timer-based sequencing ─────────────────
        self._recovery_phase_start = self.get_clock().now()
        self._recovery_phase = -1   # -1 = not active
        self._recovery_twist = Twist()
        self._recovery_sent = False

        # ─── Timer: periodic state evaluation + status publish ──
        self._eval_timer = self.create_timer(0.1, self._evaluate_state)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Autonomous Mission Node started — state: IDLE')

    # ─── Sensor callbacks ────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        if msg.ranges:
            valid = [r for r in msg.ranges
                     if not math.isnan(r) and not math.isinf(r) and r > msg.range_min]
            self._min_scan_range = min(valid) if valid else float('inf')
        else:
            self._min_scan_range = float('inf')

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg
        self._update_current_pose(msg.pose.pose, msg.header)

    def _on_filtered_odom(self, msg: Odometry):
        self._update_current_pose(msg.pose.pose, msg.header)

    def _on_imu(self, msg: Imu):
        self._latest_imu = msg
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny, cosy)

    def _update_current_pose(self, pose, header):
        ps = PoseStamped()
        ps.header = header
        ps.pose.position = pose.position
        ps.pose.orientation = pose.orientation
        self._current_pose = ps

    # ─── Command callback ────────────────────────────────────

    def _on_mission_command(self, msg: String):
        """Parse incoming /dojo_mission JSON commands."""
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON: {msg.data}')
            return

        action = cmd.get('action', '').upper()
        self.get_logger().info(f'Mission command: {action}')

        if action == 'CANCEL':
            self.sm.cancel_mission()

        elif action == 'EXPLORE':
            self.sm.start_mission(MissionType.EXPLORE)

        elif action == 'GOAL':
            pose = self._json_to_pose(cmd.get('pose', {}))
            if pose:
                self.sm.start_mission(MissionType.NAV_TO_GOAL, target=pose)
                self._send_nav_goal(pose)

        elif action == 'WAYPOINTS':
            wps = [self._json_to_pose(p) for p in cmd.get('poses', [])]
            wps = [p for p in wps if p is not None]
            if wps:
                self.sm.start_mission(MissionType.WAYPOINT_RUN, waypoints=wps)
                self._send_waypoint_goal(wps)

        elif action == 'DOCK':
            pose = self._json_to_pose(cmd.get('dock_pose', {}))
            self.sm.start_mission(MissionType.DOCK, target=pose)
            if pose:
                self._send_nav_goal(pose)

        elif action == 'STATUS':
            summary = self.sm.get_state_summary()
            self._status_pub.publish(String(data=json.dumps(summary)))

        else:
            self.get_logger().warn(f'Unknown mission action: {action}')

    # ─── Periodic evaluation (10 Hz) ─────────────────────────

    def _evaluate_state(self):
        """Main control loop. Evaluates collision/stuck and drives recovery timing."""

        # ── Collision gate: highest priority ──
        if self._min_scan_range < self.COLLISION_MIN_RANGE:
            if self.sm.state != MissionState.EMERGENCY:
                self.sm.evaluate_collision_risk(self._min_scan_range)
            # Always publish zero velocity in emergency
            self._cmd_vel_pub.publish(Twist())
            self._recovery_phase = -1
            return

        # ── Exit emergency if scan cleared ──
        if self.sm.state == MissionState.EMERGENCY:
            if self._min_scan_range >= self.COLLISION_CLEAR_RANGE:
                self.sm.evaluate_collision_risk(self._min_scan_range)
                self._recovery_phase = -1
            else:
                self._cmd_vel_pub.publish(Twist())
            return

        # ── Check collision warning (not stopped yet) ──
        self.sm.evaluate_collision_risk(self._min_scan_range)

        # ── Stuck detection ──
        if self.sm.mission_active and self._current_pose is not None:
            self.sm.evaluate_stuck(self._current_pose)

        # ── Recovery timer-based actions ──
        if self.sm.state == MissionState.RECOVERING:
            self._handle_recovery()
        else:
            self._recovery_phase = -1  # reset when not recovering

    def _handle_recovery(self):
        """
        Timer-driven recovery sequence (non-blocking):
          Phase 0: Back up gently for 1.5s (publish negative x velocity)
          Phase 1: Spin in place for 2.5s (publish angular z velocity)
          Phase 2: Stop, clear costmaps (call clear services), mark recovery done
        """
        now = self.get_clock().now()

        if self._recovery_phase == -1:
            self._recovery_phase = 0
            self._recovery_phase_start = now
            self._recovery_sent = False
            self.get_logger().info(f'Recovery phase 0: backing up (attempt {self.sm.ctx.recovery_attempts + 1})')

        phase_elapsed = (now - self._recovery_phase_start).nanoseconds * 1e-9

        if self._recovery_phase == 0:
            # Backup: publish negative x for ~1.5s
            if phase_elapsed < 1.5:
                twist = Twist()
                twist.linear.x = -0.12
                self._cmd_vel_pub.publish(twist)
            else:
                # Stop and advance
                self._cmd_vel_pub.publish(Twist())
                self._recovery_phase = 1
                self._recovery_phase_start = now
                self.get_logger().info('Recovery phase 1: spinning')

        elif self._recovery_phase == 1:
            # Spin: publish angular z for ~2.5s
            if phase_elapsed < 2.5:
                twist = Twist()
                twist.angular.z = 0.6
                self._cmd_vel_pub.publish(twist)
            else:
                # Stop
                self._cmd_vel_pub.publish(Twist())
                self._recovery_phase = 2
                self._recovery_phase_start = now
                self.get_logger().info('Recovery phase 2: clearing costmaps')

        elif self._recovery_phase == 2:
            self._cmd_vel_pub.publish(Twist())
            if not self._recovery_sent:
                self._recovery_sent = True
                # Try to call clear costmap services
                self._clear_costmaps()
            if phase_elapsed > 1.0:
                self.sm.recovery_complete()
                self._recovery_phase = -1
                self._recovery_sent = False
                self.get_logger().info(
                    f'Recovery complete, attempts={self.sm.ctx.recovery_attempts}')

    def _clear_costmaps(self):
        """Try to clear local and global costmaps. Non-blocking."""
        from nav2_msgs.srv import ClearEntireCostmap
        local_srv = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        # Fire and forget with brief timeout
        if local_srv.wait_for_service(timeout_sec=0.5):
            local_srv.call_async(ClearEntireCostmap.Request())
        if global_srv.wait_for_service(timeout_sec=0.5):
            global_srv.call_async(ClearEntireCostmap.Request())

    # ─── Nav2 action calls ───────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped):
        """Send a single-pose goal to Nav2 NavigateToPose action server."""
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('NavigateToPose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ''

        self.get_logger().info(
            f'Sending Nav2 goal: ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f})')

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_result)

    def _send_waypoint_goal(self, poses: List[PoseStamped]):
        """Send a multi-pose goal to Nav2 NavigateThroughPoses action."""
        if not self._wp_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('NavigateThroughPoses action server not available')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ''

        self.get_logger().info(f'Sending Nav2 waypoint goal: {len(poses)} poses')

        future = self._wp_client.send_goal_async(goal)
        future.add_done_callback(self._on_wp_result)

    def _on_nav_result(self, future):
        try:
            result = future.result()
            if result.valid and result.status == 4:
                self.get_logger().info('Navigation goal reached')
                self.sm.complete_current_mission_step()
            else:
                self.get_logger().warn(f'Navigation failed: status={result.status}')
        except Exception as e:
            self.get_logger().error(f'Navigation action failed: {e}')

    def _on_wp_result(self, future):
        try:
            result = future.result()
            if result.valid and result.status == 4:
                self.get_logger().info('Waypoint run complete')
                self.sm.complete_current_mission_step()
            else:
                self.get_logger().warn(f'Waypoint run failed: status={result.status}')
        except Exception as e:
            self.get_logger().error(f'Waypoint action failed: {e}')

    # ─── Status publishing (1 Hz) ────────────────────────────

    def _publish_status(self):
        summary = self.sm.get_state_summary()
        summary['min_scan_range'] = round(self._min_scan_range, 3) if self._min_scan_range != float('inf') else None
        summary['current_yaw'] = round(self._current_yaw, 3)
        if self._current_pose:
            summary['pose_x'] = round(self._current_pose.pose.position.x, 3)
            summary['pose_y'] = round(self._current_pose.pose.position.y, 3)

        self._state_pub.publish(String(data=json.dumps(summary)))
        self._status_pub.publish(String(
            data=f"[{summary['state']}] mission={summary['mission_type']} "
                 f"active={summary['mission_active']} "
                 f"scan={summary['min_scan_range']} "
                 f"stuck={summary['is_stuck']} "
                 f"pose=({summary.get('pose_x','?')},{summary.get('pose_y','?')})"))

    def _json_to_pose(self, data: dict) -> Optional[PoseStamped]:
        """Convert JSON dict to PoseStamped in the map frame."""
        if not data:
            return None
        ps = PoseStamped()
        ps.header.frame_id = data.get('frame_id', 'map')
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(data.get('x', 0))
        ps.pose.position.y = float(data.get('y', 0))
        ps.pose.position.z = float(data.get('z', 0))
        ps.pose.orientation.z = float(data.get('qz', 0))
        ps.pose.orientation.w = float(data.get('qw', 1))
        return ps

    def destroy_node(self):
        self.get_logger().info('Autonomous Mission Node shutting down')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
