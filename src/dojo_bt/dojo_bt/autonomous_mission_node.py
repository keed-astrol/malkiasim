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
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

from dojo_bt.mission_state_machine import (
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
      /scan                   (LaserScan) - LIDAR for collision risk
      /map                    (OccupancyGrid) - frontier source for EXPLORE
      /odom                   (Odometry)  - wheel odometry for stuck detection
      /imu                    (Imu)       - orientation + angular velocity
      /dojo_mission           (String)    - mission commands as JSON

    Action Clients:
      /navigate_to_pose       (NavigateToPose)
      /navigate_through_poses (NavigateThroughPoses)

    EXPLORE missions pick frontier goals (known-free cells on the border of
    unknown space) from /map, drive to the best one and repeat. That map can
    come from a prebuilt map + AMCL or from slam_toolbox building it live
    (launch with localization:=slam).
    """

    COLLISION_MIN_RANGE = 0.25       # meters
    COLLISION_WARN_RANGE = 0.50      # meters
    COLLISION_CLEAR_RANGE = 0.40     # meters: hysteresis to exit EMERGENCY

    # Frontier exploration (EXPLORE)
    FRONTIER_MIN_CELLS = 4           # ignore frontier specks
    FRONTIER_MIN_GOAL_DIST = 0.40    # m: don't chase something under the robot
    FRONTIER_MAX_GOAL_DIST = 15.0    # m: sanity cap
    FRONTIER_CLEARANCE = 0.20        # m: free space required around the goal cell
    FRONTIER_BLACKLIST_TIME = 90.0   # s: forget a failed frontier after this
    FRONTIER_RETRY_PERIOD = 1.0      # s: minimum gap between frontier selections
    FRONTIER_EMPTY_ATTEMPTS = 3      # consecutive empty scans before giving up

    # Collision checks only look at the cone the robot is travelling into. The
    # lidar is a 360 deg unit, so using the global minimum range would flag the
    # maze walls beside and behind the robot as an "imminent collision" and
    # freeze it permanently (the arena is only a few tenths of a metre wide).
    FORWARD_HALF_ANGLE = math.radians(35.0)   # front cone, centred on +x
    REAR_HALF_ANGLE = math.radians(35.0)      # rear cone, checked before reversing
    REAR_CLEAR_RANGE = 0.30                   # only reverse if the rear is clear

    # EMERGENCY escape: a short hard stop, then reverse (or spin in place when
    # the rear is blocked) so a blocked robot can free itself. Repeats while the
    # way ahead stays blocked.
    EMERGENCY_STOP_TIME = 0.3        # seconds of zero velocity before escaping
    EMERGENCY_ESCAPE_TIME = 1.7      # seconds of reverse/spin per cycle
    EMERGENCY_REVERSE_SPEED = 0.10   # m/s
    EMERGENCY_SPIN_SPEED = 0.60      # rad/s

    # Stuck recovery: this node only bookkeeps. The tree's MissionStuck branch
    # owns the actual back-up/spin so two systems never fight over the wheels.
    RECOVERY_GRACE = 3.0             # seconds to let Nav2's own recovery run
    RECOVERY_SETTLE = 1.0            # seconds after clearing the costmaps

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

        # /map is latched by map_server and slam_toolbox, so use transient_local
        # to get the last map immediately instead of waiting for an update.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map',
            self._on_map, map_qos,
            callback_group=self._sensor_cb)

        # ─── Frontier exploration state ──────────────────────
        self._latest_map: Optional[OccupancyGrid] = None
        self._goal_in_flight = False
        self._frontier_count = 0
        self._frontier_empties = 0
        self._last_frontier_attempt = self.get_clock().now()
        self._frontier_blacklist = {}   # (ix, iy) -> expiry (sim seconds)
        self._current_frontier = None

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
        self._min_scan_range: float = float('inf')          # front cone (collision gate)
        self._min_scan_range_all: float = float('inf')      # whole sweep (diagnostics)
        self._min_scan_range_rear: float = float('inf')     # rear cone (escape safety)
        self._front_idx = None                              # cached scan index windows
        self._rear_idx = None
        self._current_pose: Optional[PoseStamped] = None
        self._current_yaw: float = 0.0

        # ─── Recovery timer-based sequencing ─────────────────
        self._recovery_phase_start = self.get_clock().now()
        self._recovery_phase = -1   # -1 = not active
        self._recovery_sent = False

        # ─── Emergency (collision stop/escape) sequencing ────
        self._emergency_start = self.get_clock().now()

        # ─── Timer: periodic state evaluation + status publish ──
        self._eval_timer = self.create_timer(0.1, self._evaluate_state)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Autonomous Mission Node started — state: IDLE')

    # ─── Sensor callbacks ────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        if self._front_idx is None:
            self._build_sector_windows(msg)

        # Collision decisions use the front cone only; the full-sweep and rear
        # minima are published for diagnostics and for escape safety.
        self._min_scan_range = self._window_min(msg, self._front_idx)
        self._min_scan_range_rear = self._window_min(msg, self._rear_idx)
        valid = [r for r in msg.ranges if math.isfinite(r) and r > msg.range_min]
        self._min_scan_range_all = min(valid) if valid else float('inf')

    def _build_sector_windows(self, msg: LaserScan):
        """Cache which scan indices fall inside the front/rear cones."""
        n = len(msg.ranges)
        if n == 0:
            self._front_idx, self._rear_idx = [], []
            return
        inc = msg.angle_increment
        if not math.isfinite(inc) or inc <= 0.0:
            inc = 2.0 * math.pi / n

        front, rear = [], []
        for i in range(n):
            # Deflection from the front (+x) / rear (-x) direction, in [-pi, pi].
            a = msg.angle_min + i * inc
            if abs(math.atan2(math.sin(a), math.cos(a))) <= self.FORWARD_HALF_ANGLE:
                front.append(i)
            if abs(math.atan2(math.sin(a - math.pi),
                              math.cos(a - math.pi))) <= self.REAR_HALF_ANGLE:
                rear.append(i)

        self._front_idx, self._rear_idx = front, rear
        if not front:
            self.get_logger().warn(
                'Lidar scan produced an empty front cone; collision gate disabled')

    @staticmethod
    def _window_min(msg: LaserScan, indices) -> float:
        """Smallest valid range over the given scan indices."""
        best = float('inf')
        for i in indices:
            r = msg.ranges[i]
            if r > msg.range_min and math.isfinite(r) and r < best:
                best = r
        return best

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
            self._frontier_empties = 0
            self._goal_in_flight = False
            self._current_frontier = None
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
        # Entered when something appears in the cone the robot drives into, and
        # left again once the way ahead is clear (with hysteresis so the robot
        # does not flap on the threshold). The mission node owns
        # /cmd_vel_emergency while a mission runs, so these commands always
        # outrank Nav2's on twist_mux.
        collision_risk = self._min_scan_range < self.COLLISION_MIN_RANGE
        in_emergency = self.sm.state == MissionState.EMERGENCY
        if collision_risk or in_emergency:
            if not in_emergency:
                self.sm.evaluate_collision_risk(self._min_scan_range)
                self._emergency_start = self.get_clock().now()
            self._recovery_phase = -1
            if self._min_scan_range >= self.COLLISION_CLEAR_RANGE:
                # Clear: return control to Nav2 (resumes the mission state).
                self.sm.evaluate_collision_risk(self._min_scan_range)
            else:
                self._handle_emergency()
            return

        # ── Check collision warning (not stopped yet) ──
        self.sm.evaluate_collision_risk(self._min_scan_range)

        # ── Stuck detection ──
        if self.sm.mission_active and self._current_pose is not None:
            self.sm.evaluate_stuck(self._current_pose)

        # ── EXPLORE: dispatch the next frontier goal ──
        if (self.sm.mission_active
                and self.sm.mission_type == MissionType.EXPLORE
                and self.sm.state == MissionState.EXPLORING
                and not self._goal_in_flight):
            self._dispatch_explore_goal()

        # ── Recovery timer-based actions ──
        if self.sm.state == MissionState.RECOVERING:
            self._handle_recovery()
        else:
            self._recovery_phase = -1  # reset when not recovering

    def _handle_emergency(self):
        """
        Collision-stop escape (non-blocking, driven at 10 Hz).

        The gate is forward-looking, so holding zero velocity forever would trap
        the robot against whatever it drove into. After a short hard stop the
        robot reverses to open up the way ahead, falling back to spinning in
        place when the rear is blocked too. The cycle repeats while blocked and
        stops as soon as the front cone clears (it never reverses more than
        EMERGENCY_ESCAPE_TIME at a time).
        """
        now = self.get_clock().now()
        elapsed = (now - self._emergency_start).nanoseconds * 1e-9
        cycle = self.EMERGENCY_STOP_TIME + self.EMERGENCY_ESCAPE_TIME
        if elapsed >= cycle:
            self._emergency_start = now
            elapsed = 0.0

        twist = Twist()
        if elapsed >= self.EMERGENCY_STOP_TIME:
            if self._min_scan_range_rear >= self.REAR_CLEAR_RANGE:
                twist.linear.x = -self.EMERGENCY_REVERSE_SPEED
            else:
                twist.angular.z = self.EMERGENCY_SPIN_SPEED
        self._cmd_vel_pub.publish(twist)

    def _handle_recovery(self):
        """
        Recovery bookkeeping (non-blocking). Deliberately commands no velocity.

        The behaviour tree's MissionStuck branch (BackUp + Spin) already moves
        the robot through Nav2's behaviour server, which halts the controller
        first. If this node also published on /cmd_vel_emergency, twist_mux
        priority 100 would make it outrank the controller and the two would
        fight - which is what produced the endless circling at doorways. So all
        we do here is give Nav2 a moment, clear the costmaps and hand back.
        """
        now = self.get_clock().now()

        if self._recovery_phase == -1:
            self._recovery_phase = 0
            self._recovery_phase_start = now
            self._recovery_sent = False
            self.get_logger().info(
                f'Recovery started (attempt {self.sm.ctx.recovery_attempts + 1}); '
                'letting Nav2 own the motion')

        phase_elapsed = (now - self._recovery_phase_start).nanoseconds * 1e-9

        if self._recovery_phase == 0:
            if phase_elapsed >= self.RECOVERY_GRACE:
                if not self._recovery_sent:
                    self._recovery_sent = True
                    self._clear_costmaps()
                self._recovery_phase = 1
                self._recovery_phase_start = now
        elif self._recovery_phase == 1:
            if phase_elapsed >= self.RECOVERY_SETTLE:
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

    # ─── Frontier exploration (EXPLORE missions) ─────────────

    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _frontier_key(self, grid: OccupancyGrid, ix: int, iy: int):
        """Coarse key so a frontier that failed once is not retried immediately."""
        res = grid.info.resolution
        return (int(ix * res / 0.25), int(iy * res / 0.25))

    def _pick_frontier(self) -> Optional[PoseStamped]:
        """
        Pick the best frontier goal from the latest /map.

        A frontier is a known-free cell touching unknown space. Clusters are
        scored by size / (1 + distance) so big, nearby openings win. The goal is
        moved a little way back into known free space so it is reachable (the
        planner must not be handed a cell sitting in a wall).
        """
        grid = self._latest_map
        if grid is None or grid.info.resolution <= 0.0:
            return None
        w, h = int(grid.info.width), int(grid.info.height)
        data = grid.data
        if w <= 0 or h <= 0 or len(data) != w * h:
            return None
        res = float(grid.info.resolution)
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        now = self.get_clock().now().nanoseconds * 1e-9

        # Seeds: free cells with an unknown 8-neighbour. Scanning unknown cells
        # (usually a minority) is much cheaper than scanning the whole grid.
        seeds = set()
        for i in range(w * h):
            if data[i] >= 0:
                continue
            x = i % w
            y = i // w
            for dy in (-1, 0, 1):
                ny = y + dy
                if ny < 0 or ny >= h:
                    continue
                for dx in (-1, 0, 1):
                    nx = x + dx
                    if nx < 0 or nx >= w:
                        continue
                    j = ny * w + nx
                    if data[j] == 0:
                        seeds.add(j)

        # Cluster adjacent frontier cells.
        clusters = []
        remaining = seeds
        while remaining:
            start = remaining.pop()
            comp = [start]
            stack = [start]
            while stack:
                cur = stack.pop()
                cx = cur % w
                cy = cur // w
                for dy in (-1, 0, 1):
                    ny = cy + dy
                    if ny < 0 or ny >= h:
                        continue
                    for dx in (-1, 0, 1):
                        nx = cx + dx
                        if nx < 0 or nx >= w:
                            continue
                        j = ny * w + nx
                        if j in remaining:
                            remaining.discard(j)
                            comp.append(j)
                            stack.append(j)
            if len(comp) >= self.FRONTIER_MIN_CELLS:
                clusters.append(comp)

        self._frontier_count = len(clusters)
        if not clusters:
            return None

        robot = self._current_pose
        best = None
        best_score = -1.0
        clear_cells = max(1, int(self.FRONTIER_CLEARANCE / res))

        for comp in clusters:
            csx = sum(c % w for c in comp) / len(comp)
            csy = sum(c // w for c in comp) / len(comp)
            # Representative cell: nearest cluster cell to the centroid that
            # has FRONTIER_CLEARANCE of room to manoeuvre, i.e. no occupied
            # cells nearby. Unknown cells in the window are fine - by definition
            # a frontier sits next to them.
            cand = None
            cand_d = None
            for c in comp:
                cx = c % w
                cy = c // w
                d = (cx - csx) ** 2 + (cy - csy) ** 2
                if cand_d is not None and d >= cand_d:
                    continue
                safe = True
                for dy in range(-clear_cells, clear_cells + 1):
                    ny = cy + dy
                    if ny < 0 or ny >= h:
                        continue
                    row = ny * w
                    for dx in range(-clear_cells, clear_cells + 1):
                        nx = cx + dx
                        if nx < 0 or nx >= w:
                            continue
                        if data[row + nx] > 0:    # occupied / lethal
                            safe = False
                            break
                    if not safe:
                        break
                if safe:
                    cand, cand_d = c, d
            if cand is None:
                continue   # cluster is too tight to be a sane goal

            wx = ox + (cand % w + 0.5) * res
            wy = oy + (cand // w + 0.5) * res
            key = self._frontier_key(grid, cand % w, cand // w)
            expiry = self._frontier_blacklist.get(key)
            if expiry is not None and expiry > now:
                continue

            if robot is not None:
                rx = robot.pose.position.x
                ry = robot.pose.position.y
            else:
                rx, ry = wx, wy
            dist = math.hypot(wx - rx, wy - ry)
            if dist < self.FRONTIER_MIN_GOAL_DIST or dist > self.FRONTIER_MAX_GOAL_DIST:
                continue

            score = len(comp) / (1.0 + dist)
            if score > best_score:
                best_score = score
                best = (wx, wy, key)

        if best is None:
            return None

        wx, wy, key = best
        self._current_frontier = key

        pose = PoseStamped()
        pose.header.frame_id = grid.header.frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        if robot is not None:
            yaw = math.atan2(wy - robot.pose.position.y, wx - robot.pose.position.x)
        else:
            yaw = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _dispatch_explore_goal(self):
        """Send the next frontier goal, or finish the EXPLORE mission."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_frontier_attempt < self.FRONTIER_RETRY_PERIOD:
            return
        self._last_frontier_attempt = now

        if self._latest_map is None:
            self.get_logger().info('EXPLORE: waiting for /map ...')
            return

        goal = self._pick_frontier()
        if goal is None:
            self._frontier_empties += 1
            self.get_logger().info(
                f'EXPLORE: no frontier found ({self._frontier_empties}/'
                f'{self.FRONTIER_EMPTY_ATTEMPTS})')
            if self._frontier_empties >= self.FRONTIER_EMPTY_ATTEMPTS:
                self.get_logger().info('EXPLORE: map fully explored')
                self.sm.complete_current_mission_step()
            return

        self._frontier_empties = 0
        self._goal_in_flight = True
        self.get_logger().info(
            f'EXPLORE: heading to frontier ({goal.pose.position.x:.2f}, '
            f'{goal.pose.position.y:.2f}) of {self._frontier_count}')
        self._send_nav_goal(goal)

    def _blacklist_frontier(self):
        if self._current_frontier is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._frontier_blacklist[self._current_frontier] = (
            now + self.FRONTIER_BLACKLIST_TIME)
        self._current_frontier = None

    # ─── Nav2 action calls ───────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped):
        """Send a single-pose goal to Nav2 NavigateToPose action server."""
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('NavigateToPose action server not available')
            self._goal_in_flight = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ''

        self.get_logger().info(
            f'Sending Nav2 goal: ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f})')

        future = self._nav_client.send_goal_async(goal)
        # send_goal_async resolves when the goal is accepted, NOT when it
        # finishes; the result future has to be chained separately.
        future.add_done_callback(self._on_nav_goal_response)

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
        future.add_done_callback(self._on_wp_goal_response)

    def _on_nav_goal_response(self, future):
        """Fires when bt_navigator accepts (or rejects) the goal."""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Navigation goal request failed: {e}')
            self._goal_in_flight = False
            return

        if not handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self._goal_in_flight = False
            self._on_goal_failed('rejected')
            return

        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """Fires when the accepted goal actually finishes."""
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Navigation result failed: {e}')
            self._goal_in_flight = False
            return

        self._goal_in_flight = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal reached')
            if self.sm.mission_type == MissionType.EXPLORE:
                # Keep exploring: the next frontier comes from the updated map.
                self._current_frontier = None
            else:
                self.sm.complete_current_mission_step()
        else:
            self.get_logger().warn(f'Navigation failed: status={status}')
            self._on_goal_failed(f'status={status}')

    def _on_goal_failed(self, reason: str):
        """A goal did not complete: keep exploring, or end the mission."""
        if self.sm.mission_type == MissionType.EXPLORE and self.sm.mission_active:
            self.get_logger().warn(f'EXPLORE: frontier unreachable ({reason}); trying another')
            self._blacklist_frontier()
        elif self.sm.mission_active:
            self.get_logger().warn(f'Mission goal failed ({reason}); cancelling mission')
            self.sm.cancel_mission()

    def _on_wp_goal_response(self, future):
        """Fires when bt_navigator accepts the waypoint goal."""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Waypoint goal request failed: {e}')
            return
        if not handle.accepted:
            self.get_logger().warn('Waypoint goal rejected')
            self._on_goal_failed('rejected')
            return
        handle.get_result_async().add_done_callback(self._on_wp_result)

    def _on_wp_result(self, future):
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Waypoint result failed: {e}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint run complete')
            self.sm.complete_current_mission_step()
        else:
            self.get_logger().warn(f'Waypoint run failed: status={status}')
            self._on_goal_failed(f'status={status}')

    # ─── Status publishing (1 Hz) ────────────────────────────

    @staticmethod
    def _round_range(value: float):
        """Round a scan range for the status JSON (None when nothing seen)."""
        return round(value, 3) if math.isfinite(value) else None

    def _publish_status(self):
        summary = self.sm.get_state_summary()
        summary['min_scan_range'] = self._round_range(self._min_scan_range)
        summary['min_scan_range_all'] = self._round_range(self._min_scan_range_all)
        summary['min_scan_range_rear'] = self._round_range(self._min_scan_range_rear)
        summary['current_yaw'] = round(self._current_yaw, 3)
        summary['frontiers'] = self._frontier_count
        summary['goal_in_flight'] = self._goal_in_flight
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
