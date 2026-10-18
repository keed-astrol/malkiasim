"""
Mission State Machine for RoboDojo autonomous maze navigation.

States:
    IDLE          - Waiting for mission command
    EXPLORING     - Frontier-based map exploration
    NAVIGATING    - En route to a single goal pose
    WAYPOINTING   - Following a sequence of waypoints
    DOCKING       - Returning to home/dock
    RECOVERING    - Spin/backup recovery when stuck
    EMERGENCY     - Collision avoidance stop

Transitions are driven by sensor data (LIDAR, odom, IMU), Nav2 feedback,
and user commands via /dojo_mission topic.
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, List
from geometry_msgs.msg import PoseStamped
import math
import time


def _yaw_from_quat(q) -> float:
    """Yaw (rad) from a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest angular difference a - b, in [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MissionState(Enum):
    IDLE = 0
    EXPLORING = 1
    NAVIGATING = 2
    WAYPOINTING = 3
    DOCKING = 4
    RECOVERING = 5
    EMERGENCY = 6


class MissionType(Enum):
    NONE = 0
    EXPLORE = 1
    NAV_TO_GOAL = 2
    WAYPOINT_RUN = 3
    DOCK = 4


@dataclass
class MissionContext:
    """Shared context passed between state machine transitions."""
    mission_type: MissionType = MissionType.NONE
    current_state: MissionState = MissionState.IDLE
    target_pose: Optional[PoseStamped] = None
    waypoints: List[PoseStamped] = field(default_factory=list)
    waypoint_index: int = 0
    mission_active: bool = False
    collision_risk: bool = False
    is_stuck: bool = False
    last_movement_time: float = field(default_factory=time.time)
    last_pose: Optional[PoseStamped] = None
    last_yaw: Optional[float] = None
    stuck_threshold_sec: float = 12.0
    stuck_distance_threshold: float = 0.08   # meters moved to reset stuck
    recovery_attempts: int = 0
    max_recovery_attempts: int = 3


class MissionStateMachine:
    """
    Deterministic state machine governing autonomous maze navigation.

    Sensor integration:
      - /scan        → collision risk assessment
      - /odom        → movement tracking, stuck detection
      - /imu         → heading verification (via EKF /odometry/filtered)
      - /map         → frontier computation for exploration
      - /tf          → coordinate transforms
    """

    # "Stuck" is a last resort signal for the behaviour tree, not a knee-jerk
    # reaction: Nav2's own progress checker already aborts on real lack of
    # progress. In this arena some time not translating is normal (pivoting to
    # line up with a doorway, creeping up the ramp), so rotation counts as
    # progress and the time budget is generous.
    STUCK_THRESHOLD_SEC = 12.0
    STUCK_DIST_THRESH = 0.08
    STUCK_YAW_THRESH = 0.35           # radians of rotation that count as progress
    COLLISION_MIN_RANGE = 0.25        # meters: emergency stop
    COLLISION_WARN_RANGE = 0.50       # meters: slow down
    MAX_RECOVERY_ATTEMPTS = 3

    def __init__(self):
        self.ctx = MissionContext()

    # ─── property access ──────────────────────────────────────

    @property
    def state(self) -> MissionState:
        return self.ctx.current_state

    @property
    def mission_type(self) -> MissionType:
        return self.ctx.mission_type

    @property
    def mission_active(self) -> bool:
        return self.ctx.mission_active

    # ─── state transitions ────────────────────────────────────

    def transition(self, new_state: MissionState, reason: str = ""):
        old = self.ctx.current_state
        if old != new_state:
            self.ctx.current_state = new_state
            # Reset recovery counter when leaving recovery state
            if old == MissionState.RECOVERING and new_state != MissionState.RECOVERING:
                self.ctx.recovery_attempts = 0

    def start_mission(self, mission_type: MissionType,
                      target: Optional[PoseStamped] = None,
                      waypoints: Optional[List[PoseStamped]] = None):
        """Begin a new autonomous mission."""
        self.ctx.mission_type = mission_type
        self.ctx.mission_active = True
        self.ctx.target_pose = target
        self.ctx.waypoints = waypoints or []
        self.ctx.waypoint_index = 0
        self.ctx.recovery_attempts = 0
        self.ctx.is_stuck = False

        state_map = {
            MissionType.EXPLORE: MissionState.EXPLORING,
            MissionType.NAV_TO_GOAL: MissionState.NAVIGATING,
            MissionType.WAYPOINT_RUN: MissionState.WAYPOINTING,
            MissionType.DOCK: MissionState.DOCKING,
        }
        self.transition(state_map.get(mission_type, MissionState.IDLE),
                        f"Mission started: {mission_type.name}")

    def cancel_mission(self):
        """Cancel current mission, return to IDLE."""
        self.ctx.mission_active = False
        self.ctx.mission_type = MissionType.NONE
        self.ctx.target_pose = None
        self.ctx.waypoints.clear()
        self.ctx.waypoint_index = 0
        self.ctx.recovery_attempts = 0
        self.transition(MissionState.IDLE, "Mission cancelled")

    # ─── sensor-driven transitions ────────────────────────────

    def evaluate_collision_risk(self, min_scan_range: float) -> bool:
        """
        Check if an obstacle is within COLLISION_MIN_RANGE.
        Returns True if emergency stop is needed.
        """
        at_risk = min_scan_range < self.COLLISION_MIN_RANGE
        if at_risk and self.ctx.current_state != MissionState.EMERGENCY:
            self.transition(MissionState.EMERGENCY, f"Collision risk: {min_scan_range:.2f}m")
        elif not at_risk and self.ctx.current_state == MissionState.EMERGENCY:
            # Return to previous mission if still active
            if self.ctx.mission_active:
                self._resume_mission_state()
            else:
                self.transition(MissionState.IDLE, "Collision cleared, no mission")
        return at_risk

    def evaluate_stuck(self, current_pose: PoseStamped) -> bool:
        """
        Detect if the robot has stopped making progress.

        Movement is measured as translation OR rotation. Rotation matters: a
        diff drive pivoting to line up with a doorway produces no translation
        at all, and counting that as "stuck" made the mission node fight the
        controller (see autonomous_mission_node._handle_recovery).
        """
        now = time.time()
        yaw = _yaw_from_quat(current_pose.pose.orientation)

        if self.ctx.last_pose is None or self.ctx.last_yaw is None:
            self.ctx.last_pose = current_pose
            self.ctx.last_yaw = yaw
            self.ctx.last_movement_time = now
            return False

        dx = current_pose.pose.position.x - self.ctx.last_pose.pose.position.x
        dy = current_pose.pose.position.y - self.ctx.last_pose.pose.position.y
        dist = (dx * dx + dy * dy) ** 0.5
        dyaw = abs(_angle_diff(yaw, self.ctx.last_yaw))

        if dist >= self.STUCK_DIST_THRESH or dyaw >= self.STUCK_YAW_THRESH:
            # Robot moved (translation or rotation), reset timer
            self.ctx.last_pose = current_pose
            self.ctx.last_yaw = yaw
            self.ctx.last_movement_time = now
            self.ctx.is_stuck = False
            return False

        elapsed = now - self.ctx.last_movement_time
        stuck = elapsed > self.STUCK_THRESHOLD_SEC
        self.ctx.is_stuck = stuck

        if stuck and self.ctx.current_state not in (MissionState.RECOVERING,
                                                     MissionState.EMERGENCY,
                                                     MissionState.IDLE):
            self.transition(MissionState.RECOVERING, f"Stuck for {elapsed:.1f}s")
        return stuck

    def complete_current_mission_step(self):
        """Called when Nav2 reports success for the current goal."""
        if self.ctx.mission_type == MissionType.WAYPOINT_RUN:
            self.ctx.waypoint_index += 1
            if self.ctx.waypoint_index >= len(self.ctx.waypoints):
                self.transition(MissionState.IDLE, "All waypoints visited")
                self.ctx.mission_active = False
        else:
            self.transition(MissionState.IDLE, "Mission complete")
            self.ctx.mission_active = False

    def recovery_complete(self):
        """Called after a recovery behavior finishes."""
        self.ctx.recovery_attempts += 1
        if self.ctx.recovery_attempts >= self.MAX_RECOVERY_ATTEMPTS:
            self.cancel_mission()
        elif self.ctx.mission_active:
            self._resume_mission_state()

    # ─── internal helpers ─────────────────────────────────────

    def _resume_mission_state(self):
        state_map = {
            MissionType.EXPLORE: MissionState.EXPLORING,
            MissionType.NAV_TO_GOAL: MissionState.NAVIGATING,
            MissionType.WAYPOINT_RUN: MissionState.WAYPOINTING,
            MissionType.DOCK: MissionState.DOCKING,
        }
        target = state_map.get(self.ctx.mission_type, MissionState.IDLE)
        self.transition(target, f"Resuming {self.ctx.mission_type.name}")

    def get_state_summary(self) -> dict:
        return {
            'state': self.ctx.current_state.name,
            'mission_type': self.ctx.mission_type.name,
            'mission_active': self.ctx.mission_active,
            'collision_risk': self.ctx.collision_risk,
            'is_stuck': self.ctx.is_stuck,
            'recovery_attempts': self.ctx.recovery_attempts,
            'waypoint_index': self.ctx.waypoint_index,
            'total_waypoints': len(self.ctx.waypoints),
        }
