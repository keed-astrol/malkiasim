#!/usr/bin/env python3
"""
Level-scan gate for 2D SLAM.

The lidar is a single horizontal plane mounted on the chassis. When the rover
pitches on an arena ramp, that plane tilts with it: the beams sweep into the
floor ahead (ranges collapse) and over the far wall behind (ranges vanish). A
2D scan matcher cannot reconcile that with a planar pose - the map smears.

This node forwards /scan to /scan_level only while the body is within
`pitch_limit` of level, using the IMU orientation. On a ramp the mapper simply
sees no scans for the second or two the rover is tilted; EKF odometry carries
the pose, and mapping resumes on the flat. Nav2 keeps consuming the raw /scan,
so the collision monitor and obstacle layer are unaffected.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def pitch_from_quaternion(q) -> float:
    """Pitch (radians) from a geometry_msgs Quaternion, ZYX convention."""
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


class LevelScanGate(Node):

    def __init__(self):
        super().__init__('level_scan_gate')

        self.declare_parameter('pitch_limit', 2.5)      # degrees
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('output_topic', '/scan_level')

        self._pitch_limit = math.radians(
            float(self.get_parameter('pitch_limit').value))
        scan_topic = self.get_parameter('scan_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        out_topic = self.get_parameter('output_topic').value

        self._pitch = None
        self._level = False
        self._passed = 0
        self._dropped = 0

        self._pub = self.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self._on_scan,
                                 qos_profile_sensor_data)

        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'Level-scan gate: {scan_topic} -> {out_topic}, '
            f'pitch limit {math.degrees(self._pitch_limit):.1f} deg')

    # ── callbacks ────────────────────────────────────────────

    def _on_imu(self, msg: Imu):
        self._pitch = pitch_from_quaternion(msg.orientation)
        was_level = self._level
        self._level = abs(self._pitch) <= self._pitch_limit
        if was_level and not self._level:
            self.get_logger().info(
                f'Tilted {math.degrees(self._pitch):.1f} deg - '
                'pausing SLAM scans until level')
        elif not was_level and self._level and self._pitch is not None:
            self.get_logger().info('Level again - resuming SLAM scans')

    def _on_scan(self, msg: LaserScan):
        if self._level:
            self._passed += 1
            self._pub.publish(msg)
        else:
            self._dropped += 1

    def _report(self):
        if self._dropped:
            self.get_logger().info(
                f'level scans passed {self._passed}, dropped {self._dropped} '
                f'(pitch {math.degrees(self._pitch or 0.0):.1f} deg)')


def main(args=None):
    rclpy.init(args=args)
    node = LevelScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
