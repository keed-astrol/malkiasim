#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisherNode(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_custom')
        
        # Create publisher for joint states
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Create timer to publish joint states at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)
        
        self.get_logger().info('Custom Joint State Publisher started')
        self.get_logger().info('Publishing fixed URDF joint positions')
        
    def publish_joint_states(self):
        try:
            # Create joint state message
            msg = JointState()
            
            # Get current timestamp
            current_time = self.get_clock().now()
            msg.header.stamp.sec = int(current_time.nanoseconds // 1_000_000_000)
            msg.header.stamp.nanosec = int(current_time.nanoseconds % 1_000_000_000)
            msg.header.frame_id = 'base_link'
            
            # Define all joint names from URDF - MUST match URDF exactly
            msg.name = [
                'front_left_wheel_joint',
                'front_right_wheel_joint',
                'back_left_wheel_joint',
                'back_right_wheel_joint',
                'lidar_joint'
            ]
            
            # Keep every joint at the position defined by the URDF.
            msg.position = [
                0.0,  # front_left_wheel_joint
                0.0,  # front_right_wheel_joint
                0.0,  # back_left_wheel_joint
                0.0,  # back_right_wheel_joint
                0.0,  # lidar_joint
            ]

            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing joint states: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
