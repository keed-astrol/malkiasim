from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('robodojo'))
    robot_description = package_share / 'urdf' / 'robodojo.urdf'
    rviz_config = package_share / 'config' / 'robodojo.rviz'

    return LaunchDescription([
        Node(
            package='robodojo',
            executable='joint_state_publisher_custom',
            name='joint_state_publisher_custom',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description.read_text()}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
        ),
    ])