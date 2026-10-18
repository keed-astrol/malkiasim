import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('robodojo'))
    gamefield_share = Path(get_package_share_directory('gamefield'))
    gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))
    robot_description = package_share / 'urdf' / 'robodojo.urdf'
    world = gamefield_share / 'worlds' / 'gamefield.world'
    rviz_config = package_share / 'config' / 'robodojo.rviz'

    return LaunchDescription([
        # gz-transport peer discovery via UDP multicast is unreliable on some
        # hosts (e.g. Wi-Fi interfaces without multicast loopback), which leaves
        # the GUI unable to find the server ("no subscriber to
        # /gazebo/starting_world", black screen). GUI + server are always local,
        # so pin discovery to loopback instead.
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1'),
        # Append to any pre-existing value instead of clobbering it, and include
        # both the share dirs (for package:// URIs) and the gamefield models
        # dir (for model://gamefield_arena).
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.pathsep.join(
                p
                for p in (
                    os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                    str(package_share.parent),
                    str(gamefield_share / 'models'),
                )
                if p
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(gz_sim_share / 'launch' / 'gz_sim.launch.py')
            ),
                launch_arguments={'gz_args': f'-r {world}'}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description.read_text()},
                {'use_sim_time': True},
            ],
            output='screen',
        ),
        Node(
            package='robodojo',
            executable='joint_state_publisher_custom',
            name='joint_state_publisher_custom',
            parameters=[
                {'use_sim_time': True},
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
            parameters=[
                {'use_sim_time': True},
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'robodojo',
                '-topic', 'robot_description',
                '-x', '-2.50',
                '-y', '-1.10',
                '-z', '0.6',
                '-Y', '0',
            ],
            output='screen',
        ),
    ])