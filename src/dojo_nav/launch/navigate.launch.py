"""
Launch RoboDojo autonomous navigation.

Brings up:
  - Gazebo + robot spawn (same as slam.launch.py)
  - SLAM Toolbox (mapping -> saves map)
  - Nav2 stack: AMCL, planner, controller, costmaps, BT navigator
  - RViz with nav visualization config

Usage:
  ros2 launch dojo_nav navigate.launch.py

The slam_toolbox map is used live. Once you have a good map from
mapping mode, you can save it and switch to localization mode.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robodojo = Path(get_package_share_directory('robodojo'))
    pkg_gamefield = Path(get_package_share_directory('gamefield'))
    pkg_dojo_nav = Path(get_package_share_directory('dojo_nav'))
    pkg_gz_sim = Path(get_package_share_directory('ros_gz_sim'))

    world = pkg_gamefield / 'worlds' / 'gamefield.world'
    robot_desc = pkg_robodojo / 'urdf' / 'robodojo.urdf'
    slam_cfg = pkg_robodojo / 'config' / 'slam.yaml'
    ekf_cfg = pkg_robodojo / 'config' / 'ekf.yaml'
    nav2_cfg = pkg_dojo_nav / 'config' / 'nav2_params.yaml'
    rviz_cfg = pkg_dojo_nav / 'config' / 'dojo_nav.rviz'

    # ─── Gazebo + robot bringup (same as slam.launch.py) ───
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_gz_sim / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc.read_text(),
            'use_sim_time': True,
        }],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robodojo',
            '-topic', 'robot_description',
            '-x', '-2.7',
            '-y', '1.2',
            '-z', '0.6',
            '-Y', '0',
        ],
        output='screen',
    )

    # ─── EKF ───
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_cfg, {'use_sim_time': True}],
        output='screen',
    )

    # ─── SLAM Toolbox ───
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
        output='screen',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_cfg, {'use_sim_time': True}],
        output='screen',
    )

    # ─── Nav2 stack ───
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory('nav2_bringup'))
                / 'launch'
                / 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': str(nav2_cfg),
            'autostart': 'true',
        }.items(),
    )

    # ─── RViz with nav config ───
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg)],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ld = LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=f'{pkg_robodojo.parent}:{pkg_gamefield / "models"}',
        ),
        gz_sim,
        robot_state_pub,
        gz_bridge,
        spawn_robot,
        ekf,
        slam_lifecycle,
        slam_toolbox,
        # Start Nav2 after a short delay so SLAM map is available
        TimerAction(
            period=3.0,
            actions=[nav2_bringup],
        ),
        rviz,
    ])

    return ld
