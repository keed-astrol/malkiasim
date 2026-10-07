"""Launch RoboDojo autonomous navigation using a saved map./home/keedastro/dojourdf/src/dojo_nav/launch/navigate.launch.py"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
        DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robodojo = Path(get_package_share_directory('robodojo'))
    pkg_gamefield = Path(get_package_share_directory('gamefield'))
    pkg_dojo_nav = Path(get_package_share_directory('dojo_nav'))
    pkg_gz_sim = Path(get_package_share_directory('ros_gz_sim'))

    world = pkg_gamefield / 'worlds' / 'gamefield.world'
    robot_desc = pkg_robodojo / 'urdf' / 'robodojo.urdf'
    ekf_cfg = pkg_robodojo / 'config' / 'ekf.yaml'
    nav2_cfg = pkg_dojo_nav / 'config' / 'nav2_params.yaml'
    rviz_cfg = pkg_dojo_nav / 'config' / 'dojo_nav.rviz'
    default_map = Path('/home/keedastro/dojourdf/maps/arena.yaml')

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
            '-y', '-1.2',
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

    # ─── Nav2 localization and navigation ───
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory('nav2_bringup'))
                / 'launch'
                / 'localization_launch.py'
            )
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': str(nav2_cfg),
            'autostart': 'true',
        }.items(),
    )

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

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_dojo_nav',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'route_server',
                'behavior_server',
                'velocity_smoother',
                'collision_monitor',
                'bt_navigator',
                'waypoint_follower',
                'docking_server',
            ],
        }],
        output='screen',
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
        DeclareLaunchArgument(
            'map',
            default_value=str(default_map),
            description='Full path to the Nav2 occupancy-grid YAML map.',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=f'{pkg_robodojo.parent}:{pkg_gamefield / "models"}',
        ),
        gz_sim,
        robot_state_pub,
        gz_bridge,
        spawn_robot,
        ekf,
        # Start localization and navigation after the simulator and robot TF exist.
        TimerAction(
            period=3.0,
            actions=[localization, nav2_bringup, lifecycle_manager],
        ),
        rviz,
    ])

    return ld
