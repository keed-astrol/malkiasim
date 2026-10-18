"""
Launch RoboDojo autonomous maze navigation with custom Behavior Tree.

This extends the existing navigate.launch.py with:
  - Custom BT XML loaded into nav2_bt_navigator
  - dojo_bt_plugins C++ shared library for custom BT nodes
  - autonomous_mission_node for the state machine runtime
  - Additional parameters for collision avoidance and exploration
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue


def _default_map() -> Path:
    """Locate maps/arena.yaml by walking up from this file (works from
    src/ and from install/) instead of hardcoding a user-specific path."""
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'maps' / 'arena.yaml'
        if candidate.is_file():
            return candidate
    raise FileNotFoundError('maps/arena.yaml not found above ' + str(Path(__file__).resolve()))


def generate_launch_description():
    pkg_robodojo = Path(get_package_share_directory('robodojo'))
    pkg_gamefield = Path(get_package_share_directory('gamefield'))
    pkg_dojo_nav = Path(get_package_share_directory('dojo_nav'))
    pkg_dojo_bt = Path(get_package_share_directory('dojo_bt'))
    pkg_gz_sim = Path(get_package_share_directory('ros_gz_sim'))

    world = pkg_gamefield / 'worlds' / 'gamefield.world'
    robot_desc = pkg_robodojo / 'urdf' / 'robodojo.urdf'
    ekf_cfg = pkg_robodojo / 'config' / 'ekf.yaml'
    nav2_cfg = pkg_dojo_nav / 'config' / 'nav2_params.yaml'
    slam_cfg = pkg_robodojo / 'config' / 'slam.yaml'
    twist_mux_cfg = pkg_dojo_nav / 'config' / 'twist_mux.yaml'
    bt_cfg = pkg_dojo_bt / 'config' / 'dojo_bt_params.yaml'
    rviz_cfg = pkg_dojo_nav / 'config' / 'dojo_nav.rviz'
    default_map = _default_map()

    # ─── Gazebo + robot bringup ─────────────────────────────────
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
            '-x', '-2.50',
            '-y', '-1.10',
            '-z', '0.05',
            '-Y', '0',
        ],
        output='screen',
    )

    # ─── EKF ────────────────────────────────────────────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_cfg, {'use_sim_time': True}],
        output='screen',
    )

    # ─── Nav2 localization and navigation ───────────────────────
    # localization:=amcl  map_server + AMCL against a prebuilt map (default)
    # localization:=slam  slam_toolbox builds the map live, no prior map needed
    use_amcl = PythonExpression(
        ["'", LaunchConfiguration('localization'), "' == 'amcl'"])
    use_slam = PythonExpression(
        ["'", LaunchConfiguration('localization'), "' == 'slam'"])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory('nav2_bringup'))
                / 'launch'
                / 'localization_launch.py'
            )
        ),
        condition=IfCondition(use_amcl),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': str(nav2_cfg),
            'autostart': 'true',
        }.items(),
    )

    slam_localization = [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_cfg, {'use_sim_time': True}],
            output='screen',
            condition=IfCondition(use_slam),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            condition=IfCondition(use_slam),
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
        ),
    ]

    nav_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Merge nav2 params with our BT params.
    # ParameterFile(allow_substs=True) is required for the
    # "$(find-pkg-share dojo_bt)/..." path in dojo_bt_params.yaml to be
    # expanded — plain string paths skip substitution and would be passed
    # through literally.
    nav_params = [
        ParameterFile(str(nav2_cfg), allow_substs=True),
        ParameterFile(str(bt_cfg), allow_substs=True),
        {'use_sim_time': True},
    ]

    nav_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
            ],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            output='screen',
            parameters=nav_params,
            remappings=nav_remaps,
        ),
        # Final velocity arbitration: cmd_vel_safe (navigation) vs
        # /cmd_vel_emergency (mission e-stop); publishes the winner on cmd_vel
        # for the Gazebo diff-drive plugin.
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_cfg, {'use_sim_time': True}],
            remappings=[('cmd_vel_out', 'cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }],
        ),
    ]

    # ─── Custom BT: autonomous mission node ─────────────────────
    autonomous_mission = Node(
        package='dojo_bt',
        executable='autonomous_mission_node',
        name='autonomous_mission_node',
        output='screen',
        parameters=[
            bt_cfg,
            {'use_sim_time': True},
            # slam mode: start EXPLORE on its own (no /dojo_mission needed).
            {'localization_mode': LaunchConfiguration('localization')},
            {'auto_explore': ParameterValue(
                LaunchConfiguration('auto_explore'), value_type=bool)},
        ],
    )

    # ─── RViz with nav config ───────────────────────────────────
    # Point the Gazebo GUI camera at the rover, the same as right-clicking the
    # model and choosing Follow. /gui/follow is a gz-transport service, so it
    # can only be called once the GUI exists (12 s is well after spawn).
    follow_camera = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/gui/follow',
                '--reqtype', 'gz.msgs.StringMsg',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '4000',
                '--req', 'data: "robodojo"',
            ],
            output='screen',
        )],
    )

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
            description='Full path to the Nav2 occupancy-grid YAML map '
                        '(used when localization:=amcl).',
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='amcl',
            choices=['amcl', 'slam'],
            description='amcl: map_server + AMCL against the map argument. '
                        'slam: slam_toolbox builds the map live (no map needed, '
                        'use this for EXPLORE on an unknown arena).',
        ),
        DeclareLaunchArgument(
            'auto_explore',
            default_value='true',
            choices=['true', 'false'],
            description='Start an EXPLORE mission by itself once the map is '
                        'usable. Only takes effect with localization:=slam; '
                        'set false to drive to goals by hand (RViz 2D Goal Pose).',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=f'{pkg_robodojo.parent}:{pkg_gamefield / "models"}',
        ),
        # Pin gz-transport discovery to loopback: multicast peer discovery is
        # unreliable on some hosts (Wi-Fi w/o multicast loopback), which leaves
        # the GUI unable to find the server (black screen / "no subscriber to
        # /gazebo/starting_world").
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1'),
        gz_sim,
        robot_state_pub,
        gz_bridge,
        spawn_robot,
        ekf,
        # Start localization, navigation, and autonomous mission after simulator TF exists
        # 10 s: at real_time_factor 0.6 the sim clock advances slower than wall
        # time, and starting the Nav2 lifecycle bringup while Gazebo/SLAM are
        # still settling can leave planner_server stuck in `configure` (the
        # manager then blocks and bt_navigator never activates, so every goal is
        # silently swallowed).
        TimerAction(
            period=10.0,
            actions=[localization, *slam_localization, *nav_nodes, autonomous_mission],
        ),
        follow_camera,
        rviz,
    ])

    return ld
