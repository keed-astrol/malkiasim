#/home/keedastro/dojourdf/src/robodojo/launch/slam.launch.py
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
    slam_config = package_share / 'config' / 'slam.yaml'
    ekf_config = package_share / 'config' / 'ekf.yaml'

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=f'{package_share.parent}:{gamefield_share / "models"}',
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
            parameters=[{
                'robot_description': robot_description.read_text(),
                'use_sim_time': True,
            }],
        ),
        # joint_state_publisher_custom removed: real joint states come from Gazebo
        Node(
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
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config, {'use_sim_time': True}],
            output='screen',
        ),
        
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
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
        ),
         Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            parameters=[
                {
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                },
            ],
            output='screen',
        ),
        # --- SLAM Toolbox ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config, {'use_sim_time': True}],
            output='screen',
        ),
        
    ])