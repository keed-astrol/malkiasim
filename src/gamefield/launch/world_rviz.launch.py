from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('gamefield'))
    ros_gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))
    world = package_share / 'worlds' / 'gamefield.world'
    rviz_config = package_share / 'config' / 'gamefield.rviz'

    return LaunchDescription([
        # Pin gz-transport discovery to loopback: multicast peer discovery is
        # unreliable on some hosts (Wi-Fi w/o multicast loopback), which leaves
        # the GUI unable to find the server (black screen).
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1'),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=str(package_share / 'models'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_sim_share / 'launch' / 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world}'}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_world_inspection',
            arguments=['-d', str(rviz_config)],
            output='screen',
        ),
    ])