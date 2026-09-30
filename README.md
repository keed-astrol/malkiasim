# RoboDojo ROS 2 Simulation

ROS 2 workspace for the RoboDojo robot, including its URDF, wheel and lidar models, Gazebo gamefield, RViz configuration, and SLAM launch setup.

## Packages

- `robodojo`: robot URDF, meshes, joint-state publisher, RViz configuration, Gazebo bridge, and SLAM launch files.
- `gamefield`: Gazebo world and arena model.

## Requirements

Install and source a ROS 2 environment with these packages available:

- Gazebo Sim and `ros_gz_sim`
- `ros_gz_bridge`
- `robot_state_publisher`
- `rviz2`
- `slam_toolbox`
- `rmw_zenoh_cpp`

The commands below assume the workspace is located at `~/dojourdf`. Adjust the path if needed.

## Build

From the workspace root:

```bash
cd ~/dojourdf
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` again in each new terminal before using the workspace.

## Launch Gazebo and RViz

```bash
ros2 launch robodojo gz_sim.launch.py
```

This starts the gamefield, robot state publisher, custom joint-state publisher, Gazebo to ROS bridge, and RViz. The robot is spawned near the back-left start area at approximately:

- `x = -2.7`
- `y = 1.2`
- `z = 0.6`
- yaw `= 0`

The exact meaning of back-left depends on the world coordinate orientation.

## Launch SLAM

```bash
ros2 launch robodojo slam.launch.py
```

This starts the Gazebo simulation and SLAM Toolbox using `src/robodojo/config/slam.yaml`. Drive the robot with `/cmd_vel`; lidar, odometry, transforms, and the map are available through the configured ROS topics.

## Useful topics

- `/cmd_vel`: robot velocity commands
- `/scan`: lidar scan
- `/odom`: wheel odometry
- `/tf`: transforms
- `/map`: SLAM map
- `/joint_states`: wheel and lidar joint states

Example command to stop the robot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Robot description and RViz

The robot description is stored in:

```text
src/robodojo/urdf/robodojo.urdf
```

The RViz configuration is stored in:

```text
src/robodojo/config/robodojo.rviz
```

The custom wheel joint publisher is implemented in:

```text
src/robodojo/robodojo/joint_state_publisher_node.py
```

## Troubleshooting

### Gazebo cannot find `model://gamefield_arena`

Rebuild and source the workspace so the installed model directory is available:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robodojo gz_sim.launch.py
```

The launch file sets `GZ_SIM_RESOURCE_PATH` to the workspace package and gamefield model locations. If starting Gazebo manually, export the resource path first:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/install/gamefield/share/gamefield/models:$PWD/install/robodojo/share/robodojo"
```

### Zenoh router warning

The launch files start a Zenoh router with `rmw_zenohd`. If starting nodes manually, start the router first:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Clean generated build files

Build output is ignored by Git. To rebuild from a clean workspace:

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```
