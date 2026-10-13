# RoboDojo ROS 2 Simulation

ROS 2 workspace for the RoboDojo robot, including its URDF, wheel and lidar models, Gazebo gamefield, RViz configuration, SLAM, and autonomous behavior-tree maze navigation.

## Packages

- `robodojo`: robot URDF, meshes, joint-state publisher, RViz configuration, Gazebo bridge, and SLAM launch files.
- `gamefield`: Gazebo world and arena model.
- `dojo_nav`: Nav2 configuration and navigation launch files.
- `dojo_bt`: Custom Behavior Tree nodes (C++ plugins), autonomous mission state machine (Python), and full autonomous maze launch.

## Requirements

Install and source a ROS 2 environment with these packages available:

- Gazebo Sim and `ros_gz_sim`
- `ros_gz_bridge`
- `robot_state_publisher`
- `rviz2`
- `slam_toolbox`
- `nav2_bringup` and Nav2 stack (`nav2_bt_navigator`, `nav2_controller`, etc.)
- `robot_localization` (EKF)
- `behaviortree_cpp`
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

This starts Gazebo, the robot localization EKF, and SLAM Toolbox using `src/robodojo/config/slam.yaml`. Drive the robot with `/cmd_vel`; SLAM Toolbox uses `/scan` and the `odom` to `base_link` transform to build `/map`.

The Gazebo world must load the `gz-sim-imu-system` plugin for the robot IMU sensor to produce samples. The SLAM launch bridges the simulator IMU to ROS as `/imu` and starts the EKF, which publishes filtered odometry on `/odometry/filtered`.

Verify the sensor and SLAM inputs in another sourced terminal:

```bash
ros2 topic hz /imu
ros2 topic hz /scan
ros2 topic echo /odometry/filtered --once
ros2 topic echo /map --once
```

If `/imu` exists but `ros2 topic hz /imu` receives no messages, restart the simulation after rebuilding so Gazebo loads the current world file:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robodojo slam.launch.py
```

## Autonomous Behavior Tree Maze Navigation (`dojo_bt`)

The `dojo_bt` package provides custom Behavior Tree C++ plugins for Nav2, a Python mission state machine, and an all-in-one launch file for fully autonomous maze exploration.

### 1. Build

```bash
cd ~/dojourdf
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to dojo_bt
source install/setup.bash
```

### 2. Create a map

The autonomous launch requires a pre-built Nav2 occupancy-grid map. Generate one by running SLAM and driving the robot through the maze, then saving:

```bash
ros2 launch robodojo slam.launch.py
```

In another sourced terminal, drive the robot and save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/dojourdf/maps/arena
```

### 3. Launch autonomous maze navigation

```bash
ros2 launch dojo_bt autonomous_maze.launch.py
```

This single launch brings up:
- Gazebo with the gamefield world and `robodojo` robot
- EKF (`robot_localization`) fusing odometry + IMU
- Nav2 stack (localization, controller, planner, BT navigator, behaviors, collision monitor, lifecycle manager)
- Custom BT plugins (`dojo_bt_plugins.so`) loaded into `nav2_bt_navigator`
- Autonomous mission node (Python state machine)

### 4. Send a mission command

Publish a JSON mission to `/dojo_mission`:

```bash
ros2 topic pub /dojo_mission std_msgs/msg/String 'data: "{\"mission\": \"explore\", \"goal\": {\"x\": 3.0, \"y\": 0.0, \"qz\": 0.0, \"qw\": 1.0}}"'
```

### 5. Monitor status

```bash
ros2 topic echo /dojo_mission/status
ros2 topic echo /dojo_mission/state
```

### Key files

| File | Purpose |
|---|---|
| `src/dojo_bt/launch/autonomous_maze.launch.py` | Top-level launch file |
| `src/dojo_bt/config/dojo_bt_params.yaml` | BT and mission parameters |
| `src/dojo_bt/src/bt_actions.cpp` | C++ custom BT action nodes |
| `src/dojo_bt/src/bt_conditions.cpp` | C++ custom BT condition nodes |
| `src/dojo_bt/dojo_bt/autonomous_mission_node.py` | Python mission state machine (ROS2 node) |
| `src/dojo_bt/dojo_bt/mission_state_machine.py` | State machine logic |
| `src/dojo_bt/behavior_trees/maze/autonomous_maze.xml` | BT XML loaded by Nav2 BT navigator |

## Useful topics

- `/cmd_vel`: robot velocity commands
- `/scan`: lidar scan
- `/odom`: wheel odometry
- `/imu`: IMU data
- `/odometry/filtered`: EKF fused pose
- `/tf`: transforms
- `/map`: SLAM map
- `/dojo_mission`: mission commands (JSON)
- `/dojo_mission/state`: mission state machine status (JSON)
- `/dojo_mission/status`: human-readable status messages
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
