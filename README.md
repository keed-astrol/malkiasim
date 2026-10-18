# RoboDojo ROS 2 Simulation

A ROS 2 workspace for the **RoboDojo** robot: URDF/meshes, a Gazebo gamefield,
SLAM, and a Nav2 navigation stack with a custom Behavior Tree for autonomous
maze navigation.

Target platform: **ROS 2 Jazzy** + **Gazebo Harmonic** (`ros_gz`), on Ubuntu 24.04.

If you run ROS 2 inside a container (e.g. `distrobox`), run every `ros2`/`gz`
command **inside that container**, not on the host.

---

## Packages

| Package | Purpose |
|---|---|
| `robodojo` | Robot URDF + meshes, custom joint-state publisher, RViz config, Gazebo/SLAM launch files |
| `gamefield` | Gazebo world (`gamefield.world`, `auto_arena.world`) and the arena model/meshes |
| `dojo_nav` | Nav2 configuration (`nav2_params.yaml`, `twist_mux.yaml`) and `navigate.launch.py` |
| `dojo_bt` | Custom Behavior Tree C++ plugin, Python mission state machine, custom BT XML, and the all-in-one `autonomous_maze.launch.py` |

---

## Requirements

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-twist-mux \
  ros-jazzy-robot-localization \
  ros-jazzy-behaviortree-cpp \
  libjsoncpp-dev \
  python3-colcon-common-extensions
```

`nav2_bringup` pulls in the Nav2 stack (`nav2_bt_navigator`, `nav2_controller`,
`nav2_planner`, `nav2_behaviors`, `nav2_collision_monitor`, …). `twist_mux` is
required because the velocity chain terminates in a mux (see
[Velocity command chain](#velocity-command-chain)).

## Build

From the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in every new terminal before launching anything.

---

## Launch files

| Command | What it does |
|---|---|
| `ros2 launch robodojo display.launch.py` | Robot model + joint states in RViz (no Gazebo) |
| `ros2 launch robodojo gz_sim.launch.py` | Gazebo gamefield + robot + bridge + RViz |
| `ros2 launch robodojo slam.launch.py` | Gazebo + EKF + SLAM Toolbox (builds `/map`) |
| `ros2 launch dojo_nav navigate.launch.py` | Gazebo + EKF + full Nav2 stack + RViz, using a saved map |
| `ros2 launch dojo_bt autonomous_maze.launch.py` | Everything above **plus** the custom BT and the autonomous mission node |
| `ros2 launch gamefield world_rviz.launch.py` | World inspection in RViz |

The robot spawns at approximately `x = -2.7`, `y = -1.2`, `z = 0.6`, yaw `0`.

### Gazebo GUI shows a black screen / the world dies instantly

`gz-transport` finds peers over UDP multicast. On hosts whose interfaces do not
loop multicast back (common on Wi-Fi), the GUI can never discover the server, so
it hangs in `createGui` and kills the server. The symptom in the log is:

```text
no subscriber to /gazebo/starting_world
```

**Every launch file in this repository that starts Gazebo already pins discovery
to loopback**, so no shell configuration is needed:

```python
SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),   # gz-transport
SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1'),  # legacy name
```

If you start `gz sim` **by hand** outside these launch files, export the same
variables first (or add them to your shell rc yourself — the repository does not
modify your shell configuration):

```bash
export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1
```

Other useful fallbacks for headless/VMs: `LIBGL_ALWAYS_SOFTWARE=1` for software
rendering and `QT_QPA_PLATFORM=xcb` if the GUI misbehaves under Wayland.

---

## SLAM

```bash
ros2 launch robodojo slam.launch.py
```

This starts Gazebo, the `robot_localization` EKF (`/odometry/filtered`) and SLAM
Toolbox, which builds `/map` from `/scan`. Verify the inputs:

```bash
ros2 topic hz /scan
ros2 topic hz /imu
ros2 topic echo /odometry/filtered --once
```

Save a map once the arena has been driven through:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/arena
```

The default map used by the navigation launches is `maps/arena.yaml`. Both
`navigate.launch.py` and `autonomous_maze.launch.py` locate it automatically, or
you can override it:

```bash
ros2 launch dojo_bt autonomous_maze.launch.py map:=/absolute/path/to/my_map.yaml
```

---

## Autonomous maze navigation

```bash
ros2 launch dojo_bt autonomous_maze.launch.py
```

This brings up Gazebo + EKF + Nav2 (localization, controller, planner, smoother,
behaviors, collision monitor, BT navigator, waypoint follower, lifecycle
manager) + `twist_mux` + the custom Behavior Tree + the autonomous mission node.

### Sending a mission

Missions are JSON on `/dojo_mission` (a `std_msgs/String`). Supported actions:

| Action | Payload | Meaning |
|---|---|---|
| `EXPLORE` | – | Frontier-based exploration |
| `GOAL` | `pose: {x, y [, z, qz, qw, frame_id]}` | Navigate to one pose |
| `WAYPOINTS` | `poses: [{x, y, …}, …]` | Follow a sequence of poses |
| `DOCK` | `dock_pose: {x, y, …}` | Return to the dock pose |
| `CANCEL` | – | Cancel the current mission |
| `STATUS` | – | Re-publish the current state summary |

```bash
# Navigate to a single pose (map frame; qz/qw give the yaw quaternion)
ros2 topic pub --once /dojo_mission std_msgs/msg/String \
  'data: "{\"action\": \"GOAL\", \"pose\": {\"x\": 1.5, \"y\": 0.0, \"qz\": 0.0, \"qw\": 1.0}}"'

# Explore autonomously
ros2 topic pub --once /dojo_mission std_msgs/msg/String \
  'data: "{\"action\": \"EXPLORE\"}"'

# Cancel
ros2 topic pub --once /dojo_mission std_msgs/msg/String \
  'data: "{\"action\": \"CANCEL\"}"'
```

### Monitoring

```bash
ros2 topic echo /dojo_mission/status   # human-readable
ros2 topic echo /dojo_mission/state    # JSON state machine snapshot
```

`/dojo_mission/state` is published at 1 Hz and contains `state`, `mission_type`,
`mission_active`, `is_stuck`, `min_scan_range`, `pose_x`, `pose_y`, … The
Behavior Tree reads this topic for its collision/stuck decisions.

---

## Velocity command chain

All velocity stages use **plain `geometry_msgs/Twist`** (every node sets
`enable_stamped_cmd_vel: false`, matching the Gazebo bridge):

```text
controller_server ──► cmd_vel_nav
        velocity_smoother ──► cmd_vel_smoothed
        collision_monitor ──► cmd_vel_safe          ┐
                                                     ├─► twist_mux ──► cmd_vel ──► Gazebo
/cmd_vel_emergency (mission node)                   ┘   (priority 100)
                                                         navigation priority 10
```

`twist_mux` arbitrates so an emergency command always beats navigation commands.
`/cmd_vel_emergency` has a **single writer**, `autonomous_mission_node`:

* zero velocity during the hard-stop phase of a collision emergency, and
* its timed reverse/spin escape manoeuvre, and
* its timed back-up/spin recovery manoeuvres.

Keeping one writer matters: `twist_mux` passes the newest message on the
highest-priority input, so a second publisher on the same topic would fight the
mission node's escape (a fast zero-publisher would simply out-vote it). The
`dojo_bt` `EmergencyStop` node is still registered for use in other trees, but
the shipped tree deliberately does **not** publish zeros — it withholds
controller commands via `IsCollisionImminent` and lets the mission node arbitrate.

Config: `src/dojo_nav/config/twist_mux.yaml`.

---

## Behavior Tree

`src/dojo_bt/behavior_trees/maze/autonomous_maze.xml` is loaded by
`nav2_bt_navigator` via `default_nav_to_pose_bt_xml`
(`src/dojo_bt/config/dojo_bt_params.yaml`). Shape:

```text
RecoveryNode (6 retries)
├── PipelineSequence
│   ├── RateController(1 Hz) → RecoveryNode(1) → ComputePathToPose + ClearGlobalCostmap
│   └── RecoveryNode(1) → GuardedFollowPath
│         ├── Inverter(IsCollisionImminent)   (only drive when the path is clear)
│         └── FollowPath
└── RecoveryStrategy
      ├── MissionStuck  → BackUp + Spin
      ├── IsCollisionImminent → BackUp + Wait
      └── RoundRobin (clear costmaps / Spin / Wait)
```

The tree only has to reach the goal handed to it by the navigator. Mission
orchestration (mission types, frontier selection, goal sequencing, recovery
timing) lives in `autonomous_mission_node`.

Custom nodes are provided by `libdojo_bt_plugins.so`
(`src/dojo_bt/src/bt_conditions.cpp`, `bt_actions.cpp`) and registered through
`plugin_lib_names` in `dojo_bt_params.yaml`. Nav2 *appends* that list to its own
built-in plugin list, so the stock Nav2 nodes remain available.

> **Naming note:** two custom nodes deliberately avoid Nav2's node IDs, because
> BehaviorTree.CPP throws on duplicate registrations: our stuck check is
> `MissionStuck` (Nav2 already has `IsStuck`) and our dock action is
> `MissionDock` (Nav2 already has `DockRobot`).

---

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Final command to the Gazebo diff-drive plugin |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Controller output |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | Smoother output |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | Collision-monitor output |
| `/cmd_vel_emergency` | `geometry_msgs/Twist` | Highest-priority mux input |
| `/scan` | `sensor_msgs/LaserScan` | Lidar |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/imu` | `sensor_msgs/Imu` | IMU |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF output |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM map |
| `/dojo_mission` | `std_msgs/String` | Mission commands (JSON) |
| `/dojo_mission/state` | `std_msgs/String` | State machine snapshot (JSON, 1 Hz) |
| `/dojo_mission/status` | `std_msgs/String` | Human-readable status |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Transforms |

Manual stop:

```bash
ros2 topic pub --once /cmd_vel_emergency geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## Key files

| File | Purpose |
|---|---|
| `src/dojo_bt/launch/autonomous_maze.launch.py` | All-in-one autonomous launch |
| `src/dojo_nav/launch/navigate.launch.py` | Nav2 on a saved map |
| `src/dojo_nav/config/nav2_params.yaml` | Nav2 parameters (all nodes) |
| `src/dojo_nav/config/twist_mux.yaml` | Velocity arbitration (`/cmd_vel_emergency`) |
| `src/dojo_bt/config/dojo_bt_params.yaml` | BT XML path + `plugin_lib_names` |
| `src/dojo_bt/behavior_trees/maze/autonomous_maze.xml` | The Behavior Tree |
| `src/dojo_bt/src/bt_conditions.cpp` | Custom BT conditions |
| `src/dojo_bt/src/bt_actions.cpp` | Custom BT actions |
| `src/dojo_bt/dojo_bt/autonomous_mission_node.py` | ROS interface / action clients |
| `src/dojo_bt/dojo_bt/mission_state_machine.py` | State machine logic |
| `src/robodojo/urdf/robodojo.urdf` | Robot description |
| `maps/arena.yaml` | Default Nav2 map |

---

## Troubleshooting

**No `/scan`, `/odom`, `/imu`, or `/clock`** — the `ros_gz_bridge` did not start
or Gazebo is not running. Check for the bridge in the launch output, and that
the launch printed no Gazebo errors.

**`model://gamefield_arena` not found** — rebuild so the model files are
installed, and make sure `GZ_SIM_RESOURCE_PATH` includes the gamefield models
directory. The launch files set this automatically; for a manual `gz sim`:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/install/robodojo/share:$PWD/install/gamefield/share/gamefield/models"
```

**Nav2 nodes stay `unconfigured`** — check the `lifecycle_manager_navigation`
output; a missing plugin or parameters file makes a node fail to configure and
the manager then aborts the whole stack.

**BT fails to load in `bt_navigator`** — verify the tree:
`default_nav_to_pose_bt_xml` must point at an existing XML, every node ID must be
registered, and no custom node ID may collide with a stock Nav2 one
(BehaviorTree.CPP throws `ID [...] already registered`).

**The robot never moves** — check, in order:

```bash
ros2 topic hz /cmd_vel_nav        # controller producing commands?
ros2 topic hz /cmd_vel_safe       # collision monitor passing them through?
ros2 topic hz /cmd_vel            # twist_mux output?
ros2 topic echo /dojo_mission/state --once   # is_stuck / min_scan_range sane?
```

A `min_scan_range` of `null` means "no obstacle data"; the BT treats that as
"clear". A stale `null`-as-`0.0` reading would otherwise look like a permanent
collision.

`min_scan_range` is measured over the **forward cone** of the 360° lidar (±35°),
not the whole sweep. Using the global minimum looked "safer" but froze the robot
in this arena: the maze walls beside and behind it are permanently inside the
0.25 m emergency radius, so it held zero velocity forever. The whole-sweep and
rear-cone minima are still published as `min_scan_range_all` /
`min_scan_range_rear`, and the emergency state is escapable: after a 0.3 s hard
stop the robot reverses (or spins in place when the rear is also blocked) until
the way ahead is clear.

**Clean rebuild**

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## Notes on this branch

* Gazebo discovery is pinned to loopback in every launch that starts Gazebo
  (`GZ_IP`/`IGN_IP`), fixing the black-screen/instant-death failure.
* `default_bt_xml_filename` (removed in Nav2 Galactic) was replaced with
  `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml`; the old name
  was silently ignored, so the custom tree never loaded.
* `autonomous_maze.xml` was rewritten as a valid `BTCPP_format="4"` tree, and
  the custom BT node IDs `IsStuck`/`DockRobot` were renamed to
  `MissionStuck`/`MissionDock` to avoid the duplicate-registration failure.
* `/cmd_vel_emergency` is now actually connected: `collision_monitor` outputs
  `cmd_vel_safe` and `twist_mux` arbitrates it against the emergency topic before
  `/cmd_vel` reaches Gazebo.
* The collision gate was made **directional** and **escapable**. It used the
  minimum range over the whole 360° sweep, so in this arena (walls always within
  25 cm to the side or rear) the robot sat in `EMERGENCY` publishing zeros
  forever. It now watches the forward cone and, when blocked, stops briefly and
  then backs away instead of latching.
* Hard-coded, user-specific paths (`/home/keedastro/...`,
  `/home/damian/...`, `/opt/ros/kilted/...`) were removed from the launch files,
  `nav2_params.yaml` and `measure.py`; the default map is discovered from the
  repository.
* Node parameters that targeted non-existent nodes were removed, and
  `dojo_bt`/`dojo_nav` `package.xml` files now declare the packages their launch
  files actually use.
