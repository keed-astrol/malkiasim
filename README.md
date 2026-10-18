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
| `ros2 launch dojo_bt autonomous_maze.launch.py` | Everything above **plus** the custom BT and the autonomous mission node (`localization:=slam` to map live instead of using the saved map; `auto_explore:=false` to disable the automatic EXPLORE) |
| `ros2 launch gamefield world_rviz.launch.py` | World inspection in RViz |

The robot spawns at world `x = -2.50`, `y = -1.10`, `z = 0.6`, yaw `0` (map
`(0.20, 0.20)`, which is the AMCL `initial_pose`).

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

### Localisation modes

`autonomous_maze.launch.py` takes a `localization` argument that picks how `/map`
is produced:

| Value | What runs | Use when |
|---|---|---|
| `amcl` (default) | `map_server` + AMCL against the `map` argument | You already have a good map |
| `slam` | `slam_toolbox` (`async_slam_toolbox_node`) builds `/map` live | Unknown arena, or you want `EXPLORE` to map it |

```bash
# Explore an unknown arena: build the map and drive frontier goals, no map file
ros2 launch dojo_bt autonomous_maze.launch.py localization:=slam
```

SLAM mode uses `src/robodojo/config/slam.yaml` and is managed by
`lifecycle_manager_slam`; `map_server`/AMCL are not started. Note that a map
built live has no prior information — the robot only trusts what the lidar has
seen, so expect it to make a few detours before the frontier goals settle down.

If `localization:=slam` runs but the costmap logs

```
[global_costmap.global_costmap]: Received map message is malformed. Rejecting.
```

then `/map` is being published as a **0×0 grid**. `slam_toolbox` only resizes its
message when the grid dimensions change, so a mapper that has not received a
single scan publishes `width = height = 0`, and Jazzy's `nav2_util::validateMsg()`
rejects any `MapMetaData` with `width == 0 || height == 0`. The global costmap
then has no map at all (`Can't update static costmap layer, no map received`,
`Robot is out of bounds of the costmap`) and nothing can plan.

Two separate faults produce that empty grid.

**1. A parked rover maps nothing.** `slam_toolbox`'s `shouldProcessScan()` only
accepts a scan after the robot has travelled `0.8 × minimum_travel_distance`,
and the single "first measurement" pass does not make it into the published
grid. With the default `minimum_travel_distance: 0.1` the rover therefore
published a 0×0 map for as long as it sat still after launch — measured: `/map`
stayed 0×0 for minutes, then jumped to 25×60 the moment the rover was nudged
~0.3 m. Nav2 then rejects the map, the costmap has no static layer and `EXPLORE`
never gets a frontier, so nothing ever moves: a deadlock. `slam.yaml` now sets
`minimum_travel_distance: 0.0`, so scans are accepted on the
`minimum_time_interval: 0.2` timer instead of on travel.

**2. Scan bursts overflow the tf2 filter.** `slam_toolbox` subscribes through a
tf2 `MessageFilter` whose default queue holds **one** scan; at 100 Hz the queue
overflowed (`Message Filter dropping message: frame 'lidar_link' ... queue is
full`). The lidar now runs at 15 Hz (`robodojo.urdf`) and `slam.yaml` sets
`scan_queue_size: 5` / `transform_timeout: 0.5`.

Verify with:

```bash
ros2 topic echo /map --field info --once     # width/height must be non-zero
ros2 topic echo /pose --once                 # slam_toolbox's own pose estimate
ros2 topic hz /scan                          # want ~15 Hz
ros2 run tf2_ros tf2_echo odom lidar_link    # the transform the filter waits for
```

Jazzy's `slam_toolbox` also logs

```
[lifecycle_manager_slam]: Server slam_toolbox was unable to be reached after 4.00s by bond
```

at activation. It is cosmetic here (the node is `active`, publishes `/map` and
`/tf`); the bond handshake happens while `on_activate` is still starting the
mapper threads.

A full autonomous run has been saved as `maps/arena_slam.{pgm,yaml}`
(124×65 cells @ 0.05 m) so you can drive the same arena in AMCL mode without
mapping it again:

```bash
cd <repository root>
ros2 launch dojo_bt autonomous_maze.launch.py localization:=amcl \
  map:=$PWD/maps/arena_slam.yaml
```

One caveat: a SLAM-built map puts its origin at the pose where mapping started,
so the spawn is at map `(0, 0)` in `arena_slam`, while the `initial_pose`
hard-coded in `nav2_params.yaml` (0.20, 0.20) was measured for the shipped
`maps/arena.yaml` (which uses `map = world + (2.70, 1.30)`). Edit the
`initial_pose` params to `x: 0.0, y: 0.0` before using `arena_slam`.

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
| `EXPLORE` | – | Frontier-based exploration (see below) |
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

### What `EXPLORE` does

`EXPLORE` is real frontier exploration, driven by the `/map` the node subscribes
to (from `map_server` in `amcl` mode, or `slam_toolbox` in `slam` mode):

1. A **frontier** is a known-free cell that touches unknown space. Adjacent
   frontier cells are clustered (specks under 4 cells are dropped).
2. Each cluster is scored `size / (1 + distance)`; the representative goal cell
   is the one nearest the cluster centroid that has `FRONTIER_CLEARANCE` (0.20 m)
   of clear space, so goals are never handed to the planner inside a wall.
3. The goal is sent to `navigate_to_pose`, the next frontier is picked once it is
   reached, and so on until no frontier survives (`FRONTIER_EMPTY_ATTEMPTS`
   consecutive empty scans), at which point the mission completes.
4. A frontier that fails or is rejected is **blacklisted** for 90 s and the
   mission continues with the next one, instead of giving up.

In `slam` mode the robot **starts `EXPLORE` by itself**, about 5 s after launch,
as soon as `/map` has real cells and a `map → base_footprint` transform exists —
there is no need to publish `/dojo_mission` first:

```bash
ros2 launch dojo_bt autonomous_maze.launch.py localization:=slam
#   [autonomous_mission_node]: Auto-start: EXPLORE after 5.1s (map 25x59)
#   [autonomous_mission_node]: EXPLORE: heading to frontier (0.33, 1.16) of 3
```

Turn it off with `auto_explore:=false` (e.g. to drive goals by hand with RViz's
*2D Goal Pose*; in `amcl` mode it is disabled automatically). Once the whole map
is explored the mission ends on its own (`EXPLORE: map fully explored` → `IDLE`).

The autostart waits for a non-empty map on purpose: `slam_toolbox` publishes an
empty 0×0 grid until it has processed its first scan, and Nav2 rejects that
(`Received map message is malformed`).

Note that a *static, fully explored* map has no frontiers at all — with the
shipped `maps/arena.yaml` an `EXPLORE` command finishes almost immediately. Use
`localization:=slam` (or a map with unexplored regions) for it to do anything.

### Monitoring

```bash
ros2 topic echo /dojo_mission/status   # human-readable
ros2 topic echo /dojo_mission/state    # JSON state machine snapshot
```

`/dojo_mission/state` is published at 1 Hz and contains `state`, `mission_type`,
`mission_active`, `is_stuck`, `min_scan_range`, `min_scan_range_all`,
`min_scan_range_rear`, `frontiers`, `goal_in_flight`, `pose_x`, `pose_y`,
`current_yaw`. The Behavior Tree reads this topic for its collision/stuck
decisions.

Both topics are `std_msgs/String` messages whose payload is JSON, so `--field`
cannot be used (there are no message fields to select); echo the whole message:

```bash
ros2 topic echo /dojo_mission/state --once
```

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
`/cmd_vel_emergency` has a **single writer**, `autonomous_mission_node`, and it
publishes on it for one reason only: the hard-stop phase of a collision
emergency (0.5 s). After that it goes **silent for 2.5 s** so `twist_mux` (0.5 s
timeout) falls back to Nav2, whose tree withholds the controller via
`IsCollisionImminent` and runs its own `BackUp`/`Spin` recovery. The gate is a
stop, not a driver: re-driving the wheels from the mission node made the two
layers take turns reversing.

Keeping one writer matters: `twist_mux` passes the newest message on the
highest-priority input, so a second publisher on the same topic would fight the
mission node. The `dojo_bt` `EmergencyStop` node is still registered for use in
other trees, but the shipped tree deliberately does **not** publish zeros — it
withholds controller commands via `IsCollisionImminent` and lets the mission node
arbitrate.

**Stuck recovery is owned by the tree, not by this node.** The `MissionStuck`
branch of the BT runs `BackUp + Spin` through Nav2's behaviour server, which
halts the controller first — one system at a time. An earlier revision had the
mission node publish its own back-up/spin on `/cmd_vel_emergency` as well; that
outranked the controller, so every attempt to pivot into a doorway was
countermanded and the robot circled. `_handle_recovery` now only waits, clears
the costmaps and hands control back.

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
| `src/dojo_bt/dojo_bt/autonomous_mission_node.py` | ROS interface / action clients / frontier exploration |
| `src/dojo_bt/dojo_bt/mission_state_machine.py` | State machine logic |
| `src/robodojo/urdf/robodojo.urdf` | Robot description |
| `src/robodojo/config/slam.yaml` | `slam_toolbox` mapping config |
| `maps/arena.yaml` | Default Nav2 map |

---

## Troubleshooting

**No `/scan`, `/odom`, `/imu`, or `/clock`** — the `ros_gz_bridge` did not start
or Gazebo is not running. Check for the bridge in the launch output, and that
the launch printed no Gazebo errors.

**`Detected jump back in time. Clearing TF buffer.` spam, and the rover drives
into walls without stopping** — two `/clock` publishers are fighting, almost
always a *leftover* `parameter_bridge` (and `twist_mux`) from an earlier run
whose launch was only partially killed. Every backwards jump clears the TF
buffers, so scans can no longer be transformed: the costmap obstacle layer goes
blind, `collision_monitor` logs `Robot to stop due to invalid source`, and
`slam_toolbox` drops its scans. Verify:

```bash
ros2 topic info /clock               # must be "Publisher count: 1"
ros2 topic info /scan                # must be 1 as well
ros2 node list | grep -c twist_mux   # must be 1
```

Grep the launch log for `jump back in time` — a healthy run has zero.

**Restarting from scratch** — killing the wrong subset of nodes is what leaves
those duplicates behind. Use one script for the whole set (`pkill -f` must not
appear in the caller's own command line, or it kills the caller too):

```bash
cat > ~/robo_kill.sh <<'EOF'
#!/usr/bin/env bash
for pat in autonomous_maze.launch autonomous_mission_node parameter_bridge \
  ros_gz_bridge component_container lifecycle_manager robot_state_publisher \
  twist_mux slam_toolbox ekf_node "gz sim" rviz2 "ros2 launch" ros2-daemon; do
  pkill -9 -f -- "$pat" 2>/dev/null
done
sleep 3
EOF
bash ~/robo_kill.sh
ros2 daemon stop          # a daemon left inside the container holds the
                          # podman overlay mount, which then cannot be remounted
ros2 node list            # must be empty
```

Then `colcon build --symlink-install` and relaunch.

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
`min_scan_range_rear`. The emergency exits once the front reaches
`COLLISION_CLEAR_RANGE` (0.30 m) — deliberately below 0.40 m, because a 0.42 m
corridor with a 0.28 m rover never offers 0.40 m of clear front and the rover
then never left `EMERGENCY`.

**The robot wedges on the plateau ramps, and the planner reports `Failed to
create a plan from potential when a legal potential was found`** — this is a
sensor-geometry trap, not a planner bug. The lidar is a horizontal 2D scanner
mounted **on** the robot, 0.1975 m above the floor. When the rover pitches on the
arena's plateau ramps (12 %, ~7°) its beams hit the floor ahead at
`lidar_height / tan(pitch)`, so the floor must stay outside the obstacle layer's
marking range:

```
obstacle_max_range < lidar_height / tan(max_pitch)
       1.2 m       < 0.1975 m   / tan(7 deg) = 1.61 m
```

Otherwise the floor is painted into the costmap as an obstacle wall ~1.6 m in
front of the robot — exactly across the corridor it is trying to drive down.
Both costmaps therefore use `obstacle_max_range: 1.2`, which makes the floor
invisible for pitches up to 9.3° and keeps ~0.5 m of headroom over the real 7°
ramp. `raytrace_max_range` stays 3.0 so cells are still cleared, and the
`collision_monitor` watches the raw scan for genuine collisions, so shortening
the marking range costs no safety.

The plateaus (floor mesh, world frame):

| plateau | world x | world y | shape |
|---|---|---|---|
| A | 0.25 … 1.65 | ≈ −0.6 | 5 cm high, ramps at both x ends |
| B | −1.2 … 0.1 | ≈ 1.2 | 5 cm high, ramps at both x ends |

The plateau **sides** are 5 cm vertical faces: invisible to a lidar mounted at
0.198 m, and too narrow to squeeze past (the gap beside them is ~0.24 m against a
0.286 m rover), so the ramps are the only way across. `/tmp/opencode/test_ramp_geometry.py`
re-derives all of this from the URDF and the floor mesh and fails if
`obstacle_max_range` ever grows back above the floor-return distance.

**The robot circles in place and won't enter a doorway** — this is a
recovery/authority problem rather than a planning one, and there are three
places to look:

* `ros2 topic echo /dojo_mission/state --once` — an `is_stuck: true` while the
  robot is pivoting means something is treating rotation as a failure. The stuck
  check counts **translation or rotation** as progress (12 s / 0.08 m / 0.35 rad)
  precisely so that lining up with a corridor is not mistaken for being stuck.
* The `MissionStuck` branch must be the only thing that moves the robot during
  recovery. If the mission node is also publishing on `/cmd_vel_emergency`, the
  priority-100 mux input wins and the controller is overridden.
* `collision_monitor`'s approach polygon projects the footprint ahead by
  `time_before_collision`; the default 1.2 s is ~0.31 m of look-ahead at full
  speed, which registers a ~0.5 m doorway as a collision and stops the robot
  exactly where it needs to commit. It is set to 0.5 s here.

Wall keep-off is a compromise: the hard keep-out is the footprint's **inscribed
radius**, which no parameter can reduce. `inflation_radius` (0.22 m) and
`cost_scaling_factor` (8.0) only shape the *preference* for the corridor centre
— a steep falloff and a tight radius make the robot willing to squeeze past
walls, they do not let it clip them.

The arena is genuinely tighter than the rover. Measuring the walls mesh gives
corridors of median **0.42 m** width (25th percentile 0.22 m) against a
0.271 × 0.286 m rover, and **36 %** of the free floor has less clearance than the
real 0.136 m inscribed radius — the planner treats it as lethal. At the true
footprint the channel for the robot's centre in a median corridor is only
~0.15 m, so NavFn frequently reports no plan at all.

The costmaps therefore plan with a **deliberately smaller** footprint
(`[[0.125, 0.115], …]`, inscribed 0.115 m), which buys ~4.2 cm of channel per
corridor. The cost is that DWB will accept trajectories where the wheels overlap
a wall by up to ~0.026 m, so expect scraping in tight spots. The
`collision_monitor` is deliberately **not** shrunk with it: it uses a static
polygon at the real size (`0.145 × 0.150`) instead of
`/local_costmap/published_footprint`, so the emergency stop still believes the
rover is as wide as it is. Passages much below the rover's 0.286 m width remain
impossible — there the honest fixes are a smaller rover or a wider world.

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
* The collision gate watches only the forward cone (±35°) and is a **stop, not
  a driver**: it brakes for 0.5 s, then releases `/cmd_vel_emergency` for 2.5 s
  so `twist_mux` hands control back to Nav2, whose tree withholds the controller
  and runs its own `BackUp`/`Spin`. Driving the wheels from both layers made them
  take turns reversing (the rover rocked back and forth), and holding the topic
  at 10 Hz outranked Nav2 forever. The exit threshold is 0.30 m rather than
  0.40 m: a 0.42 m corridor with a 0.28 m rover never offers 0.40 m of clear
  front, which trapped it in `EMERGENCY` permanently.
* Hard-coded, user-specific paths (`/home/keedastro/...`,
  `/home/damian/...`, `/opt/ros/kilted/...`) were removed from the launch files,
  `nav2_params.yaml` and `measure.py`; the default map is discovered from the
  repository.
* Node parameters that targeted non-existent nodes were removed, and
  `dojo_bt`/`dojo_nav` `package.xml` files now declare the packages their launch
  files actually use.
* Turning is much tighter: DWB `max_vel_theta` 2.0 rad/s with ±4.0 rad/s²
  limits, matching limits in the velocity smoother, behaviour server and the
  Gazebo diff-drive plugin. The robot also plans with its **measured footprint**
  (`0.272 × 0.281 m`, from the meshes) rather than a circumscribed
  `robot_radius`, so it no longer keeps a phantom 0.18 m of clearance on every
  side.
* Fixed the ramp trap that wedged the robot and made the planner report
  `Failed to create a plan from potential when a legal potential was found`: with
  `obstacle_max_range: 2.5` the lidar painted the plateau ramps' floor into the
  costmap as a phantom wall ~1.6 m ahead. Both obstacle layers now stop marking
  at 1.2 m (see Troubleshooting).
* Fixed `localization:=slam` never getting a map. The decisive cause is
  `slam_toolbox`'s travel gate: it adds no scan until the rover has moved
  `0.8 × minimum_travel_distance`, and the single first-scan pass never reaches
  the published grid, so a rover parked after launch published a 0×0 `/map`
  forever — which Jazzy rejects as `malformed`, leaving the global costmap empty
  and `EXPLORE` without frontiers. That is a deadlock, because the rover cannot
  move without a map. `minimum_travel_distance: 0.0` makes the mapper work on
  the `minimum_time_interval` timer instead of on travel. The lidar also went
  from 100 Hz to 15 Hz and `slam.yaml` gained `scan_queue_size: 5` /
  `transform_timeout: 0.5` so scan bursts do not overflow the tf2 `MessageFilter`.
* The planners now run a **smaller footprint than the metal** (inscribed 0.115 m
  instead of 0.136 m) because the maze's median corridor is 0.42 m against a
  0.286 m rover. The `collision_monitor` keeps the true footprint via a static
  polygon, so the emergency stop is not weakened.
* Spawn moved to world `(-2.50, -1.10)` (spawn wall clearance 0.22 m → 0.44 m)
  with the AMCL `initial_pose` updated to match, using the measured map frame
  `map = world + (2.70, 1.30)`.
* `EXPLORE` was a stub and `complete_current_mission_step()` never fired: the
  mission node attached its result callback to `send_goal_async`, which resolves
  when the goal is *accepted*, not when it finishes. Goals now chain through
  `_on_nav_goal_response` → `get_result_async()` → `_on_nav_result`, and missions
  actually complete.
* `EXPLORE` now does frontier exploration over `/map`, and
  `autonomous_maze.launch.py` gained `localization:=slam` to build that map live
  with `slam_toolbox`.
* The mission node no longer fights the controller for the wheels: its stuck
  recovery only bookkeeps and clears costmaps, leaving `BackUp + Spin` to the
  tree's `MissionStuck` branch. Stuck detection also counts rotation as progress
  and waits 12 s instead of 5 s.
* `EXPLORE` starts by itself in `slam` mode (`auto_explore`, on by default):
  ~5 s after launch, once `/map` has real cells and `map → base_footprint`
  exists, the mission node enters `EXPLORING`. `auto_explore:=false` restores
  manual `2D Goal Pose` driving; in `amcl` mode autostart is disabled.
* Fixed a crash that made every `EXPLORE` mission die on its first tick:
  `_last_frontier_attempt` was a `rclpy.time.Time` while the dispatcher
  subtracted a float from it
  (`TypeError: unsupported operand type(s) for -: 'float' and 'Time'`).
* Replanning dropped to **0.5 Hz** (`RateController` in the tree) and the
  translational limit to **0.20 m/s**: at 1 Hz / 0.26 m/s the global path
  flipped between near-equal options in the tight divisions before the
  controller could commit, which read as the rover dithering.
* A full autonomous run mapped the whole arena and it is saved as
  `maps/arena_slam.{pgm,yaml}` (124×65 @ 0.05 m) as a ready-made prior map for
  `localization:=amcl`.
