# Changelog — `fixes` branch

Comparison: **`main` → `fixes`** · 21 commits (two are merges of `main`) · 36 files,
+2 676 / −1 068 lines.

Everything below was verified on the running Jazzy + Gazebo Harmonic sim, not
just read from the code.

---

## TL;DR

* The stack now **runs end to end**: Gazebo + EKF + Nav2 + our BT plus an
  autonomous mission node that maps, drives and completes missions.
* **`localization:=slam` works and auto-starts `EXPLORE`**: the rover maps the
  whole 6 × 3 m arena by itself and saves a map of **6.10 × 3.15 m** (a 1 % match
  to the arena, reproducible across runs).
* **Missions work on the self-built map** (`localization:=amcl`, default map
  `maps/arena_slam2.yaml`): `GOAL`, `WAYPOINTS` (3/3), `DOCK`, `CANCEL`,
  `STATUS`. Example: a goal at `(3.0, 1.5)` was reached at `(3.007, 1.660)`.
* The simulation is deliberately throttled to **0.6 × real time** and the drive
  limits match a heavily geared-down robot (0.15 m/s, 0.35 m/s², 1.2 rad/s,
  1.5 rad/s²). Gauging a "normal" speed is what made it crash into walls.
* **Open item — plateaus (important for missions, not for mapping).** The 5 cm
  plateaus are invisible to the 0.198 m-high 2D lidar, so **no map contains
  them** (checked: both the SLAM map and the shipped `maps/arena.pgm` report free
  at their centres). A planner will route across one and the rover stops at the
  lip — this is what killed the `DOCK` test. Options are listed in
  [Known limitations](#9-known-limitations--next-steps).
* **Sim-only assumptions** are called out in
  [Moving to the real robot](#8-moving-to-the-real-robot). Read that section
  before touching hardware.

---

## 1. Bring-up and environment

| Problem | Fix |
|---|---|
| Gazebo GUI black screen / world dies instantly | `gz-transport` multicast discovery fails on some hosts. Every launch that starts Gazebo now pins `GZ_IP=127.0.0.1` and `IGN_IP=127.0.0.1`. |
| Old, ignored BT parameter | `default_bt_xml_filename` (removed in Galactic) → `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml`. |
| Hard-coded user paths | Removed `/home/keedastro/...`, `/opt/ros/kilted/...` from launches, params and `measure.py`; the default map is discovered by walking up from the launch file. |
| Missing runtime deps | `dojo_bt`/`dojo_nav` `package.xml` now declare every package their launches use (Nav2 servers, `twist_mux`, `robot_localization`, `slam_toolbox`, …). |
| Parameter substitutions | `ParameterFile(..., allow_substs=True)` so `$(find-pkg-share ...)` resolves. |
| **Duplicate `/clock` publishers** | A half-killed run left a `parameter_bridge` (and a second `twist_mux`) alive: two `/clock` streams produced **70 213** `Detected jump back in time. Clearing TF buffer.` messages, which blinds the costmap, collision monitor and SLAM. The README now has a one-shot kill script covering *all* nodes plus `ros2 daemon stop` (a daemon left in the container holds the podman overlay mount). |
| Nav2 bringup stalled | At 0.6 × real time, starting Nav2 3 s after Gazebo left `planner_server` in `configure`; the manager blocked, `bt_navigator` never activated and goals were silently swallowed. Bringup delay is now **10 s**. |
| Lifecycle bond timeouts | `slam_toolbox`'s on_activate can outlast the 4 s bond window, leaving it **inactive** (no `/map`, no `map` frame → everything blocks on `Invalid frame ID "map"`). Both lifecycle managers use `bond_timeout: 0.0`. |

---

## 2. Navigation stack

**Velocity chain** (single path, documented in `README.md`):

```
controller_server  -> cmd_vel_nav
velocity_smoother  -> cmd_vel_smoothed
collision_monitor  -> cmd_vel_safe          ┐
/cmd_vel_emergency (mission node)           ├─ twist_mux -> cmd_vel -> Gazebo
                                            ┘   priority 100 vs 10
```

* `twist_mux.yaml` added (emergency 100, navigation 10, 0.5 s timeouts);
  `collision_monitor.cmd_vel_out_topic` → `cmd_vel_safe`.
* `behavior_server.cmd_vel_topic: cmd_vel_nav` — by default Nav2 sends
  `BackUp`/`Spin` **straight to `/cmd_vel`**, bypassing the smoother and the
  collision monitor, so recovering backwards drove blind into walls.
* **Rotation shim controller** wraps DWB. DWB only samples forward arcs, so a
  rover 0.21 m from a wall it must turn away from was vetoed by
  `collision_monitor`'s swept footprint *forever* (measured: `nav.x +0.150`,
  `nav.w +0.786`, `safe.x/w 0.000`, pose frozen for 15 s). Now >45° off the path
  it rotates in place (0.6 rad/s) until within 22.5°, then drives.
* Motion limits (also in the Gazebo DiffDrive plugin): **0.15 m/s, 0.35 m/s²,
  1.2 rad/s, 1.5 rad/s²**. The velocity smoother and behaviour server match.
* Replan rate **0.3 Hz** (was 1 Hz): the path flipped between near-equal options
  before DWB could commit.
* **No rotation on arrival**: `yaw_goal_tolerance: 3.15` and the `RotateToGoal`
  critic removed. Spinning at each frontier clipped wall corners; the rover now
  keeps the heading it arrived with.
* Footprint: **0.128 × 0.134 m** for planning (the true metal is 0.272 × 0.281 m,
  inscribed 0.136 m). A smaller 0.115 m footprint let the planner aim at gaps the
  rover cannot pass; the full 0.141 m made the start area unplannable.
* Global costmap: **rolling 16 × 16 m** @ 0.05 (sized by the static layer it only
  covered the previous map, and every frontier goal — which sits on the map edge
  — was rejected in 6–13 ms with `worldToMap failed`). Local costmap 3 × 3 m @
  0.025.
* `cost_scaling_factor` 8.0 → **4.0** and `inflation_radius` 0.22: the steep
  falloff made wall-adjacent cost negligible so NavFn hugged walls.
* Obstacle layers mark to `obstacle_max_range: 1.2 m`. With 2.5 m the lidar
  painted the plateau ramps' floor into the costmap as a phantom wall
  (`Failed to create a plan from potential when a legal potential was found`).
  The rule and the numbers are in the README: `obstacle_max_range <
  lidar_height / tan(max_pitch)` (0.1975 m / tan 7° = 1.61 m).
* `collision_monitor` keeps the **true** padded footprint (`0.145 × 0.150`) via a
  static polygon, so the emergency stop is not weakened by the planner's
  smaller footprint.
* Custom BT rewritten as valid `BTCPP_format="4"`; custom node IDs renamed
  (`MissionStuck`, `MissionDock`) to avoid duplicate-ID aborts. It is the
  `NavigateToPose` tree only — the **through-poses** slot now points at Nav2's
  stock tree (using ours there made every waypoint plan to an empty frame).

---

## 3. Autonomous mission layer (`dojo_bt/autonomous_mission_node.py`)

`/dojo_mission` (JSON on `std_msgs/String`): `EXPLORE`, `GOAL`, `WAYPOINTS`,
`DOCK`, `CANCEL`, `STATUS`.

* **Goal chaining fixed**: results were attached to `send_goal_async` (which
  resolves on *acceptance*), so `EXPLORE` never completed and
  `complete_current_mission_step()` never fired. Now
  `_on_nav_goal_response → get_result_async() → _on_nav_result`.
* **Goal identity tagging**: a preempted/dropped goal delivers `ABORTED` later;
  that was blamed on the *new* goal, blacklisting every frontier and declaring
  the map complete on a 1.2 × 3 m map. Results carry their goal id and stale
  ones are ignored.
* **Frontier `EXPLORE`**: free cell touching unknown; clusters scored
  `size / (1 + distance)`; goal pulled 0.15 m back from the unknown edge
  (`FRONTIER_GOAL_INSET`); clearance 0.20 m; failed/rejected frontiers
  blacklisted 90 s; completion requires 3 consecutive empty scans **and** no
  known frontiers pending.
* **Auto-start**: in `localization:=slam`, `EXPLORE` starts itself ~5 s after
  launch once `/map` has cells, `map→base_footprint` exists and the
  `NavigateToPose` server is up. `auto_explore:=false` disables it; AMCL mode
  never auto-starts.
* **Wedge escape**: when Nav2 commands motion but the pose has not moved for
  4 s, the node reverses briefly on `/cmd_vel_emergency` (which outranks the
  collision monitor) and drops the goal. Rear-guarded: needs **0.45 m** of rear
  clearance to start, aborts below **0.35 m**, 0.05 m/s for 0.8 s (≤4 cm).
* **Collision gate is diagnostics only**: it publishes `min_scan_range*` (the BT
  reads `min_scan_range` for its recovery decision) but no longer drives the
  wheels — competing emergency stops/escapes caused the rocking back and forth.
* **Stuck recovery** is owned by the tree (`MissionStuck → BackUp + Spin`);
  the node only bookkeeps, clears costmaps and hands control back. Stuck
  detection counts rotation and waits 12 s.
* `CANCEL` now cancels the Nav2 action as well as the mission state (before, the
  navigator kept driving after a cancel).
* **`WAYPOINTS` drives one `NavigateToPose` at a time** and chains the next pose
  from the result, so each waypoint gets the same stuck/collision handling as a
  single goal.
* `is_stuck`, `mission_active` etc. are published at 1 Hz on
  `/dojo_mission/state` (JSON) and `/dojo_mission/status` (human readable).

---

## 4. SLAM and mapping

The map-related failures took the longest; here is what each one actually was.

| Symptom | Root cause | Fix |
|---|---|---|
| `/map` stays `0×0`, Nav2 rejects it as `malformed` | `slam_toolbox` only adds a scan after the robot has travelled `0.8 × minimum_travel_distance`; the single "first scan" pass never reaches the published grid, so a parked rover published an empty map **forever** — and a rover with no map cannot move, so `EXPLORE` deadlocked | `minimum_travel_distance: 0.0` (scans accepted on the `minimum_time_interval` timer) |
| Scans dropped in bursts (`Message Filter ... queue is full`) | the async mapper cannot drain the tf2 filter during a scan match | `scan_queue_size: 5 → 20` |
| Map "blew up" (rotated ~25°, duplicated walls, 7 × 5 m for a 6 × 3 m arena) | one false **loop closure** optimises globally; `loop_search_space_dimension: 8.0` rotates everything | `do_loop_closing: false` |
| Rover froze on a ramp while the gate paused scans | `map→odom` is stamped `last_scan_time + transform_timeout` unless `restamp_tf`; with scans paused that stamp froze (44.19 s while the clock was at 124 s) and every Nav2 lookup failed with `Lookup would require extrapolation into the past` | `restamp_tf: true`, and goals are stamped at send time |
| Map smeared at the ramps | a horizontal 2D lidar on a pitching body sweeps the floor into the beam ahead and over the wall behind; a 2D matcher cannot reconcile that | `dojo_bt/level_scan_gate`: forwards `/scan` → `/scan_level` only while IMU pitch ≤ **2.5°**; `slam_toolbox` subscribes to `/scan_level`, Nav2 keeps raw `/scan` |

Other SLAM settings: 0.05 m resolution, `map_update_interval` 1 s,
`transform_publish_period` 0.02 s, `minimum_time_interval` 0.5 s,
`max_laser_range` 8 m, `min_laser_range` 0.15 m.

---

## 5. Simulation fidelity

* **Real time factor 0.6** via `<physics><real_time_factor>0.6</real_time_factor>`
  in both worlds (`max_step_size` unchanged, so the dynamics are identical).
  Measured with a `/clock` probe: **RTF 0.60**. The rover's inertia and the
  controller's corrections are now followable on screen.
* **Lidar 100 Hz → 15 Hz** (`robodojo.urdf`; the IMU stays 100 Hz). At 100 Hz the
  sim ran at ~6 % real time and slam's filter overflowed.
* **Spawn height `-z 0.08`** (was 0.6): dropping the rover 0.55 m made it bounce
  and spin at every launch. A value of 0.05 was worse — the wheels' contact
  plane is **0.0604 m below `base_link`**, so it spawned 1 cm *inside* the floor.
* **Measured drive parameters**: `wheel_separation 0.2437` (was 0.243),
  `wheel_radius 0.0407` (was 0.0406), footprint 0.272 × 0.281 m (from the
  meshes).
* DiffDrive limits: `max_linear_velocity 0.15`, `max_linear_acceleration 0.35`,
  `max_angular_velocity 1.2`, `max_angular_acceleration 1.5`.
* Gazebo camera follows the rover (`/gui/follow` called 12 s after launch).

---

## 6. Tooling, docs and maps

* `README.md` rewritten and kept current: launch/mode table, mission API, velocity
  chain, BT description, SLAM diagnostics, ramp geometry rule, restart-from-
  scratch procedure (incl. `ros2 daemon stop`), and the note that
  `autonomous_mission_node.py`/`level_scan_gate.py` are **installed as copies**
  (`CMakeLists.txt install(PROGRAMS)`), so they need a `colcon build`.
* `maps/arena_slam2.{pgm,yaml}` — the map the robot built itself; now the default
  prior map.
* `maps/arena_slam.{pgm,yaml}`, `maps/arena1.*` — earlier runs, kept for
  comparison.
* `maps/arena.yaml` — the curated prior map (unchanged).

---

## 7. Evidence

* Sim clock: **RTF 0.60** (`/clock` delta vs wall time).
* `/scan` 15 Hz (sim), ≈9 Hz wall at 0.6 RTF; `/scan_level` identical when level.
* Autonomous maps (two independent runs):

  | | run 1 | run 2 |
  |---|---|---|
  | extent | 122 × 64 @ 0.05 = **6.10 × 3.20 m** | 122 × 63 = **6.10 × 3.15 m** |
  | occupied / free / unknown | 11.7 / 82.4 / 5.9 % | 11.2 / 84.1 / 4.8 % |
  | arena for reference | 6.00 × 3.00 m | 6.00 × 3.00 m |

* `GOAL`: spawn `(0,0)` → `(3.007, 1.660)` (target `3.0, 1.5`).
* `WAYPOINTS`: `Waypoint 1/3 → reached → 2/3 → reached → 3/3 → reached`.
* `CANCEL`: `NAVIGATING` → `IDLE`, Nav2 action cancelled.
* Healthy run counters: `jump back in time` **0**, `Extrapolation Error` **0**,
  `queue is full` **0**, `malformed` **0**.

---

## 8. Moving to the real robot

**Strip the sim-only pieces**

* `real_time_factor` in the world files, `GZ_IP`/`IGN_IP`, `/gui/follow`, the
  spawn `-z 0.08`, `ros_gz_bridge`, and the `auto_arena`/`gamefield` launches are
  simulation-only.
* The maps and the numbers measured from them (`arena_slam2`, the arena's 6 × 3 m
  geometry) are sim-specific.

**Powertrain / controller tuning**

* The DiffDrive limits in `robodojo.urdf` are the *simulated plant*. Your motor
  driver must enforce the same limits (0.15 m/s, 0.35 m/s², 1.2 rad/s,
  1.5 rad/s²), otherwise DWB and the velocity smoother are tuned for a robot
  that no longer exists.
* Wheel radius/separation are measured from the CAD meshes (0.0407 m /
  0.2437 m). Measure the real wheels; a 1 % error causes visible yaw drift.
* Real wheels slip. `do_loop_closing` is off here because a false closure rotated
  the map; on hardware with more slip, either re-enable it or add 3D/landmark
  localisation and expect to retune `link_match_*` and the correlation search.

**Sensors**

* `level_scan_gate` uses the **raw IMU orientation**, not TF (the EKF is
  `two_d_mode`, so pitch is not in the tree). It needs an IMU with usable
  roll/pitch bias; retune `pitch_limit` for the real ramps and the real IMU noise.
* 15 Hz lidar / `scan_queue_size 20` / `minimum_time_interval 0.5` are tuned for
  this sensor and this sim's CPU. Recheck `Message Filter ... queue is full`
  after changing either.
* `obstacle_max_range: 1.2` is derived from **lidar height 0.1975 m** and max
  pitch 7°: `range < height / tan(pitch)`. If the real lidar sits at a different
  height or the ramps differ, recompute it or the floor becomes a phantom wall.
* The planners use a **0.128 × 0.134 m** footprint while the monitor uses
  0.145 × 0.150. That deliberately accepts a little wheel scrape in tight gaps.
  Measure the real bumpers before trusting it.

**Safety paths to review**

* `/cmd_vel_emergency` (mission node, priority 100) is the **only** command that
  bypasses the collision monitor. Its reverse is rear-guarded in software — keep
  that guard, or route it through a hardware/velocity-level check.
* The collision monitor's lookahead is `time_before_collision 0.5 s`. At the sim
  speed that is 7.5 cm; at higher real speeds it is correspondingly longer.
* `bond_timeout: 0.0` is safe in a single launch but means a dead node is not
  detected by bond. Keep an external heartbeat/watchdog.

**Localisation / maps**

* A SLAM-built map has its origin at the **start pose**, so AMCL's
  `initial_pose` must match: `0,0` for `arena_slam2`, `0.20, 0.20` for
  `maps/arena.yaml` (frame `map = world + (2.70, 1.30)`). The default in
  `nav2_params.yaml` is `0,0` and the default map is `arena_slam2`.
* Save maps explicitly (`nav2_map_server map_saver_cli -f <abs path>`); nothing
  saves automatically.

---

## 9. Known limitations / next steps

1. **Invisible plateaus (top priority for missions).** 5 cm high, lidar at
   0.198 m, so they appear as free space in every map. The rover bumps the lip
   and stops (`DOCK` test failed this way at plateau B). Three options:
   * **low scan / cliff sensor** (recommended): a second 2D lidar ~4 cm above the
     floor — a horizontal beam never returns from a flat floor, so the plateau
     sides become clean obstacles; add it to the local costmap's obstacle layer.
     Models a real cliff/ToF sensor.
   * **bump marking**: on a stall, insert a small obstacle into the local costmap
     (PointCloud2 source). Portable to any robot without new sensors, but it has
     to bump once per edge.
   * **stamp the plateau outline** from the arena mesh into the map files.
     Instant for this fixed arena, but arena-specific.
2. `EXPLORE` can report `map fully explored` while some reachable corridor is
   still dark (it happened once at a crossroads; a manual waypoint finished the
   map). It is a frontier-exhaustion heuristic, not a coverage guarantee.
3. The planner legitimately refuses some gaps: the plateau sides leave ~0.24 m
   against a 0.28 m rover. Those frontiers are blacklisted rather than driven.
4. RTF 0.6 makes wall-clock tests ~1.7 × longer. That is intentional.
5. No 3D perception: no overhangs, no negative obstacles, no plateau tops.

---

## 10. Commits

```
0fa7295 Cancel stops the robot: cancel the Nav2 action, not just the mission state
12b779e Drive WAYPOINTS from the mission node, one NavigateToPose at a time
2365290 Use the self-built map by default and match the AMCL initial pose
60af851 Add the clean autonomous SLAM map (arena_slam2)
5ea5206 Stop the map blowing up: no loop closure, no dropped scans
03e74fa Keep map->odom fresh while scans are paused on a ramp
4abb0c8 Let the controller rotate in place instead of freezing against walls
fd4de53 Pause SLAM scans while the rover is pitched on a ramp
8f7def5 Keep the heading on arrival and make every reverse short and slow
de59673 Follow the rover with the Gazebo camera, straighten paths, de-smear SLAM
4ac13f0 Stop the recovery reverse from driving into walls
977bab1 Delay the Nav2 bringup so planner_server cannot stall in configure
a6677a0 Run the sim at 0.6x and make the rover behave like a geared-down robot
105fad6 Make SLAM exploration work: auto-start, parked mapping, stop-and-release gate
c780567 Fix the ramp trap, the empty SLAM map, and the too-wide keep-out
1986689 Stop the robot circling at doorways; add frontier EXPLORE and SLAM mode
86ba9cc Tune turning and wall clearance for narrow maze corridors
667da36 Fix collision gate freezing the robot: make it directional and escapable
3e49a80 Fix Gazebo peer discovery, Nav2/BT wiring, and hardcoded paths
a47bde7 Merge branch 'main' into fixes
```

---

## 11. Quick commands

```bash
cd <repository root>
colcon build --symlink-install          # required after editing the Python nodes

# Map the arena autonomously (auto-starts EXPLORE, ~0.6x real time)
ros2 launch dojo_bt autonomous_maze.launch.py localization:=slam

# Drive missions on the self-built map
ros2 launch dojo_bt autonomous_maze.launch.py

# Save whatever SLAM has right now
ros2 run nav2_map_server map_saver_cli -f $PWD/maps/arena_slam3

# Mission examples (JSON on /dojo_mission)
#   GOAL      {"action":"GOAL","pose":{"x":3.0,"y":1.5}}
#   WAYPOINTS {"action":"WAYPOINTS","poses":[{"x":0.6,"y":1.6},{"x":3.0,"y":1.6}]}
#   CANCEL    {"action":"CANCEL"}
#   STATUS    {"action":"STATUS"}
ros2 topic pub --once /dojo_mission std_msgs/msg/String \
  'data: "{\"action\": \"GOAL\", \"pose\": {\"x\": 3.0, \"y\": 1.5}}'

# Monitoring
ros2 topic echo /dojo_mission/status --once
ros2 topic echo /dojo_mission/state  --once     # JSON
grep -E "Tilted|Level again|Wedge" <launch log>
```
