# Phase 3 — Nav2 autonomous navigation

End-to-end walkthrough for the Phase 3 navigation stack. Assumes Phase 2 is
working (see `instruction_2.md`) and the `phase-3-nav2` branch (or `main` after
the Phase 3 PR merges) is checked out in the container at
`~/workspace/ros2_ws/src/autonomousfleet-ai/`.

Everything below runs **inside the MentorPi Docker container** unless stated
otherwise. Drive tests are done in the mapped room with the robot on the
floor — keep a remote e-stop within reach (`stop` goal on the `/navigate_to_pose`
action client, or simply `Ctrl-C` the Nav2 bringup).

---

## 1. What Phase 3 adds

`af_navigation` — Nav2 configuration, behaviour trees, and launch composition
for the MentorPi M1 mecanum base:

| File | Purpose |
|---|---|
| `af_navigation/config/nav2_params.yaml` | Tuned parameters for controller, planner, costmaps, recoveries, waypoint follower, velocity smoother, and lifecycle manager |
| `af_navigation/behavior_trees/af_navigate_to_pose.xml` | Custom BT for single-goal navigation with cheapest-first recovery (clear costmaps → spin → backup → wait) |
| `af_navigation/behavior_trees/af_navigate_through_poses.xml` | Multi-waypoint variant with `RemovePassedGoals` |
| `af_navigation/launch/navigation.launch.py` | Brings up the full Nav2 stack (controller, planner, smoother, behaviors, bt_navigator, waypoint_follower, velocity_smoother) under `lifecycle_manager_navigation` |
| `af_bringup/launch/nav2.launch.py` | One-shot top-level launcher — robot + lidar + af_slam localisation + af_navigation |

**Controller**: `dwb_core::DWBLocalPlanner` with holonomic `min_vel_y` /
`max_vel_y` enabled (mecanum lateral motion). Conservative speed limits
(`max_vel_x = 0.20`, `max_vel_y = 0.15`, `max_vel_theta = 1.0`).

**Planner**: `nav2_smac_planner/SmacPlanner2D` with `MOORE` 8-connectivity —
natural for a holonomic robot.

**Costmaps**: global uses `static_layer + obstacle_layer + inflation_layer` at
5 cm resolution; local is a 3 m × 3 m rolling window at 3 cm resolution. Both
subscribe to `/scan`. The local costmap additionally has a `vision` slot on
`/vision_obstacles` (PointCloud2) — dormant until Phase 4 perception publishes
it. `clearing: false` on the vision source keeps LiDAR as the authoritative
clearer.

**Velocity smoother**: sits between the controller's `cmd_vel_nav` and the
motor driver's `/cmd_vel`, rate-limiting acceleration so the RRC Lite never
sees step changes.

---

## 2. Prerequisites

1. A saved map from Phase 2 — default is
   `~/workspace/ros2_ws/src/autonomousfleet-ai/af_slam/maps/room1.yaml`.
2. The factory lidar overlay must be sourced **before** launching:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/third_party_ros2/third_party_ws/install/local_setup.bash
   source /home/ubuntu/workspace/ros2_ws/install/local_setup.bash
   ```

   (These three lines belong in `~/.bashrc` — same rule as Phase 2.)
3. No stale bringup processes — see **§3 Clean slate**. This matters because
   ROS 2 launch children that outlive their parent `ros2 launch` become zombies
   parented to PID 1. They do not re-publish topics, but they clutter
   `ros2 node list` with duplicates and can confuse lifecycle transitions on a
   fresh bringup (the lifecycle manager picks up ghost nodes and hangs waiting
   for them to bond).

---

## 3. Clean slate

Before every bringup session, kill any orphaned HAL / Nav2 processes. The Pi 5
tends to accumulate them when a prior `ros2 launch` was killed with anything
other than a clean `Ctrl-C`.

```bash
# Inside the container, inspect first — only kill what's stale
ps -eo pid,ppid,state,comm | \
    grep -E 'odom_pub|hardware|ekf_node|complem|robot_state|joint_state|\
ros_robot|usb_cam|MS200|scan_sanit|nav2_|controller_serv|planner_serv|\
bt_navig|behavior_serv|waypoint|smoother_serv|velocity_smoo|amcl|map_server|\
lifecycle_manag'
```

Anything with state `Z` (zombie) is already dead — just needs its parent
reaped. Anything with state `S` parented to PID 1 is an orphan from a prior
launch; collect the PIDs and kill them:

```bash
pgrep -f 'odom_publisher_node|hardware_watchdog|imu_calib_node|\
complementary_filter|robot_state_publisher|joint_state_publisher|\
ros_robot_controller|scan_sanitizer|usb_cam_node|/MS200|nav2_|\
lifecycle_manager|controller_server|planner_server|smoother_server|\
behavior_server|bt_navigator|waypoint_follower|velocity_smoother|\
nav2_amcl|nav2_map' | xargs -r kill
```

Wait ~2 s, then re-check with `pgrep -af`. Any survivors get `kill -9`.

**`pkill` gotcha (same one as Phase 1)**: don't use `pkill -f <pattern>`
inside a `bash -lc '...'` wrapper — the wrapper's own command line contains
the pattern, so `pkill` kills the parent shell first. Always collect PIDs with
`pgrep`, inspect, then `kill` them explicitly.

---

## 4. One-shot Nav2 bringup

```bash
ros2 launch af_bringup nav2.launch.py \
    map:=$(ros2 pkg prefix af_slam)/share/af_slam/maps/room1.yaml
```

`nav2.launch.py` composes:

- `af_bringup/robot.launch.py` — HAL + description + EKF + complementary IMU
  filter + `lidar_frame → laser_frame` static TF + usb_cam
- `af_bringup/lidar.launch.py` — MS200 driver + `scan_sanitizer_node`
- `af_slam/slam_localization.launch.py` — `map_server` + `nav2_amcl` under
  `lifecycle_manager_localization`
- `af_navigation/navigation.launch.py` — the full Nav2 stack under
  `lifecycle_manager_navigation`

Launch args:

| Arg | Default | When to change |
|---|---|---|
| `map` | *(required)* | Any map YAML |
| `start_robot` | `true` | `false` if HAL is already running |
| `start_lidar` | `true` | `false` if the MS200 driver is already running |
| `use_sim_time` | `false` | `true` in Gazebo |

### Launch-time health checks

Wait ~15 s after bringup, then verify, in order:

1. **Lifecycle nodes all `active`**:

   ```bash
   for n in map_server amcl controller_server planner_server smoother_server \
            behavior_server bt_navigator waypoint_follower velocity_smoother; do
     printf "%-22s " "$n"
     ros2 lifecycle get /$n
   done
   ```

   All nine must print `active [3]`. If any say `inactive [2]`, the lifecycle
   manager has failed to transition — read the tail of the launch log for the
   offending node's error.

2. **TF chain complete**: `map → odom → base_footprint`:

   ```bash
   ros2 run tf2_ros tf2_echo map base_footprint
   ```

   Should print a pose within a few cycles. `Invalid frame ID "map"` means
   AMCL has not yet converged — give it a 2D Pose Estimate in RViz or check
   that `/scan` is flowing (`ros2 topic hz /scan` should be ~10 Hz).

3. **Costmaps alive**:

   ```bash
   ros2 topic hz /local_costmap/costmap   # ~2 Hz
   ros2 topic hz /global_costmap/costmap  # ~1 Hz
   ```

4. **`/cmd_vel` is the smoother output**, not the controller's raw output:

   ```bash
   ros2 topic info /cmd_vel
   # Publisher count: 1
   # Node: velocity_smoother
   ```

   `velocity_smoother` remaps `cmd_vel_smoothed → cmd_vel`, and
   `controller_server` is remapped to publish on `cmd_vel_nav`. If `/cmd_vel`
   has two publishers, the remap failed and the robot will see stepped
   velocities.

Only after all four checks pass is the stack safe to send goals to.

---

## 5. Sending goals

Two options:

### RViz (Dev PC)

Use the Phase 2 RViz config as a starting point — it already has the correct
fixed frame and tools. Add a **Nav2 Goal** tool and click on the map to send
`NavigateToPose` goals.

### CLI

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 1.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

For multi-waypoint patrols use `/follow_waypoints` with a `poses[]` array.

---

## 6. Acceptance tests

Six tests from the implementation plan. Run them in order; each depends on
the previous one working. Log results in
`logs/phase3/validation_session_<date>.md`.

### Test 1 — Point-to-point navigation accuracy

**Pass criterion**: robot reaches a goal pose within 10 cm / 0.15 rad.

1. Start a rosbag:
   ```bash
   ros2 bag record -o phase3_test1 /tf /tf_static /scan /odom /amcl_pose \
       /cmd_vel /plan /local_costmap/costmap /global_costmap/costmap \
       /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status
   ```
2. In RViz, place the robot at the room origin.
3. Send a 1.0 m × 0.0 m forward goal. Verify the robot reaches it
   (read `/amcl_pose` at goal-reached).
4. Send 0.0 m × 1.0 m — **lateral** motion, this proves the mecanum holonomic
   config is live. Watch `cmd_vel.linear.y` go non-zero on the bag.
5. Send a 45°-rotated goal (x=0.7, y=0.7). Verify the robot takes a direct
   path (SmacPlanner MOORE).

### Test 2 — Multi-waypoint patrol

**Pass criterion**: 5 waypoints visited in sequence, `follow_waypoints` action
reports SUCCESS.

1. Pick 5 points around the mapped room, at least 0.5 m apart.
2. Fire the action via CLI (`ros2 action send_goal /follow_waypoints …`) or
   the RViz **Waypoint Navigation** mode.
3. Observe that `RemovePassedGoals` drops completed waypoints (check the
   `/plan` topic — it should shorten after each waypoint).

### Test 3 — LiDAR obstacle avoidance

**Pass criterion**: robot re-routes around a novel obstacle placed in its path.

1. Send a goal 2 m in front of the robot, clear line of sight.
2. While it is moving, drop a chair or box directly on its planned path.
3. The local costmap should mark the obstruction within one planner cycle
   (~1 s); the controller should steer around it. If the obstacle fully
   blocks the path, the planner replans.
4. Verify by watching `/local_costmap/costmap` in RViz — the obstacle appears
   as a high-cost lethal cell, and `/plan` detours around it.

### Test 4 — Recovery behaviour trigger

**Pass criterion**: stuck condition triggers the recovery subtree, and at
least one recovery action (spin / backup / wait) executes.

1. Box the robot in with obstacles on three sides while it tries to move.
2. `bt_navigator` will run `ComputePathToPose` retries, then drop into the
   `RecoveryFallback` subtree. Expect to see, in order:
   - `ClearEntireCostmap` on local and global
   - `Spin` (1.57 rad — 90°)
   - `BackUp` (0.30 m at 0.05 m/s)
   - `Wait` (5 s)
3. `ros2 topic echo /behavior_server/transition_event` shows the active
   recovery action.

### Test 5 — Sustained 0.2 m/s navigation

**Pass criterion**: 3 minutes of continuous driving at the configured max
speed without crashes or stalls.

1. Kick off a looping waypoint patrol between 4 corners of the room.
2. Watch `ros2 topic hz /cmd_vel` — it should sit at 10 Hz continuously.
3. `ros2 topic echo /diagnostics` should stay green (Phase 1's
   `hardware_watchdog` logs `OK` at 1 Hz).
4. Pass condition: 3 minutes elapsed with no dropped goals and no
   `TF_TIMEOUT` errors in the launch log.

### Test 6 — CPU / RAM budget

**Pass criterion**: Pi 5 CPU <80 %, RAM <3 GB during the patrol test.

1. In a separate container shell:
   ```bash
   top -b -d 1 -n 180 | grep -E 'controller_server|planner_server|\
bt_navigator|amcl|costmap_2d|af_hal|slam_toolbox' > /tmp/phase3_top.log
   ```
2. After Test 5, compute max CPU% per process and total memory usage.
3. If >80 % total, drop `controller_frequency` from 10 Hz to 5 Hz in
   `nav2_params.yaml` and rerun.

---

## 7. Topics and TF reference

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `scan_sanitizer_node` | AMCL, Nav2 costmaps |
| `/odom` | `nav_msgs/Odometry` | `ekf_filter_node` | controller_server, AMCL motion model |
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` | AMCL, global_costmap |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `amcl` | — (observe) |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | `controller_server` | `velocity_smoother` |
| `/cmd_vel` | `geometry_msgs/Twist` | `velocity_smoother` | `ros_robot_controller` (motors) |
| `/plan` | `nav_msgs/Path` | `planner_server` | — (observe) |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | `local_costmap` | — (observe) |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | `global_costmap` | — (observe) |
| `/navigate_to_pose` | action | `bt_navigator` | user / mission_manager |
| `/follow_waypoints` | action | `waypoint_follower` | user / mission_manager |

TF chain: `map → odom → base_footprint → base_link → {lidar_frame, imu_link,
depth_cam, wheel_*}`. AMCL publishes `map → odom`, EKF publishes
`odom → base_footprint`, `robot_state_publisher` emits the rest from the URDF.

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'oradar_lidar' not found` at launch | Factory overlay not sourced | `source /home/ubuntu/third_party_ros2/third_party_ws/install/local_setup.bash` |
| Nav2 lifecycle stuck at `inactive` | TF chain incomplete (usually no `map → odom`) | Check AMCL is receiving `/scan` and has an initial pose |
| `Message Filter dropping message: frame 'laser_frame'` (AMCL log) | `odom → base_footprint` missing → AMCL can't transform scans | Verify `ekf_filter_node` is alive and `/odom` is publishing |
| Controller publishes, robot doesn't move | `/cmd_vel` has two publishers (remap failed) or `velocity_smoother` inactive | Re-check `ros2 topic info /cmd_vel` — expect one publisher (`velocity_smoother`) |
| Planner says `Failed to compute path` | Start or goal on a lethal cell (in inflation radius) | Nudge the goal out of the inflation zone or reduce `inflation_radius` |
| Robot circles the goal without converging | `RotateToGoal.scale` or `GoalAlign.scale` too low for mecanum | Raise both in `nav2_params.yaml` (rebuild the overlay, relaunch) |
| Nav2 OK in sim, unstable on robot | `controller_frequency` too high for Pi 5 under full load | Drop to 5 Hz |
| Ghost lifecycle node in `ros2 lifecycle list` | Orphan from a previous launch | See **§3 Clean slate** |

---

## 9. Reporting back

Capture a session rosbag during each acceptance test (see Test 1) and log a
summary in `logs/phase3/validation_session_<date>.md` with:

- Timestamp and map used
- For each test: pass / fail, measured values (goal error, CPU%, recovery
  actions fired, waypoints completed)
- Any parameter tweaks made to `nav2_params.yaml` with justification
- Rosbag paths

Attach screenshots of (a) a planned path + local costmap during Test 3, and
(b) the recovery subtree firing during Test 4.

---

## 10. Teardown

```bash
# In the launch shell
Ctrl-C

# Verify nothing orphaned
pgrep -af 'nav2_|controller_server|planner_server|bt_navigator|\
velocity_smoother|behavior_server|waypoint_follower'
# → should print only the grep itself

# If anything survives, see §3 Clean slate.
```
