# Bot testing instructions — base reference

Consolidated how-to + known-issue catalogue for bringing up, testing, and
tearing down the MentorPi M1 across every phase. Any recurring error I hit in
Phases 1–3 is captured here so future sessions don't re-derive the same fix.

Per-phase walkthroughs (`setup/instruction_<N>.md`) cover what each phase
adds; this file is the common substrate — connect, source, launch, diagnose,
kill, repeat.

---

## 1. Reaching the robot

### 1.1 Passwordless SSH → Docker exec

The robot lives at `pi@192.168.149.1` on the HiWonder AP hotspot (AP mode). ROS 2 runs only
inside the `MentorPi` Docker container. The canonical one-liner from the Dev
PC:

```bash
ssh pi@192.168.149.1 'docker exec -u ubuntu -w /home/ubuntu MentorPi \
    bash -lc "source /opt/ros/humble/setup.bash && \
    source /home/ubuntu/third_party_ros2/third_party_ws/install/local_setup.bash && \
    source /home/ubuntu/workspace/ros2_ws/install/local_setup.bash && \
    <ros2 command>"'
```

Use this for `ros2 topic hz`, one-off test drivers, rosbag starts, etc. For
long-running launches, `docker cp` a script into the container and run it with
`nohup … > /tmp/af_logs/*.log 2>&1 & disown`.

### 1.2 VNC fallback

Only needed for first-boot Wi-Fi configuration (AP → STA switch) or when SSH
is unreachable. RealVNC Viewer → `<robot-ip>:5900`, auth `pi` / `raspberrypi`
→ click the LXTerminal desktop icon. Full procedure in
`setup/instruction_1.md` §1–2.

### 1.3 Dev PC ↔ container file transfer

The container mount `/home/ubuntu/shared` is bind-mounted from the host at
`/home/pi/docker/tmp`. Drop files into `/home/pi/docker/tmp` on the host (or
rsync to it over SSH), then read them from `/home/ubuntu/shared` inside the
container. This is the path to use when an SSH heredoc keeps eating your
Python quoting — write the script on the Dev PC, sync, then run.

### 1.4 Syncing the Pi repo to a new branch

When a new branch lands on GitHub that the Pi needs (e.g. `phase-N-foo`),
never blow away the working tree with `git checkout -- .` or `git reset
--hard` — uncommitted Pi-side edits from a live debug session can
disappear. Safe recipe:

```bash
# Inside /home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai
git stash push -u -m "pre-<branch>-sync backup $(date +%F-%H%M)"
git fetch origin
git checkout <branch>
git branch -D <stale-local-branch>   # only after confirming nothing was lost
```

Any pre-existing uncommitted Pi edits are then recoverable with `git stash
list` → `git stash show -p stash@{0}` → `git stash pop` (or `apply`). Stashes
persist until explicitly dropped, so even a stash made days ago is still
inspectable. This matters because the Pi is a separate checkout from the
Dev PC, and a fast-moving session may have in-situ fixes that were not yet
pushed.

---

## 2. Sourcing overlays — the rule that burns every session

The container has **three** stacked ROS 2 workspaces. They must be sourced in
this order, and you must use `local_setup.bash`, **not** `setup.bash`, for
the second and third:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/local_setup.bash
source /home/ubuntu/workspace/ros2_ws/install/local_setup.bash
```

| Overlay | Contents |
|---|---|
| `/opt/ros/humble` | ROS 2 Humble base install |
| `third_party_ros2/third_party_ws` | Factory packages — `oradar_lidar`, `peripherals`, `ros_robot_controller`, etc. |
| `workspace/ros2_ws` | Our overlay — `af_bringup`, `af_hal`, `af_slam`, `af_navigation`, … |

**Why `local_setup.bash`**: the plain `setup.bash` rewrites the `AMENT_PREFIX_PATH`
chain and drops whichever overlay was sourced before it. Layering with
`local_setup.bash` appends to the chain instead. Symptom of getting this
wrong: `Package 'oradar_lidar' not found` or `Package 'af_bringup' not found`
from a launch file, even though both packages clearly built.

Put all three lines in `~/.bashrc` inside the container so every new
`docker exec` shell just works.

---

## 3. Clean slate — the most important step, every session

**Always** reap stale processes before `ros2 launch`. Phase 1 and Phase 2
both burned hours on ghost-process symptoms before this became routine.

### 3.1 What goes wrong if you skip it

- `ros2 topic hz /odom` shows 3× the expected rate (duplicate publishers
  layered over each other).
- `/scan` at 20 Hz with readings wildly variable (13–396 range samples per
  message) — two MS200 driver instances fighting for `/dev/ldlidar`.
- `lifecycle_manager` hangs at `inactive` waiting for a ghost node to bond.
- slam_toolbox rejects every scan because a second driver's jitter doesn't
  match the first driver's latched `LaserRangeFinder` geometry.
- AMCL converges to a bogus pose because it's hearing two `/scan` streams.

### 3.2 `pkill` gotchas — the two that bite

**Gotcha 1 — 15-character `comm` truncation.** Linux truncates
`/proc/<pid>/comm` to 15 characters, so `pkill odom_publisher_node` (19 chars)
silently matches nothing. Every restart layers a new process on top of the
old ones and topic rates come out 2× or 3× inflated. Kill by 15-char comm
prefixes instead:

```bash
pkill -KILL ros_robot_contr    # ros_robot_controller_node
pkill -KILL odom_publisher_    # odom_publisher_node
pkill -KILL complementary_f    # complementary_filter_gain_node
pkill -KILL imu_calib_node     # imu_calib_node
pkill -KILL hardware_watchd    # hardware_watchdog_node
pkill -KILL usb_cam_node_ex    # usb_cam_node_exe
pkill -KILL ekf_node           # ekf_node (already ≤15 chars)
pkill -KILL scan_sanitizer     # scan_sanitizer_node
```

**Gotcha 2 — `pkill -f` kills its own parent shell.** Do **not** put
`pkill -f <pattern>` inside a `bash -lc '...'` wrapper if `<pattern>` matches
anything in the wrapper's own argv. `pkill -f` matches the full command line,
so it kills the wrapper first and exits with 137 before the real targets are
touched. Either:

- Write the kill commands to a script file (`/tmp/af_kill.sh`) and run it via
  `bash /tmp/af_kill.sh` — the patterns only appear in the script's argv,
  never in a live command line; **or**
- Use `pgrep -f <pattern> | xargs -r kill` to collect PIDs first, inspect
  the list, then kill explicitly.

### 3.3 Inspecting before killing

```bash
# See everything we might care about
ps -eo pid,ppid,state,comm | grep -E \
  'odom_pub|hardware|ekf_node|complem|robot_state|joint_state|ros_robot|\
usb_cam|MS200|scan_sanit|nav2_|controller_serv|planner_serv|bt_navig|\
behavior_serv|waypoint|smoother_serv|velocity_smoo|amcl|map_server|\
lifecycle_manag|slam_toolbox'
```

- State `Z` (zombie) → already dead, needs its parent reaped.
- State `S` parented to PID 1 → orphan from a prior `ros2 launch` that was
  killed with anything other than clean `Ctrl-C`. Collect PIDs with `pgrep`
  and `kill` them.
- Anything alive from a previous session → kill it.

### 3.4 The one-shot nuke

```bash
bash /tmp/af_kill.sh    # script form of the 15-char prefix list above
sleep 2
ros2 daemon stop && ros2 daemon start
ros2 node list          # must be empty or only /rosout
```

Only after `ros2 node list` comes back clean is the stack safe to launch.

---

## 4. Launching a phase

Pick the right launch file for the phase under test:

| Phase | Launch | Notes |
|---|---|---|
| 1 | `ros2 launch af_bringup robot.launch.py` | HAL + description + EKF + usb_cam + `lidar_frame → laser_frame` static TF |
| 2 mapping | `ros2 launch af_bringup slam.launch.py` | + MS200 + `scan_sanitizer` + slam_toolbox |
| 2 localisation | `ros2 launch af_bringup slam.launch.py mode:=localization map:=<path>.yaml` | + map_server + AMCL |
| 3 | `ros2 launch af_bringup nav2.launch.py map:=<path>.yaml` | Full Nav2 stack under `lifecycle_manager_navigation` |
| 5a/5b (Pi) | `./af_bringup/scripts/start_pi.sh` | Full stack via SSH: HAL+SLAM+Nav2+mission+perception+explorer. Kills stale procs, sources all 3 workspaces, waits 35s, verifies 4 critical nodes |
| 5a/5b (Pi) | `./af_bringup/scripts/start_pi.sh enable_explore:=false` | Same but without auto-explorer — use for NLP-controlled missions |
| 5b (Dev PC) | `./af_bringup/scripts/start_nlp.sh` | NLP node only (Ollama LLM translator). Requires Ollama running locally |
| 5b (Dev PC) | `./af_bringup/scripts/start_nlp.sh --with-cli` | NLP node + opens interactive CLI in a second terminal |
| teardown | `./af_bringup/scripts/stop_pi.sh` | Kills all ROS/Python inside the Pi container, reports remaining processes |

Launch args worth knowing:

| Arg | Default | When to override |
|---|---|---|
| `map` | required on Phase 2 loc / Phase 3 | Any saved `.yaml` |
| `start_robot` | `true` | `false` if HAL already running |
| `start_lidar` | `true` | `false` if MS200 driver already running |
| `use_sim_time` | `false` | `true` in Gazebo |
| `enable_explore` | `true` | `false` for NLP-controlled missions (no auto-explorer) |
| `enable_perception` | `true` | `false` to skip YOLO (lighter CPU) |

### 4.1 Post-launch health checks (wait ~15 s, then run in order)

**Step 1 — Nodes up.** `ros2 node list` should show the expected set with
no duplicates. On Phase 3 that's HAL nodes + `/map_server`, `/amcl`,
`/controller_server`, `/planner_server`, `/bt_navigator`,
`/behavior_server`, `/smoother_server`, `/waypoint_follower`,
`/velocity_smoother`, `/lifecycle_manager_navigation`,
`/lifecycle_manager_localization`.

**Step 2 — Lifecycle nodes active (Phase 2/3).**

```bash
for n in map_server amcl controller_server planner_server smoother_server \
         behavior_server bt_navigator waypoint_follower velocity_smoother; do
  printf "%-22s " "$n"
  ros2 lifecycle get /$n
done
```

Every line must read `active [3]`. `inactive [2]` means the lifecycle
manager failed to transition that node — read the launch log tail for the
offending node's error.

**Step 3 — TF chain complete.**

```bash
ros2 run tf2_ros tf2_echo map base_footprint   # Phase 2+
ros2 run tf2_ros tf2_echo odom base_footprint  # Phase 1
```

`Invalid frame ID "map"` from Phase 2+ = AMCL hasn't converged yet → drop a
2D Pose Estimate in RViz, or verify `/scan` is actually flowing.

**Step 4 — Topic rates sane.**

| Topic | Expected | Publisher |
|---|---|---|
| `/scan` | ~10 Hz | `scan_sanitizer_node` (fixed 450-bin grid) |
| `/odom` | ~49 Hz | `ekf_filter_node` |
| `/odom_raw` | ~50 Hz | `odom_publisher_node` |
| `/imu` | ~48 Hz | `complementary_filter_gain_node` |
| `/diagnostics` | ~2 Hz | `hardware_watchdog_node` |
| `/image_raw` | ~11 Hz raw yuyv | `usb_cam` |
| `/image_raw/compressed` | ~16 Hz | `usb_cam` image_transport |
| `/local_costmap/costmap` | ~2 Hz | Phase 3 only |
| `/global_costmap/costmap` | ~1 Hz | Phase 3 only |
| `/map` | ~0.2 Hz mapping mode | slam_toolbox |
| `/cmd_vel` | ~20 Hz when driving | `velocity_smoother` (Phase 3) |

Rates that are a clean 2×/3× of these = duplicate processes → §3.

**Step 5 — `/cmd_vel` has exactly one publisher (Phase 3).**

```bash
ros2 topic info /cmd_vel
# Publisher count: 1
# Node name: /velocity_smoother
```

Two publishers = `velocity_smoother`'s `cmd_vel_smoothed → cmd_vel` remap
failed and the `controller_server` is also driving `/cmd_vel` directly. The
robot will see stepped velocities. Relaunch cleanly.

---

## 5. Recurring issues catalogue

Canonical index of everything that has bitten me across phases. First column
is the symptom you actually see in a log or topic echo.

### 5.1 Connection / overlay

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'oradar_lidar' not found` at launch | Factory `third_party_ws` overlay not sourced | Source it (use `local_setup.bash`, not `setup.bash`) — see §2 |
| `Package 'af_bringup' not found` after sourcing third_party | Used `setup.bash` for third_party → it clobbered the prefix chain | Re-source with `local_setup.bash` for both overlays |
| `ros2: command not found` on the Pi host (outside container) | ROS 2 only lives inside the `MentorPi` container | Enter the container first |
| `apt update` prints `EXPKEYSIG … Open Robotics` | Factory image ROS 2 apt signing key is expired | `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg` |
| `ldconfig: File /lib/aarch64-linux-gnu/libcudnn*.so … is empty` during apt | Broken Jetson symlinks left over on the Pi 5 image | Harmless — ignore |

### 5.2 Clean-slate / process hygiene

| Symptom | Cause | Fix |
|---|---|---|
| `/odom` or `/imu` at 2×/3× expected rate | Duplicate HAL processes from failed `pkill` | `bash /tmp/af_kill.sh` (uses 15-char comm prefixes), verify with `ros2 node list` |
| `/scan` at 20 Hz with ranges 13–396 | Two MS200 driver instances fighting for `/dev/ldlidar` | Same fix — `bash /tmp/af_kill.sh` before every launch |
| `pkill … `-f …`` run inside `bash -lc` silently kills the shell (exit 137) | `pkill -f` matches the wrapper's own argv | Put the kill in a script file and `bash /tmp/af_kill.sh` it, or use `pgrep -f … | xargs kill` |
| `ros2 node list` shows three `/odom_publisher` nodes | Previous `pkill odom_publisher_node` (19 chars) silently no-op'd (Linux truncates `/proc/<pid>/comm` to 15 chars) | Use 15-char prefixes: `pkill -KILL odom_publisher_` |
| Ghost lifecycle node in `ros2 lifecycle list` | Orphan from a previous launch, parented to PID 1 | §3 — inspect with `ps -eo pid,ppid,state,comm`, kill the S-state orphans |

### 5.3 Bringup / HAL (Phase 1)

| Symptom | Cause | Fix |
|---|---|---|
| `usb_cam` aborts ~7 s in with `Invalid v4l2 format` | `pixel_format: yuyv2rgb` hits `usb_cam 0.6.x` swscaler bug | Use raw `yuyv` in the launch; Phase 4 handles YUY2→RGB on the Dev PC |
| `SerialException` from `ros_robot_controller` | STM32 on wrong port | `ls /dev/ttyUSB*`; override with `--ros-args -p port:=/dev/ttyUSB1` |
| `Could not open /dev/video0` | Camera held by factory `start_node.service` | `sudo systemctl stop start_node.service && sudo systemctl disable start_node.service` (host, not container) |
| `/diagnostics` EKF ERROR `odometry/filtered topic status — No events recorded` | EKF's internal diagnostic watches the unremapped name but the launch remaps `odometry/filtered → /odom` | Cosmetic — `/odom` still publishes fine |
| Battery reports 7000 on `/ros_robot_controller/battery` | RRC Lite reports raw `UInt16` in **millivolts**, and the shipped pack is 2S 7.4 V (not 3S) | `hardware_watchdog_node` scales `msg.data/1000.0`; defaults `warn=6.8 V`, `error=6.4 V` |
| Forward 1 m test overshoots by 5–10 % with `teleop_twist_keyboard` | Key-repeat timing jitter | Drive with an in-process `rclpy` publisher at 20 Hz × 10 s × 0.1 m/s |
| Odometry invisible to stalls / slippage | `odom_publisher_node` integrates commanded velocity, not encoder feedback | Known open-loop limitation; laser/AMCL corrects in Phase 2+ |
| Square closure drifts ~13 cm on tile | 1.0 rad/s yaw command under-rotates to ~0.91 rad/s on smooth tile | AMCL corrects at map level in Phase 3; or try 1.5–2.0 rad/s |

### 5.4 LiDAR / scan path (Phases 1–3)

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` shows `/scan_raw`, not `/scan` | Factory driver log line `ROS topic:MS200/scan` is misleading — it actually publishes on `/scan_raw` | Phase 1: `ros2 run topic_tools relay /scan_raw /scan`. Phase 2+: `scan_sanitizer_node` does this automatically |
| `tf filter` drops scans with `frame_id: laser_frame` | MS200 tags scans `laser_frame`, URDF link is `lidar_frame` | `af_bringup` runs a static TF bridge `0 0 0 0 0 0 lidar_frame laser_frame` |
| slam_toolbox floods `LaserRangeScan contains 449 readings, expected 450` | Karto latches `LaserRangeFinder` geometry from the first scan; MS200 driver jitters between 447–451 samples per message | `scan_sanitizer_node` resamples onto a canonical 450-bin grid — launched automatically by `af_bringup/lidar.launch.py` |
| `/scan` at ~7 Hz, readings are only a 316° arc per message | Factory `ms200_scan.launch.py` emits partial scan chunks, not full 360° accumulations | Known behaviour; sanitizer gives Karto fixed geometry. Do **not** reintroduce rf2o — it can't handle partial arcs and outputs sign-flipped deltas |
| `peripherals lidar.launch.py` crashes with `KeyError: 'need_compile'` | That launch reads env vars set by factory startup scripts we stop | Use `peripherals ms200_scan.launch.py` directly (or the `af_bringup/lidar.launch.py` wrapper) |

### 5.5 SLAM / AMCL (Phases 2–3)

| Symptom | Cause | Fix |
|---|---|---|
| `/scan` at 10 Hz but `/map` never updates and no `map → odom` TF | slam_toolbox rejecting scans (see 5.4) | `ros2 node list \| grep scan_sanitizer` — if missing, relaunch via `slam.launch.py` |
| AMCL converges to symmetric wrong pose (180° off) in a square room | Initial pose too far off | Click **2D Pose Estimate** again with a better guess and drive a short distance |
| `Invalid frame ID "map" passed to canTransform` running `tf2_echo map odom` | AMCL hasn't published `map → odom` yet | Wait 2–3 s after launch; if it persists, check for rejected-scan errors |
| `amcl: Failed to transform initial pose in time (Lookup would require extrapolation into the future)` | RViz stamps initial-pose with Dev-PC wall time, slightly ahead of the robot's TF cache | Benign — AMCL still accepts the pose; TF converges once the robot moves |
| Planner: `Starting point in lethal space! Cannot create feasible plan` | Robot sitting on (or inflated onto) a lethal costmap cell. Two sub-causes: (a) `inflation_radius` too big for a small room; (b) stale map feature (e.g. captured carpet edge) matches onto AMCL's current pose | Lower `inflation_radius` (0.15 m was the Phase 3 setting); re-map the room if the static map is wrong |
| Map looks right but has ghost walls where the carpet used to be | slam_toolbox captured a carpet edge as a wall in a prior mapping run | Re-map: `ros2 launch af_bringup slam.launch.py`, teleop a loop ≤0.15 m/s, `ros2 run nav2_map_server map_saver_cli -f <path>/<name> --ros-args -p save_map_timeout:=5.0` |

### 5.6 Nav2 (Phase 3)

| Symptom | Cause | Fix |
|---|---|---|
| `Message Filter dropping message: frame 'laser_frame'` in AMCL log | `odom → base_footprint` TF missing → AMCL can't transform scans | Verify `ekf_filter_node` is alive and `/odom` publishing |
| Controller publishes `/cmd_vel_nav`, robot doesn't move | `/cmd_vel` has two publishers (remap failed) OR `velocity_smoother` inactive | `ros2 topic info /cmd_vel` — expect a single publisher `/velocity_smoother`; relaunch cleanly if not |
| Lateral goal makes no progress, controller logs `Failed to make progress`, BT runs `ClearLocalCostmap` | DWB critic set includes `RotateToGoal` / `GoalAlign` / `PathAlign` — heading-biased critics dominate scoring and block strafe | Drop those three critics (they're wrong for mecanum), raise `vy_samples` to 15. See `af_navigation/config/nav2_params.yaml` `FollowPath:` |
| Robot circles the goal without converging | Opposite problem — all alignment critics removed and the goal-align bias is now too weak | Tune `GoalDist.scale` up; this was `24 → 32` in the Phase 3 retune |
| `controller_server` sitting at 100 % of one core | DWB with 10×15×15 sample grid is CPU-bound; expected shape on Pi 5 | OK if system `%Cpu(s)` < 80 %. If Phase 4 needs more headroom: drop `vx_samples`, `vy_samples`, `vtheta_samples`, or drop `controller_frequency` 10 → 5 Hz |
| `bt_navigator` dies with `rclcpp::exceptions::RCLError: client will not receive response, at ./src/rmw_response.cpp:154` (exit −6) | Known Nav2 Humble rmw interaction — action result dispatched to a client whose context has already shut down. Usually triggered by aborted-goal cancellations under load | **Not a regression** — clean relaunch. Watch for this after goal completion. Candidate mitigations for later: bump `default_server_timeout`, try Cyclone↔Fast DDS swap, or rebuild against a newer Nav2 patch |
| `BackUp` recovery aborts with `Collision Ahead - Exiting DriveOnHeading` | Desired behaviour — the obstacle collision check is vetoing the reverse motion into a known wall | Not a fix; the BT will move on to `Wait` next |
| AMCL pose jumps ~1 m between goal-issue and goal-abort | BT `BackUp` (0.30 m at 0.05 m/s, called multiple times) physically moved the robot during failed recovery loops | Fix the upstream abort cause (usually the lethal-space issue above); re-set 2D Pose Estimate in RViz if AMCL is lost |
| `ros2 topic hz /cmd_vel` reports ~18–20 Hz, but walkthrough said 10 Hz | The `/cmd_vel` publisher is `velocity_smoother` (`smoothing_frequency: 20 Hz`), not `controller_server` (10 Hz) | 18–20 Hz is correct |

### 5.7 Dev PC side (RViz, teleop, DDS)

| Symptom | Cause | Fix |
|---|---|---|
| rviz2 crashes on `libpthread.so.0: undefined symbol: __libc_pthread_init` | Launched from a VS Code integrated terminal — VS Code snap env (`GTK_PATH`, `LOCPATH`) leaks in and pulls snap libs | Scrub env: `env -i HOME=$HOME USER=$USER DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY PATH=/opt/ros/humble/bin:/usr/bin:/bin bash -c 'source /opt/ros/humble/setup.bash && rviz2 -d <config>.rviz'` |
| RViz Image display shows "No Image" with no error, raw transport works | `usb_cam` publishes compressed frames as `yuv422_yuy2; jpeg compressed mono8`, a non-standard variant rviz's compressed transport can't decode | Use raw `/image_raw` transport (Phase 1 config already does this); Phase 4 does explicit YUY2 decode on Dev PC |
| LaserScan not appearing in RViz | MS200 driver publishes with `BEST_EFFORT` QoS; default RViz display is `RELIABLE` | Set Reliability Policy → Best Effort on the LaserScan display. Provided configs already do this |
| `ros2 topic list` empty from Dev PC | DDS discovery not traversing the AP, or domain ID mismatch | `export ROS_DOMAIN_ID=0` (match container's); on corporate LANs set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both ends and restart |
| Goals/initial poses not landing on robot | Same DDS issue, or the Dev PC and robot are on different subnets | Ping the robot, check `ros2 node list` matches across both machines |

### 5.8 Build / colcon

| Symptom | Cause | Fix |
|---|---|---|
| `colcon build` OOM-killed on the Pi | Pi 5 4 GB runs out of RAM with default job count | `MAKEFLAGS="-j2" colcon build --packages-select <pkgs> --symlink-install` |
| `setup.py install is deprecated` / `easy_install is deprecated` stderr during build | Standard ament_python deprecation warnings on Humble | Not an error — look for `Summary: N packages finished` |
| Edited a Python node and the change isn't picked up | Built without `--symlink-install` | Rebuild with `--symlink-install`; for already-installed pkgs, re-source `install/setup.bash` |

---

## 6. Common command recipes

### 6.1 Send a Nav2 goal from the CLI

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 1.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}" --feedback
```

For multi-waypoint patrols, use `/follow_waypoints` with a `poses[]` array.
Generate long waypoint lists with a local Python script and rsync the
resulting YAML via the shared mount — do **not** try to build the YAML
through SSH heredoc quoting.

### 6.2 Save a map

```bash
ros2 run nav2_map_server map_saver_cli \
    -f /home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai/af_slam/maps/<name> \
    --ros-args -p save_map_timeout:=5.0
```

Produces `<name>.pgm` + `<name>.yaml`. Sync back to the Dev PC repo via the
shared mount.

### 6.3 Record a session rosbag

```bash
# Phase 1
ros2 bag record -o phase1_validation /scan /odom /odom_raw /imu /cmd_vel \
    /diagnostics /tf /tf_static /image_raw/compressed /ros_robot_controller/battery

# Phase 2
ros2 bag record -o phase2_validation /scan /scan_raw /odom /odom_raw /imu \
    /cmd_vel /map /map_metadata /amcl_pose /particle_cloud /initialpose \
    /diagnostics /tf /tf_static

# Phase 3
ros2 bag record -o phase3_validation /tf /tf_static /scan /odom /amcl_pose \
    /cmd_vel /plan /local_costmap/costmap /global_costmap/costmap \
    /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status
```

### 6.4 CPU / RAM snapshot during a patrol

```bash
top -b -d 1 -n 200 > /tmp/phase3_top_full.log
```

Grep the log for the process set afterwards; peak `%Cpu(s)` and peak
`MiB Mem ... used` are the budget numbers.

### 6.5 In-process motion test publisher

For timing-sensitive odometry tests, don't use `teleop_twist_keyboard` or
`ros2 topic pub --rate`. Use a short `rclpy` script with a 20 Hz timer that
counts messages and stops after exactly N ticks:

```python
# /tmp/straight_test.py — 10 s forward at 0.1 m/s
import rclpy
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node('straight_test')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
msg = Twist(); msg.linear.x = 0.1
count = [0]
def tick():
    pub.publish(msg); count[0] += 1
    if count[0] >= 200:  # 10 s at 20 Hz
        msg.linear.x = 0.0; pub.publish(msg); rclpy.shutdown()
node.create_timer(0.05, tick)
rclpy.spin(node)
```

Phase 1 validation showed this closes to within 2–4 % of commanded distance
vs 5–10 % for keyboard teleop.

---

## 7. Teardown

```bash
# In the launch shell:
Ctrl-C

# Verify nothing orphaned:
pgrep -af 'nav2_|controller_server|planner_server|bt_navigator|\
velocity_smoother|behavior_server|waypoint_follower|scan_sanitizer|\
odom_publisher|hardware_watchdog|ros_robot_controller|usb_cam|amcl|\
map_server|ekf_node'
# → should print only the grep itself

# If anything survives:
bash /tmp/af_kill.sh
sleep 2
ros2 daemon stop && ros2 daemon start
ros2 node list    # must be empty or only /rosout
```

Then `exit` the container shell and power the robot off via the chassis
switch — do not yank the battery, let filesystems flush.

---

## 8. Phase 5b — NLP mission control workflow

### 8.1 Prerequisites

- Ollama installed and running on the Dev PC: `ollama serve`
- Model pulled: `ollama pull qwen2.5:7b-instruct-q4_K_M`
- Robot stack already running on the Pi (via `start_pi.sh`)

### 8.2 Full NLP session (Pi + Dev PC)

```bash
# 1. Bring up the Pi stack (NLP-controlled, no auto-explorer)
cd af_bringup/scripts
./start_pi.sh enable_explore:=false
# Wait for "Robot stack is ready" and all 4 ✓ checks

# 2. Launch NLP on Dev PC (with interactive CLI)
./start_nlp.sh --with-cli

# 3. Send commands in the CLI terminal:
#   > find the suitcase
#   > go to the door
#   > move forward for 5 seconds
#   > stop
#   > explore this room

# 4. Teardown
./stop_pi.sh
# Ctrl-C the NLP node on Dev PC
```

### 8.3 NLP command types

| Command type | Example phrases | What it does |
|---|---|---|
| `find_object` | "find the suitcase", "look for a chair" | Triggers FindObject action (explore + YOLO + return home) |
| `navigate_to` | "go to the door", "move to corner_a" | Nav2 goal to named room location (from `rooms.yaml`) or coordinates |
| `patrol` | "patrol the room", "visit door and far_wall" | FollowWaypoints through listed locations |
| `drive_for` | "move forward for 5 seconds", "go left for 3 seconds" | Open-loop cmd_vel for specified duration and direction |
| `stop` | "stop", "halt" | Zeros cmd_vel, cancels active Nav2 goals and drive timers |
| `return_home` | "come back", "go home" | Nav2 goal to (0,0) |
| `set_speed` | "go faster", "set speed to 0.2" | Adjusts speed parameter (runtime reconfigure pending) |
| `scan_area` | "explore", "scan the area", "look around" | Triggers exploration without object-detection stop condition |

### 8.4 NLP troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ERROR: Ollama is not running` from `start_nlp.sh` | Ollama server not started | `ollama serve` in a separate terminal |
| First command takes 30–60s | Ollama cold-loading the model into RAM | Normal on first call; subsequent calls ~2–3s |
| LLM returns text instead of tool call | Qwen2.5 occasionally ignores tool-calling schema | Retry mechanism handles this automatically (up to 2 retries + keyword fallback) |
| `_processing` flag stuck, no new commands accepted | Previous Ollama call crashed without clearing the guard | Restart `nlp_command_node` |
| Robot auto-exploring when NLP should control | `enable_explore:=false` not passed to `start_pi.sh` | Relaunch: `./stop_pi.sh && ./start_pi.sh enable_explore:=false` |
| CLI shows flood of `[CMD]` messages | Stale `nlp_command_node` processes from previous sessions | Kill all: `pkill -9 -f nlp_command` then relaunch |

---

## 9. When something new breaks

1. Check this file first — symptom tables in §5 and §8.4 cover every
   recurring failure mode across Phases 1–5b.
2. If it's new, read the tail of the launch log. Nav2 and HAL both log the
   offending node name and a human-readable error.
3. Check §3 — 80 % of "new" issues are actually duplicate-process or
   stale-overlay symptoms.
4. Check §2 — the other 15 % are overlay-sourcing order issues.
5. If it's genuinely new: capture the symptom, the log snippet, and the fix
   in the current phase's validation log, and add a row to §5 of this file.
