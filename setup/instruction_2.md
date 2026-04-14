# Phase 2 — SLAM mapping and AMCL localisation

End-to-end walkthrough. Assumes Phase 1 already works on this robot (see
`instruction_1.md`) and that the `phase-2-slam` branch (or `main` after PR
#2 merges) is checked out in the container at
`~/workspace/ros2_ws/src/autonomousfleet-ai/`.

Everything below runs **inside the MentorPi Docker container** unless stated
otherwise.

---

## 1. One-shot bringup

Two operating modes, same launch file:

```bash
# Mapping mode (default) — slam_toolbox builds a map online
ros2 launch af_bringup slam.launch.py

# Localisation mode — map_server + AMCL against a saved map
ros2 launch af_bringup slam.launch.py mode:=localization \
    map:=$(ros2 pkg prefix af_slam)/share/af_slam/maps/room1.yaml
```

`slam.launch.py` composes:

- `af_bringup/launch/robot.launch.py` — Phase 1 HAL + description + EKF +
  usb_cam + `lidar_frame → laser_frame` static TF
- `af_bringup/launch/lidar.launch.py` — MS200 driver (`oradar_lidar`) +
  `scan_sanitizer_node` publishing on `/scan`
- Either `af_slam/launch/slam_mapping.launch.py` **or**
  `af_slam/launch/slam_localization.launch.py`

The factory `oradar_lidar` package lives in `/home/ubuntu/ros2_ws/install/`,
not our overlay, so source it first:

```bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/workspace/ros2_ws/install/setup.bash
```

Add those two lines to `~/.bashrc` to make every `docker exec` shell work
without the dance.

## 2. Why `scan_sanitizer_node` is mandatory

`slam_toolbox` (Karto) latches `LaserRangeFinder` parameters from the first
scan it sees and rejects every subsequent scan whose `len(ranges)` differs.
The MS200 driver's per-message sample count drifts between 447 and 451, so
without the sanitizer Karto logs:

```
LaserRangeScan contains 449 range readings, expected 450
```

…for every scan, and no map ever builds.

`af_hal/scan_sanitizer_node.py` subscribes to `/scan_raw`, resamples each
message onto a canonical 450-bin grid (`angle_min = 0`,
`angle_increment = 2π/450`), and publishes `/scan`. Every downstream
consumer (slam_toolbox, AMCL, Nav2 costmaps in Phase 3) sees a fixed
geometry and accepts every scan.

The sanitizer is launched automatically by `af_bringup/launch/lidar.launch.py`.
If you want to disable it for another lidar later, set `start_lidar:=false`
on `slam.launch.py` and run your own.

## 3. Build a map

1. Launch mapping mode:

   ```bash
   ros2 launch af_bringup slam.launch.py
   ```

2. On your **Dev PC**, open the SLAM RViz config:

   ```bash
   rviz2 -d ~/ros2_ws/src/project/af_slam/config/slam.rviz
   ```

   Fixed Frame is `map`. You should see an empty map with the robot model
   at origin; the map fills in as scans accumulate.

   If rviz crashes on `libpthread undefined symbol: __libc_pthread_init`
   under a VS Code snap terminal, scrub the env first:

   ```bash
   env -i HOME=$HOME USER=$USER DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY \
       PATH=/opt/ros/humble/bin:/usr/bin:/bin \
       bash -c 'source /opt/ros/humble/setup.bash && \
                rviz2 -d ~/ros2_ws/src/project/af_slam/config/slam.rviz'
   ```

3. Drive the robot around the room with teleop (Dev PC, separate terminal):

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

   Keep linear speed ≤0.15 m/s (`x` twice to drop from default 0.5). Do a
   complete loop and return to the start point — loop closure is what makes
   slam_toolbox correct accumulated drift.

4. Save the map (inside the container):

   ```bash
   ros2 run nav2_map_server map_saver_cli \
       -f /home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai/af_slam/maps/room1 \
       --ros-args -p save_map_timeout:=5.0
   ```

   Produces `room1.pgm` + `room1.yaml`. The shortcut is
   `af_slam/scripts/save_map.sh [map_name]`.

## 4. Localise within a saved map

1. Launch localisation mode:

   ```bash
   ros2 launch af_bringup slam.launch.py mode:=localization \
       map:=/home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai/af_slam/maps/room1.yaml
   ```

2. On the Dev PC, open the same RViz config (`af_slam/config/slam.rviz`).

3. **Set the initial pose**:
   - Click **2D Pose Estimate** in the RViz toolbar (green arrow icon).
   - Press-and-hold at the robot's real position on the map.
   - Drag in the direction the robot is facing.
   - Release.

   Tolerance: position within ~30 cm, heading within ~15°. AMCL's particle
   filter refines the rest as soon as the robot moves ≥0.1 m or rotates
   ≥0.2 rad (`update_min_d` / `update_min_a` in `amcl_params.yaml`).

4. Drive around with teleop. LaserScan points (yellow) should stay aligned
   with the map's black wall pixels. If they're not, your initial pose was
   bad — click 2D Pose Estimate again and re-set it.

5. AMCL pose published on `/amcl_pose`; `map → odom` TF updates whenever
   AMCL applies a correction.

## 5. Config quick reference

| File | Role | Key knobs |
|---|---|---|
| `af_slam/config/mapper_params_online_async.yaml` | slam_toolbox mapping | `max_laser_range: 13.4` (MS200 spec), `minimum_travel_distance: 0.3`, `map_update_interval: 5.0`, `resolution: 0.05` |
| `af_slam/config/amcl_params.yaml` | AMCL + map_server | `robot_model_type: nav2_amcl::OmniMotionModel` (critical — mecanum), `min_particles: 500`, `max_particles: 2000`, `laser_model_type: likelihood_field` |
| `af_slam/config/slam.rviz` | RViz view for both modes | Fixed Frame `map`, TopDownOrtho, Map + LaserScan (BEST_EFFORT) + Odometry + RobotModel, `SetInitialPose` + `SetGoal` tools enabled |

## 6. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `LaserRangeScan contains N range readings, expected M` flood | `scan_sanitizer_node` not running or `/scan` has wrong geometry | `ros2 node list | grep scan_sanitizer` — if missing, relaunch via `slam.launch.py` (or check that `af_hal` was rebuilt after adding the sanitizer) |
| `/scan` at 10 Hz but `/map` never updates, no `map → odom` TF | slam_toolbox rejecting scans; see above | same fix |
| `/scan` hz is 20 Hz, readings wildly variable (13–396) | Two MS200 driver instances fighting `/dev/ldlidar` | `bash /tmp/af_kill.sh` (the script form of the Phase 1 kill list) before every `ros2 launch` |
| AMCL converges to wrong symmetric pose (e.g., 180° off) | Initial pose too far off in a square room | Click **2D Pose Estimate** again with a better guess, drive a short distance |
| `Invalid frame ID "map" passed to canTransform` when running `tf2_echo map odom` | AMCL or slam_toolbox not publishing the `map → odom` TF yet | Wait 2–3 s after launch; if it persists, check for rejected-scan errors (fix above) |
| `ros2 launch af_bringup slam.launch.py` fails with `Package 'oradar_lidar' not found` | Factory overlay not sourced | `source /home/ubuntu/ros2_ws/install/setup.bash` before the launch (add to `~/.bashrc`) |

## 7. Reporting back

Capture a session rosbag:

```bash
ros2 bag record -o phase2_validation \
    /scan /scan_raw /odom /odom_raw /imu /cmd_vel \
    /map /map_metadata /amcl_pose /particle_cloud \
    /initialpose /diagnostics /tf /tf_static
```

Drive once in mapping mode, save the map, switch to localisation mode, drop
the initial pose, drive another loop, `Ctrl-C` the bag. Attach screenshots
of (a) the final map in RViz during mapping, (b) the robot's laser aligning
with walls after AMCL convergence.

## Teardown

```bash
# In each ros2 launch terminal: Ctrl-C
# Or nuke the whole stack from any shell:
bash /tmp/af_kill.sh
```

---

**Phase 3 bake-in targets**: Nav2 launch files into `af_bringup`, local
costmap configured to consume `/scan` from the sanitizer (and later
`/vision_obstacles` in Phase 4), DWB tuned with `min_vel_y` / `max_vel_y`
for mecanum holonomic motion.
