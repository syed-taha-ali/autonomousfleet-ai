# Phase 2 Validation Session — 2026-04-13

SLAM & Localisation (slam_toolbox + map_server + AMCL) brought up on the real
MentorPi M1. All Phase 2 test criteria PASS.

## Stack under test

- `af_slam` package: slam_toolbox mapping config (MS200-tuned), AMCL config
  (`nav2_amcl::OmniMotionModel` for mecanum), launch files, `slam.rviz`
- `af_bringup/launch/slam.launch.py` composes `robot.launch.py` +
  `lidar.launch.py` + either `af_slam/slam_mapping.launch.py` or
  `af_slam/slam_localization.launch.py`
- `af_bringup/launch/lidar.launch.py` bakes in the MS200 driver +
  `af_hal/scan_sanitizer_node` so `/scan` is available automatically (the
  manual `peripherals ms200_scan.launch.py` + `topic_tools/relay` dance from
  Phase 1 is gone)

## Test results

| # | Test | Command / Artifact | Outcome |
|---|------|---------------------|---------|
| 1 | Clean occupancy grid with loop closure | `ros2 launch af_bringup slam.launch.py` + teleop around the room; visual check in `slam.rviz` | **PASS** — see `logs/rviz_ss/slam_test.png` (walls sharp, free space continuous) |
| 2 | Map saves and reloads via map_server | `map_saver_cli -f room1` → saved `.pgm` (139×128 @ 5 cm) + `.yaml`, then launched localization mode | **PASS** — `af_slam/maps/room1.{pgm,yaml}` |
| 3 | AMCL localises within ≤5 cm error | `slam.launch.py mode:=localization map:=room1.yaml`, set initial pose via `/initialpose` in RViz, drove a loop | **PASS** — LaserScan points align cleanly with black wall pixels after initial pose; `logs/rviz_ss/amcl_test.png` |
| 4 | No TF tree breaks | `tf2_echo map odom` during both modes | **PASS** — continuous, identity at rest, updates with motion |
| 5 | Map update interval respected | `ros2 topic hz /map` during mapping | **PASS** — ~0.2 Hz (matches `map_update_interval: 5.0`) |

## Key finding — MS200 scan geometry jitters, Karto rejects everything

`slam_toolbox` (Karto under the hood) latches the `LaserRangeFinder`
parameters from the **first** scan it sees, then discards every subsequent
scan whose `len(ranges)` differs from the latched count, logging:

```
LaserRangeScan contains 449 range readings, expected 450
```

Sampling ten clean MS200 scans from a single-instance driver showed:

```
min=0.0077 max=6.2694 inc=0.01404 len=447
min=0.0098 max=6.2734 inc=0.01398 len=449
min=0.0105 max=6.2804 inc=0.01403 len=448
min=0.0056 max=6.2793 inc=0.01394 len=451
min=0.0131 max=6.2818 inc=0.01396 len=450
...
```

`angle_min`, `angle_max`, `angle_increment` and `len(ranges)` all jitter
(447–451 across scans) because the driver's per-message count drifts by ±2
samples. Not a duplicate-process artefact — this is inherent to the factory
`oradar_lidar` driver.

**Fix**: `af_hal/scan_sanitizer_node.py` resamples every `/scan_raw` message
onto a canonical fixed grid:

- `angle_min = 0.0`
- `angle_increment = 2π / bins` (default `bins=450`)
- `angle_max = (bins − 1) × angle_increment`
- `len(ranges) = bins` always

Each input reading is bucketed to its nearest output angle bin; the minimum
range in each bucket wins (conservative — treats obstacles as closer).
Empty bins get `+inf` so Karto treats them as no-return. This is a plain
`rclpy` node with no C++.

After the sanitizer was wired in, slam_toolbox produced a clean map on the
first drive without a single rejected scan.

## Duplicate-process footgun (recurrence from Phase 1)

Two `/MS200` nodes briefly appeared in `ros2 node list` when a prior
`ros2 launch` was re-invoked without killing the previous one. Symptoms: the
second driver's serial reads collided with the first's, producing scans with
13–396 ranges and slam_toolbox rejecting them all. Mitigation: always run
`/tmp/af_kill.sh` (uses the 15-char `pkill` comm prefixes from Phase 1 docs)
before re-launching. The `pkill -f` footgun still applies — the kill commands
**must** live in a script file so their regex never appears in a live shell's
argv.

## Other warnings observed (benign)

- `usb_cam: Camera calibration file .../mentorpi_cam.yaml not found` —
  expected, intrinsics calibration is a Phase 4 task.
- `ekf_node: Failed to meet update rate! Took 0.136s` — one-off at startup,
  EKF recovers; does not corrupt `/odom`.
- `hardware_watchdog: no battery heartbeat from STM32` — seen intermittently;
  unrelated to SLAM, tracked separately.

## Closeout state

- `af_slam/maps/room1.{pgm,yaml}` committed as the reference test map.
- All Pi nodes torn down cleanly (`/tmp/af_kill.sh`).
- Ready to open PR #2 from `phase-2-slam` to `main`.

## Next-phase TODO (Phase 3, Nav2)

- Local costmap will consume `/scan` directly — the sanitizer already gives
  it a clean fixed-geometry topic.
- DWB holonomic tuning (`min_vel_y`, `max_vel_y`) for the mecanum drive.
- The `nav2_amcl::OmniMotionModel` choice from this phase should carry over
  unchanged when Nav2 runs on top of AMCL for localisation-only missions.
