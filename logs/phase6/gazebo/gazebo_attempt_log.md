# Phase 6 — Gazebo Classic Simulation Attempt Log

**Date**: 2026-04-17 to 2026-04-18
**Goal**: Launch N robots in Gazebo Classic 11 with SLAM (slam_toolbox) + Nav2 per robot.
**Outcome**: Abandoned in favour of MVSim due to two blocking issues that proved intractable within Gazebo Classic.

---

## Configuration

- **Platform**: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.10.2
- **Robot**: MentorPi M1 (mecanum), xacro URDF with Gazebo plugins (planar_move, ray_sensor, camera, IMU)
- **World**: `search_arena.world` — 8x6 m walled room with 5 obstacles and 5 target cylinders
- **Launch**: `af_gazebo/launch/simulation_full.launch.py` using `OpaqueFunction` + event-driven spawning
- **Nav2 params**: `af_gazebo/config/nav2_sim_params.yaml` (DWB, SmacPlanner2D, costmap layers)
- **SLAM params**: `af_gazebo/config/slam_sim_params.yaml` (async slam_toolbox, Ceres solver)

---

## Issues Encountered and Fixes Applied

### 1. Nav2 DWB Critics Not Loading ("No critics defined for FollowPath")

**Root cause**: Namespaced nodes (e.g. `/robot_0/controller_server`) couldn't find params in a flat YAML file. Sub-nodes like costmaps need the YAML wrapped under the namespace key.

**Fix**: Used `nav2_common.launch.RewrittenYaml` with `root_key=namespace` for both Nav2 and SLAM params. This wraps the YAML under the robot's namespace so all nodes and sub-nodes find their parameters.

**Status**: SOLVED

### 2. SLAM scan_topic Defaulting to `/scan` Instead of Relative `scan`

**Root cause**: Same as above — SLAM params weren't namespace-wrapped, so the namespaced `/robot_0/slam_toolbox` node couldn't find its `scan_topic: scan` setting and fell back to the absolute `/scan` default.

**Fix**: Applied `RewrittenYaml` to SLAM params as well.

**Status**: SOLVED

### 3. DDS Shared Memory Exhaustion

**Symptom**: "Failed init_port fastrtps_port*: open_and_lock_file failed" after repeated test runs.

**Root cause**: FastRTPS shared memory ports in `/dev/shm/` accumulate across runs.

**Fix**: `rm -f /dev/shm/fastrtps_*` before each test, plus `export ROS_LOCALHOST_ONLY=1`.

**Status**: SOLVED (operational workaround)

### 4. gzserver "Address Already in Use"

**Root cause**: Leftover gzserver/gzmaster processes from previous runs.

**Fix**: `killall -9 gzserver gzclient gzmaster` before launching.

**Status**: SOLVED (operational workaround)

### 5. Timer-Based Spawning Race Conditions

**Root cause**: Original launch used `TimerAction` with hardcoded delays (8s initial, +10s per robot). Timing was fragile — Nav2 could start before Gazebo finished spawning.

**Fix (from reference repo `arshadlab/tb3_multi_robot`)**: Replaced `TimerAction` with event-driven `RegisterEventHandler(OnProcessExit(...))`. Robot N+1 spawns only after robot N's `spawn_entity.py` process exits. Nav2 starts only after the spawn process exits.

**Status**: SOLVED

### 6. Multi-Robot TF Frame Conflicts

**Root cause**: Gazebo Classic publishes all robots' TF to the global `/tf` with identical frame names (`odom`, `base_footprint`), causing conflicts between robots.

**Attempted fix (from reference repo)**: Remap `/tf` → `tf` and `/tf_static` → `tf_static` at the node level for RSP, SLAM, and Nav2 nodes. Also tried adding `<remapping>/tf:=tf</remapping>` to Gazebo plugins.

**Result**: TF remapping broke SLAM — it couldn't process scans because the TF message filter couldn't resolve frames across the remapped topics. The reference repo uses pre-built SDF files (not URDF→SDF conversion), which sidesteps this issue.

**Status**: DEFERRED — reverted TF remapping; single-robot works without it

### 7. Gazebo Classic URDF→SDF Strips Custom Plugin Parameters

**Root cause**: Gazebo Classic's URDF→SDF converter strips custom `<ros>` sub-elements like `<odometry_frame>` and `<robot_base_frame>` from plugins. This prevents frame name customisation for multi-robot setups.

**Attempted fix**: Set namespaced frame names (`robot_0/odom`, etc.) in the xacro.

**Result**: Gazebo logged "Publishing odom transforms between [odom] and [base_footprint]" regardless of what was set in the xacro — the custom parameters were stripped during conversion.

**Status**: UNSOLVABLE in Gazebo Classic with URDF. Reference repos use pre-built SDF files to avoid this.

### 8. gzserver SIGSEGV (Exit Code -11) — ~50% Crash Rate

**Symptom**: gzserver crashes intermittently during or shortly after robot spawn with `exit code -11` (segmentation fault).

**Root cause investigation**:
- `ldd` on `libgazebo_ros_camera.so` revealed missing dependencies: `libCameraPlugin.so` and `libDepthCameraPlugin.so`. These exist in `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/` but that directory was NOT in `LD_LIBRARY_PATH` or `GAZEBO_PLUGIN_PATH`.
- The camera plugin's unresolved symbols during load caused the SIGSEGV.

**Fixes applied**:
1. Added `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins` to both `GAZEBO_PLUGIN_PATH` and `LD_LIBRARY_PATH` in the launch file's `_setup_gazebo_env()`.
2. Made the camera plugin conditional (`enable_camera` xacro arg) and disabled it for headless simulation.
3. Reduced physics rate from 250 Hz / 0.004s step to 200 Hz / 0.005s step.

**Result**: Disabling the camera plugin eliminated the SIGSEGV entirely. With camera enabled, crashes persisted even with correct library paths (suggesting a deeper Gazebo Classic bug with camera + URDF→SDF conversion).

**Status**: SOLVED by disabling camera. Camera is not needed for SLAM/Nav2 testing.

### 9. SLAM Never Processes Scans (Blocking Issue)

**Symptom**: slam_toolbox starts, receives the first scan (logs "minimum laser range" warning and "Registering sensor"), but never processes any scan. No map published, no "Processing" output, no error messages.

**Verified working**:
- `/robot_0/scan` topic actively publishing valid LaserScan at 10 Hz sim time
- `/robot_0/odom` topic publishing
- TF chain `odom` → `base_footprint` → `base_link` → `lidar_frame` fully available
- `getOdomPose()` succeeds (no "Failed to compute odom pose" warning)
- All Nav2 lifecycle nodes activate successfully
- `use_sim_time: true` set on all nodes
- `/clock` topic publishing from Gazebo

**Attempted fixes**:
1. Increased `transform_timeout` from 0.3 to 10.0 seconds — no effect
2. Enabled `debug_logging: true` — no additional SLAM output beyond registration
3. Switched from `async_slam_toolbox_node` to `sync_slam_toolbox_node` — same behaviour
4. Temporarily set `use_sim_time: false` on SLAM — got "Message Filter dropping message: frame 'lidar_frame' at time 0.460 for reason 'discarding message because the queue is full'" (confirms TF message filter timing mismatch is the root cause)
5. Checked `ros2 topic info /robot_0/scan --verbose` — showed 0 subscribers (message filter subscription doesn't appear in topic info)

**Root cause hypothesis**: slam_toolbox's internal `tf2_ros::MessageFilter` drops all scan messages because the TF lookup at the scan's sim-time timestamp fails or times out within the filter. This appears to be a timing/synchronisation issue between Gazebo Classic's sim time clock and the TF2 message filter's internal timeout logic. The filter receives the scan, checks for TF availability at the scan timestamp, fails (possibly due to clock skew between `/clock` publication rate and scan publication rate), and silently drops the message. Only the very first scan passes through (triggering sensor registration), likely because it hits a special "first message" code path that bypasses the filter.

**Status**: UNSOLVED — this is the primary reason for abandoning Gazebo Classic.

---

## Reference Repo Analysis

Cloned `arshadlab/tb3_multi_robot` (humble branch) to `src/project/inspo_sim/` for comparison.

**Key patterns from the reference repo**:
- Event-driven spawning with `RegisterEventHandler(OnProcessExit(...))` — adopted
- TF remapping `('/tf', 'tf')` on RSP and in SDF plugins — attempted, broke SLAM
- Pre-built SDF model files (not URDF→SDF conversion) — avoids plugin parameter stripping
- `RewrittenYaml` for Nav2 params with `root_key=namespace` — adopted
- `-robot_namespace` flag on `spawn_entity.py` — adopted
- Uses AMCL (pre-loaded map), not SLAM — avoids the SLAM timing issue entirely

**Key difference**: The reference repo uses **AMCL with a pre-loaded map**, not online SLAM. This sidesteps the TF message filter timing issue we hit with slam_toolbox in simulation.

---

## Files Modified During Gazebo Attempt

| File | Changes |
|------|---------|
| `af_gazebo/launch/simulation_full.launch.py` | RewrittenYaml, event-driven spawning, TF remappings (reverted), GAZEBO_PLUGIN_PATH fix, camera disable, sync SLAM (to revert) |
| `af_gazebo/config/slam_sim_params.yaml` | `debug_logging: true`, `transform_timeout: 10.0` (to revert) |
| `af_gazebo/worlds/search_arena.world` | Reduced physics rate to 200 Hz / 0.005s step |
| `af_description/urdf/gazebo_plugins.xacro` | Added `enable_camera` conditional, attempted `/tf:=tf` remapping (reverted) |
| `src/project/.gitignore` | Added `inspo_sim/` |

---

## Test Logs

Raw simulation output logs saved to `/tmp/` during the session:
- `sim_1r.log` — first test, gzserver "Address already in use"
- `sim_1r_clean.log` — clean start, all nodes launched but SLAM untested
- `sim_1r_unbuf.log` — unbuffered output test
- `sim_eventdriven.log` — first event-driven spawning test (with TF remapping)
- `sim_notf.log` — without TF remapping, SLAM still no processing
- `sim_bg.log` — long-running background test, verified scan data via `ros2 topic echo`
- `sim_slamdebug.log` — SLAM with `debug_logging: true`
- `sim_slamdebug2.log` — SLAM with `--log-level debug` (RCL noise, no app-level debug)
- `sim_t10.log` — `transform_timeout: 10.0`, good run (no crash), SLAM still no processing
- `sim_sync.log`, `sim_sync2.log`, `sim_sync3.log` — sync SLAM attempts (2 of 3 had gzserver crash)
- `sim_nc2.log` — camera disabled, no crash, SLAM registered but no processing
- `sim_fp2.log` — fixed plugin paths + camera disabled, same result
- `sim_final.log` — 120s run, no crash, all Nav2 active, SLAM registered but no processing
- `sim_nst2.log` — `use_sim_time: false` on SLAM, revealed MessageFilter dropping messages

---

## Decision

Abandoned Gazebo Classic 11 for the swarm simulation tier. Two blocking issues could not be resolved:

1. **Camera plugin causes SIGSEGV** — fixable by disabling camera, but limits future perception testing
2. **slam_toolbox never processes scans** — TF message filter timing issue with sim time; no fix found after extensive debugging

Proceeding with **MVSim** (lightweight 2D multi-vehicle simulator) which is purpose-built for multi-agent SLAM + Nav2 workflows and avoids Gazebo Classic's URDF→SDF conversion, plugin loading, and sim-time synchronisation issues.
