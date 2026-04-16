# Phase 5a — Find-Object-Return-Home validation session (2026-04-16)

First on-robot session for the Phase 5a exploration demo. Robot is placed
in an unknown small room, captures a home pose, explores via frontier
goals, watches for a target COCO class via YOLO + temporal voting, and
navigates back to home on confirmed detection.

Session A covers **stack-level integration** — launch composition, CPU
headroom, SLAM + Nav2 + perception coexistence, and the state-machine
dry-run. Session B (2026-04-17) covers the **suitcase demo** after the
planner tuning pass resolved §4.3.

---

## 1. Setup

- Robot: MentorPi M1 (Raspberry Pi 5, 4 GB), Humble container `MentorPi`
  accessed via passwordless SSH to `pi@192.168.1.117` → `docker exec -u
  ubuntu -w /home/ubuntu MentorPi`.
- Workspace at container path
  `/home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai/`.
- Battery at session start: 8.07 V (healthy 2S).

### 1.1 New packages deployed

| Package | Description |
|---|---|
| `af_mission_control` (new) | `mission_manager_node` (command router), `find_object_action_node` (ActionServer + state machine), `safety_validator_node` (gate service), `simple_explore_node` (random-walk frontier explorer) |
| `af_msgs` (updated) | `FindObject.action` (goal/result/feedback), `MissionStatus.msg` rewritten, `ValidateCommand.srv`, `MissionCommand.msg`. `CMakeLists.txt` gained `geometry_msgs` + `action_msgs` deps. |
| `af_bringup` (updated) | `explore.launch.py` composes the full stack (robot + lidar + slam_mapping + navigation + mission_control), staggered bringup via `TimerAction`. |
| `af_perception` (config only) | `process_every_n: 3 → 6` in `perception_pi.yaml` to free CPU for SLAM under contention. |

### 1.2 Launch line

```bash
ros2 launch af_bringup explore.launch.py \
    enable_perception:=true enable_explore:=true
```

Goal trigger from Dev PC:
```bash
ros2 action send_goal -f /find_object af_msgs/action/FindObject \
    "{target_class: suitcase, confidence_min: 0.5, votes_required: 3, \
      window_size: 5, max_duration_s: 300.0, max_distance_m: 10.0}"
```

---

## 2. Issues found and fixes applied

### 2.1 Kill-before-launch filter missed support nodes

**Symptom.** After SIGINT on a running `ros2 launch`, two
`robot_state_publisher` instances were found alive — the previous
launch's PID 1748 and the new launch's PID 2423. This broke the TF tree
(`Tf has two or more unconnected trees`, `laser_frame` disconnected from
`odom`), caused slam_toolbox to drop every scan with "Message Filter
dropping message … queue is full", and left Nav2 planner stuck in
`Starting point in lethal space`.

**Root cause.** My kill filter
`pgrep -af "af_hal|slam_toolbox|nav2_|af_mission_control|oradar_lidar|
yolo|depth_estimator|usb_cam"` did not match `robot_state_publisher`,
`complementary_filter`, or `ekf_node`. When `ros2 launch`'s parent dies
by SIGKILL, its children are reparented to PID 1 and survive unless
explicitly targeted. Any missed node outlives the kill pass and competes
with the next launch.

**Fix.** Widen the filter to include every node binary we launch:
```
ros_robot_controller|odom_publisher|imu_calib|hardware_watchdog|usb_cam|
yolo|depth_estimator|oradar|scan_sanitizer|slam_toolbox|controller_server|
smoother_server|planner_server|behavior_server|bt_navigator|
waypoint_follower|velocity_smoother|lifecycle_manager|robot_state_publisher|
complementary_filter|ekf_node|mission_manager|find_object|safety_validator|
simple_explore|static_transform|ros2 launch
```
Better yet, always SIGINT the ros2-launch parent (not SIGKILL), which
propagates to children via launch's shutdown handlers. Only escalate to
explicit per-PID kills if SIGINT is ignored.

Update `bot_testing_instructions.md` §3.2 with this broader filter.

### 2.2 Stale `/dev/shm/fastrtps_*` persists across container restart

**Symptom.** After `docker restart MentorPi`, 186 fastrtps shared-memory
files remained in `/dev/shm`. Fresh ROS processes threw
`[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7417:
open_and_lock_file failed` on startup.

**Root cause.** The MentorPi container binds the host's `/dev/shm` into
the container for DDS shared-memory transport performance. Docker
restart tears down container processes but does not touch host tmpfs.

**Fix.** `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` after any
container restart when SHM pollution is suspected. Add to the pre-launch
cleanup checklist.

### 2.3 ros2cli daemon corruption from bad DDS state

**Symptom.** `ros2 topic info`, `ros2 topic hz`, `ros2 topic echo` all
failed with `xmlrpc.client.Fault: <class 'RuntimeError'>:!rclpy.ok()`
even though target nodes were alive and publishing.

**Root cause.** The ros2cli daemon caches topic/node discovery state.
When stale DDS SHM + phantom nodes poison the graph, the daemon's rclpy
context gets into an un-ok state and every subsequent CLI call fails
until the daemon restarts.

**Fix.** `ros2 daemon stop` (it auto-respawns on next CLI call). If it
happens repeatedly, escalate to `docker restart MentorPi` + SHM cleanup
(§2.2).

### 2.4 Full-stack launch at t=0 saturates Pi 5 and starves SLAM

**Symptom.** `enable_perception:=true enable_explore:=true` without
staggering produced:
- CPU 85% user + 5% sys = 90% used, 10% idle (4 cores).
- `controller_server` at 100% of one core, stuck.
- slam_toolbox continuously dropping scans: "Message Filter dropping
  message: frame 'laser_frame' … queue is full".
- `/map` never emitted (confirmed via `ros2 topic hz -w 1 /map`).
- Planner loops in "Starting point in lethal space".

**Root cause.** ~20 nodes starting simultaneously + YOLO ONNX init +
usb_cam colorspace conversion + SLAM ceres solver warmup + Nav2 costmap
subscriptions all hit the Pi's scheduler at once. slam_toolbox's
`message_filters` queue fills with scans arriving faster than TF cache
can reach them, and once full, every subsequent scan is dropped — SLAM
never integrates a measurement, so /map never publishes.

**Fix.** Staggered bringup in `explore.launch.py` using `TimerAction`:

| t (s) | Stage |
|---|---|
| 0  | HAL + description + EKF + camera (perception explicitly off) |
| 0  | MS200 lidar + scan_sanitizer |
| 5  | slam_toolbox async mapping |
| 12 | Nav2 stack (controller, planner, bt_navigator, lifecycle_manager) |
| 15 | mission_manager, find_object_action, safety_validator |
| 22 | on-Pi YOLOv8n + depth_estimator |
| 25 | simple_explore_node |

SLAM gets a CPU-quiet window (t=5→12) to register the laser sensor and
publish initial /map; perception + explore join only after the mapping
loop is stable.

Additionally, `process_every_n: 3 → 6` in `perception_pi.yaml` drops
YOLO from ~60% CPU to ~43% CPU (0.8–1.6 FPS on 320×320), which is
acceptable for slow indoor exploration and leaves enough headroom for
SLAM + Nav2 to hit their control rates.

### 2.5 Timeout path reports "success=found" instead of "timeout"

**Symptom.** In dry-run test §3.1, the 10-second exploration timeout
completed the action with `success: true, termination_reason: "found"`
even though no detection occurred.

**Root cause.** `find_object_action_node._check_safety()` at line 287
transitions `State.EXPLORING → State.TARGET_CONFIRMED` on timeout,
which then cascades through `RETURN_HOME → DONE` with the standard
success-path result.

**Fix (deferred).** Add a distinct `State.TIMEOUT` that still triggers
`RETURN_HOME` (we still want to return) but sets
`success=False, termination_reason="timeout"` when the result is built.
Cosmetic for Session A; should be fixed before Session B reporting.

### 2.6 SSH resets during launch burst

**Symptom.** One-off SSH `Connection reset by peer` during the t=0
launch burst (before staggering was added).

**Root cause.** Known battery-WiFi coupling (see
`project_battery_wifi_coupling.md` memory) — CPU burst + WiFi transmit
under battery sag briefly starves the SSH TCP keepalive.

**Fix.** Staggered launch (§2.4) mitigates this by flattening the CPU
spike. No battery-level change needed.

---

## 3. Test results

### 3.1 State-machine dry-run (PASS)

`enable_perception:=false enable_explore:=false` with a 10-second
timeout exercises the action server + state machine + Nav2 return-home
handoff without needing the robot to move.

```bash
ros2 action send_goal -f /find_object af_msgs/action/FindObject \
    "{target_class: suitcase, confidence_min: 0.5, votes_required: 3, \
      window_size: 5, max_duration_s: 10.0, max_distance_m: 5.0}"
```

Observed state transitions (from `/mission/status`):
```
IDLE → CAPTURE_HOME → EXPLORING → TARGET_CONFIRMED → RETURN_HOME → DONE
```

Result:
```
success: True
termination_reason: "found"          # See §2.5 — should be "timeout"
home_pose: (0.00, 0.00), q≈identity  # Robot at origin since SLAM just started
detection_pose: empty
home_return_error_m: 0.0
Goal finished with status: SUCCEEDED
```

Validated:
- ActionServer accepts goal, rejects concurrent goals (REJECT when not idle).
- CAPTURE_HOME successfully reads `map → base_link` TF.
- EXPLORING polls for detections + safety, correctly times out.
- RETURN_HOME sends `NavigateToPose` goal to Nav2, callback fires.
- DONE state published, result returned.

### 3.2 Perception-only (PASS)

`enable_perception:=true enable_explore:=false` — no frontier goals
issued, so Nav2 is idle and SLAM runs unobstructed.

Steady-state (30 s after launch):

| Process | CPU% |
|---|---|
| yolo_onnx_node | 26.7 |
| find_object_action | 26.7 |
| depth_estimator_node | 20.0 |
| usb_cam_node_exe | 13.3 |
| Other (HAL, EKF, SLAM, Nav2 idle) | ~15 |
| **Total %Cpu(s) used** | **~45** |

Topic rates:
- `/scan`: 10.0 Hz
- `/detections`: 3.5 Hz
- `/map`: publishing (confirmed via Nav2 costmap subscription logs)

No scan drops, no planner errors. YOLO warmup: first inference 115 ms,
steady ~1.4 FPS (at `process_every_n: 3`).

### 3.3 Full stack — staggered (PARTIAL)

`enable_perception:=true enable_explore:=true` with the staggered
launch and `process_every_n: 6`.

Steady-state:

| Process | CPU% |
|---|---|
| controller_server | ~100 |
| yolo_onnx_node | 43.8 |
| find_object_action | 37.5 |
| mission_manager | 18.8 |
| depth_estimator_node | 12.5 |
| simple_explore | 12.5 |
| async_slam_toolbox | 12.5 |
| ekf_node | 12.5 |
| Other | ~30 |
| **Total %Cpu(s) used** | **~82** |

System: load 2.35 (from 7.86 without staggering), 0 zombies, memory 1.48
GB / 4 GB, battery ~8.0 V.

Topic + TF health:
- `/scan`: 10.0 Hz ✓
- `/detections`: 0.98 Hz ✓ (matches YOLO 1.2 FPS × throttle)
- YOLO: 0.8–1.6 FPS, avg inference 141–230 ms
- slam_toolbox: `Registering sensor: [Custom Described Lidar]` ✓ (single
  startup queue-full message, no flood)
- `map → base_link` TF: **alive** (robot at map=(-1.42, -0.37))
- simple_explore: publishing "Exploring to (x, y)" goals to Nav2

**What worked:** Everything up to and including Nav2 goal dispatch. The
staggered launch definitively fixed the SLAM starvation from §2.4. The
stack is stable and CPU headroom, while thin, is positive.

**What didn't:** See §4.3 for the planner failure chain.

### 3.4 Session B — suitcase demo (PASS, 2026-04-17)

Conducted after resolving §4.3 (global costmap tuning: `robot_radius`
0.12→0.08, `obstacle_min_range` 0→0.15, `cost_scaling_factor` 3→5) and
§4.2 (timeout bug via `_timed_out` flag).

**Trial 1 — Find-suitcase-return-home (full pipeline):**

| Metric | Value |
|---|---|
| Status | SUCCEEDED |
| Target | suitcase (YOLO confidence 0.749) |
| Votes | 3/5 |
| Home pose | (1.02, 1.82) |
| Detection pose | (1.35, 1.67) |
| Home return error | 0.477 m |

State sequence: CAPTURE_HOME → EXPLORING → TARGET_CONFIRMED →
RETURN_HOME → DONE. Full end-to-end pipeline validated: staggered
launch → SLAM mapping → Nav2 frontier goals → YOLO detection →
temporal voting → confirm → navigate home.

### 3.5 Explorer exit-condition tests (PASS, 2026-04-17)

`simple_explore_node` was rewritten with four parameter-driven exit
conditions: time limit, distance limit, object detection, and frontier
exhaustion. Three on-robot tests validated the first three:

| Test | Exit condition | Parameter | Result | Distance | Elapsed |
|---|---|---|---|---|---|
| 1 | Time | `max_explore_time_s: 60` | PASS | 6.32 m | 62 s |
| 2 | Distance | `max_explore_distance_m: 1.0` | PASS | 1.19 m | 6 s |
| 3 | Detection | `stop_on_detection: true, target_class: suitcase` | PASS | 1.74 m | 14 s |

All three exit conditions triggered correctly. The slight overshoot
(62s vs 60s, 1.19m vs 1.0m) is expected — checks run on the replan
timer interval (2s), not continuously.

---

## 4. Outstanding issues

### 4.1 CPU headroom is thin (17–18% idle)

Staggered launch + throttled YOLO gets the full stack running, but the
Pi 5's 4 cores are ~82% loaded at steady state. `controller_server`
occasionally logs `Control loop missed its desired rate of 10 Hz` and
`BehaviorTreeEngine: Behavior Tree tick rate 100 was exceeded`. The
stack functions, but no further CPU-heavy additions are feasible
without platform changes.

Candidate mitigations if Session B reveals problems:
- Raise `process_every_n` to 8 or 10 (YOLO at ~0.5 FPS; still OK for
  ≤0.1 m/s exploration).
- Drop Nav2 `controller_frequency` from 10 Hz to 5 Hz.
- Move perception to Dev PC if on-Pi constraint can be relaxed (loses
  the Phase 4.1 "on-robot vision" claim).

### 4.2 Timeout termination reports as "found" (see §2.5)

One-line fix before Session B. Add `State.TIMEOUT`, branch in
`_make_result`.

### 4.3 Nav2 global planner fails on frontier goals

**This is the Session A blocker for Session B.**

With everything else working, `simple_explore_node` publishes a frontier
goal (e.g. `(-1.71, 1.05)`) and the SmacPlanner2D alternates between
two failure modes:

1. `GridBased: failed to create plan, no valid path found.` — target
   unreachable through currently-known free space.
2. `GridBased: failed to create plan, invalid use: Starting point in
   lethal space! Cannot create feasible plan.` — the robot's own cell
   in the global_costmap is marked lethal.

The controller briefly receives a path ("Passing new path to
controller") on some attempts, but subsequent re-plans fail, and the
robot never commits meaningful motion. Recovery behaviors
(ClearCostmap → Spin → BackUp → Wait) fire but don't break the cycle.

Likely causes (to investigate):
- Phase 3 global_costmap `inflation_radius: 0.15 m` bleeds the
  lethal-edge inflation zone into the robot's own cell when initial
  partial scans first mark walls.
- `simple_explore_node._find_frontier_cells()` picks cells adjacent to
  unknown regions, but the SmacPlanner2D without `allow_unknown: true`
  can't route through unknown space to reach them.
- Frontier cell subsampling (`random.choice` on up to 200 candidates)
  frequently picks cells on the far side of narrow wall gaps.

Proposed next pass (outside this session):
- Inspect current `af_navigation/config/nav2_params.yaml` for
  `allow_unknown`, `footprint`, `inflation_radius` in the global layer.
- Add a `/clear_costmaps` call from `simple_explore_node` on repeated
  plan failures (or use Nav2's ClearCostmap recovery more aggressively).
- Consider selecting frontier cells only from the connected free
  component containing the robot (BFS from `base_link` cell).

---

## 5. Lessons learned

1. **Widen the kill-before-launch filter to every node binary you
   launch, including `robot_state_publisher`, `complementary_filter`,
   `static_transform_publisher`, and `ekf_node`.** Any missed process
   survives SIGKILL of the launch parent by reparenting to PID 1.
   Competing `robot_state_publisher` instances silently break TF.

2. **`docker restart MentorPi` does not clean `/dev/shm`.** Follow up
   with `rm -f /dev/shm/fastrtps_*` when stale DDS SHM is suspected.

3. **Staggered launch is a functional requirement, not a polish step,
   for the full Phase 5a stack on Pi 5.** SLAM's `message_filters`
   queue is small; if it fills during a CPU-saturated startup, the
   drop-every-scan failure mode is non-recoverable without relaunch.

4. **`process_every_n: 6` is the viable YOLO throttle when running
   alongside SLAM + Nav2 + explorer.** Lower values starve SLAM; higher
   values produce sub-0.5 FPS perception that may miss targets.

5. **Frontier explorer + SmacPlanner2D have a bootstrap problem that
   is not obvious from Phase 2/3 validation.** Pre-built-map navigation
   (Phase 3) hides this because inflation + static_layer are consistent
   from t=0; exploration reveals it because the costmap grows with the
   robot's view and catches the robot inside its own inflation zone.

6. **STM32 motor watchdog stops the robot when HAL dies.** Confirmed
   reliable across multiple kill/restart cycles this session — no
   runaway motor events.

---

## 6. Next steps

- ~~Fix §4.2 (TIMEOUT state)~~ — done (2026-04-17, `_timed_out` flag).
- ~~Investigate §4.3 (planner + exploration costmap tuning)~~ — done
  (2026-04-17, global costmap `robot_radius` 0.08, `cost_scaling_factor`
  5.0, `obstacle_min_range` 0.15).
- ~~Session B: suitcase demo~~ — done (2026-04-17, §3.4).
- ~~Explorer exit conditions~~ — done (2026-04-17, §3.5).
- Commit all Phase 5a work to `phase-5a-find-object-return-home` and
  merge PR #5 to main.
- Update CLAUDE.md Current State with Phase 5a summary.
