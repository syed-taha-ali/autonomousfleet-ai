# Module G — Monitoring, Logging & Visualisation: Validation Session

**Date**: 2026-04-17
**Location**: Dev PC (Ubuntu 22.04 + RTX 3060) + Pi 5 (MentorPi Docker)

---

## 1. Package Created: `af_monitoring`

New ROS 2 Python package with:

| Component | File | Purpose |
|-----------|------|---------|
| `perf_monitor_node` | `af_monitoring/perf_monitor_node.py` | Subscribes to 6 key topics, measures Hz + stamped-message latency, publishes aggregated `/diagnostics` at 1 Hz |
| `detection_overlay_node` | `af_monitoring/detection_overlay_node.py` | Dev-PC-side: composites YOLO bounding boxes from `/detections` onto `/camera/image_raw/compressed`, publishes `/detection_image/compressed` |
| RViz configs | `config/{navigation,slam,perception,full}.rviz` | 4 mode-specific layouts |
| `rviz.launch.py` | `launch/rviz.launch.py` | `mode:=navigation\|slam\|perception\|full` selector |
| `record_session.launch.py` | `launch/record_session.launch.py` | Records 12 topics with 5-min bag rotation |
| `monitor.launch.py` | `launch/monitor.launch.py` | Launches `perf_monitor_node` |

Bringup script: `af_bringup/scripts/start_monitoring.sh` (flags: `--mode`, `--no-record`, `--no-rviz`).

---

## 2. Test Results

### T1: perf_monitor_node — PASS

Published `/diagnostics` with per-topic stats:

| Topic | Rate | Avg Latency |
|-------|------|-------------|
| `/scan` | 8.3–11.0 Hz | 192 ms |
| `/odom` | 23.9–31.2 Hz | 114 ms |
| `/imu` | 27.0–27.1 Hz | 61 ms |
| `/cmd_vel` | 0 Hz (idle, expected) | — |
| `/map` | received (transient local) | — |
| `/camera/demo/compressed` | 0 Hz (demo stream off, expected) | — |

### T2: RViz2 Configs — PASS

- `full.rviz`: Map, global/local costmaps, laser scan, odometry, global/local plan, robot model, YOLO detection overlay, AMCL pose — all rendering correctly.
- Navigation tools (2D Pose Estimate, 2D Goal Pose) present in toolbar.
- TopDownOrtho view with map fixed frame.
- Screenshot evidence: `logs/rviz_ss/monitoring_ss_1.png` (initial), `monitoring_ss_2.png` (camera feed working), `monitoring_ss_3.png` (YOLO overlay with suitcase detection).

### T3: Rosbag Recording — PASS

7-second test recording captured 274 messages across 11 topics:

| Topic | Count |
|-------|-------|
| `/imu` | 163 |
| `/scan` | 40 |
| `/odom` | 26 |
| `/diagnostics` | 32 |
| `/detections` | 4 |
| `/tf_static` | 4 |
| `/tf` | 4 |
| `/map` | 1 |

Bag size: 365.9 KiB for 7.2 s. All topic types correct.

### T4: Detection Overlay (Dev-PC-side) — PASS

`detection_overlay_node` running on Dev PC, compositing YOLO bounding boxes onto camera feed. Suitcase detected with green bounding box and label in RViz camera panel (screenshot `monitoring_ss_3.png`).

### T5: start_monitoring.sh Script — PASS

Launches perf_monitor + detection_overlay + RViz2 with correct flags. `--no-record` and `--mode` arguments work. Ctrl+C cleanup trap tested.

---

## 3. Issues Encountered & Fixes

### 3.1 RViz2 Snap glibc Crash

**Symptom**: `rviz2` crashes on launch with:
```
symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

**Cause**: VS Code runs as a snap on Ubuntu 22.04. The snap runtime injects environment variables (e.g., `XDG_DATA_HOME=/home/taha/snap/code/228/...`) that cause the linker to find `/snap/core20` libraries before system ones.

**Fix**: `start_monitoring.sh` launches RViz2 via `env -i` with only `HOME`, `DISPLAY`, `XAUTHORITY`, and `XDG_RUNTIME_DIR` forwarded, then re-sources ROS inside the clean environment. This prevents snap library contamination.

### 3.2 Camera "No Image" in RViz

**Symptom**: Camera panel showed "No Image" on first launch.

**Cause**: RViz config pointed to `/camera/demo/compressed`, but `explore.launch.py` disables the demo stream (`enable_demo_stream: 'false'`).

**Fix**: Switched the full and perception RViz configs to subscribe to `/camera/image_raw` with `Transport Hint: compressed`. Camera feed appeared immediately.

### 3.3 Pi CPU Exhaustion from On-Pi YOLO Overlay

**Symptom**: First attempt added overlay rendering (cv2.rectangle + cv2.imencode JPEG) directly inside `yolo_onnx_node.py` on the Pi. The Pi became unresponsive — SSH hung, RViz froze, NLP commands stopped processing.

**Cause**: The overlay added ~30–50% CPU per frame on the Pi 5 (cv2.imencode is expensive on ARM) on top of the already ~80% baseline from HAL + SLAM + Nav2 + YOLO inference. This pushed total CPU past 100%, starving all processes.

**Fix**: Reverted `yolo_onnx_node.py` to its original state (no overlay). Created a new **Dev-PC-side** `detection_overlay_node` in `af_monitoring` that:
1. Subscribes to `/camera/image_raw/compressed` (the raw compressed stream already crossing WiFi)
2. Subscribes to `/detections` (tiny Detection2DArray, ~1 KB)
3. Decompresses the JPEG, draws bounding boxes from latest detections, re-encodes, publishes `/detection_image/compressed`

This moves all overlay CPU cost to the Dev PC (negligible on an RTX 3060 machine) while the Pi's CPU budget is unchanged. The RViz `full.rviz` config points to `/detection_image/compressed`.

### 3.4 Bash `set -euo pipefail` vs ROS setup.bash

**Symptom**: `start_monitoring.sh` and `start_nlp.sh` exited immediately with `AMENT_TRACE_SETUP_FILES: unbound variable`.

**Cause**: `set -u` (nounset) + `/opt/ros/humble/setup.bash` references `AMENT_TRACE_SETUP_FILES` without a default.

**Fix**: Changed both scripts from `set -euo pipefail` to `set -eo pipefail`.

---

### T6: NLP Dispatch with Full Monitoring Stack — PASS

After Pi recharge, full stack restarted and NLP find-object mission executed successfully with monitoring running throughout.

**NLP command**: "explore till you find the suitcase."
- Tool call: `find_object` (target_class=suitcase, confidence_min=0.5)
- LLM latency: 3.75 s (Qwen2.5 7B Q4)

**Mission execution** (from Pi logs):
- Explorer activated, sent frontier goals to Nav2
- YOLO detected suitcase, temporal voting confirmed (3/5 votes)
- State transitions: `EXPLORING → TARGET_CONFIRMED → RETURN_HOME → DONE`
- Home return error: 0.408 m
- Explorer disabled on completion, Nav2 goal cancelled cleanly
- Total exploration: 7.91 m travelled, 50 s elapsed

**Pi resource usage during mission**:

| Metric | Value | Status |
|--------|-------|--------|
| CPU load (1-min avg) | 3.70 / 4 cores (~93%) | Healthy, no starvation |
| RAM | 1485 MB / 4045 MB (37%) | Plenty of headroom |
| Temperature | 48.5 °C | Well under 80 °C throttle |
| Battery | 8.089 V (8089 mV) | Full charge |
| YOLO inference | 0.6–1.0 FPS, 118–141 ms avg | Within budget |

**Key validation**: Pi remained responsive with SSH <200 ms RTT throughout the mission. The Dev-PC-side overlay approach adds zero CPU cost to the Pi while providing full YOLO visualisation in RViz.

Screenshot evidence: `logs/rviz_ss/monitoring_ss_final.png` — RViz full config during NLP find-object mission showing YOLO detection overlay (suitcase with bounding box), SLAM map with explored area, odometry trail (red arrows showing exploration path), and costmap overlays.

---

## 4. Pi Battery Death (Mid-Session)

Pi became unreachable (0% ping) during the first test session — battery sag from prior sessions (documented in `project_battery_wifi_coupling.md`). After recharge, all tests passed on the second session. T1–T5 validated in session 1; T6 validated in session 2.

---

## 5. Architecture Summary

```
Pi 5 (MentorPi Docker)              WiFi              Dev PC
─────────────────────      ─────────────────      ──────────────
HAL + LiDAR + camera                               perf_monitor_node
SLAM (slam_toolbox)        /scan, /odom, /imu ──►    └► /diagnostics
Nav2 stack                 /detections ──────────►
YOLO (yolo_onnx_node)     /camera/.../compressed ►  detection_overlay_node
mission_manager                                       └► /detection_image/compressed
                                                    RViz2 (full.rviz)
                                                    NLP (nlp_command_node + cli)
                                                    [optional] rosbag recorder
```
