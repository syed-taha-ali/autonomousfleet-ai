# Phase 4 / 4.1 — Perception validation session (2026-04-15)

On-robot deployment and validation of the Phase 4.1 on-Pi perception
pipeline: YOLOv8n via ONNX Runtime on the Pi 5 CPU, depth_estimator
ground-plane back-projection, throttled demo camera stream for RViz.

---

## 1. Setup

- Robot: MentorPi M1 (Raspberry Pi 5, 4 GB), Humble container `MentorPi`
  accessed via passwordless SSH to `pi@192.168.1.117` → `docker exec -u
  ubuntu -w /home/ubuntu MentorPi`.
- Workspace at container path
  `/home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai/`.
- Model: `yolov8n_320.onnx` (12.1 MB, exported with opset 17, input
  `[1,3,320,320]`, output `[1,84,2100]`).
- Launch line:
  ```bash
  ros2 launch af_bringup robot.launch.py enable_perception:=true
  ```

### 1.1 New packages deployed

| Package | Description |
|---|---|
| `af_perception` | `yolo_onnx_node` (ONNX Runtime YOLOv8n), `depth_estimator_node` (ground-plane back-projection) |
| `af_bringup` (updated) | `robot.launch.py` now accepts `enable_perception`, `enable_demo_stream`, `demo_stream_hz` args; composes perception_pi launch |

### 1.2 Deployment pipeline

1. ONNX export on Dev PC: `scripts/export_onnx.sh` → `yolov8n_320.onnx`
2. rsync each package separately to Pi host `/home/pi/docker/tmp/`
3. `docker exec ... cp` from shared mount into container workspace
4. `colcon build --packages-select af_perception af_bringup af_navigation`
5. Model placed at `install/af_perception/share/af_perception/models/yolov8n_320.onnx`

---

## 2. Issues found and fixes applied

### 2.1 ONNX Runtime on aarch64

**Symptom.** Initial concern that onnxruntime would not be available on
Pi 5 (aarch64).

**Resolution.** `pip install --user onnxruntime` pulled
onnxruntime-1.23.2 with CPUExecutionProvider. No build-from-source
needed. ~60 ms inference at 320×320.

### 2.2 YOLOv8 output format

**Symptom.** First attempt at post-processing produced no detections.

**Root cause.** YOLOv8 ONNX output shape is `[1, 84, 2100]` — the 84
dimension is `4 (xywh) + 80 (classes)`, and there is no objectness
score (unlike YOLOv5). Confidence is `max(class_scores)` directly.
Detections are columns, not rows.

**Fix.** Transpose output (`.T`), extract class confidence as
`max(scores[4:])` per detection, apply threshold, convert xywh→xyxy
with letterbox padding removal, then `cv2.dnn.NMSBoxes`.

### 2.3 rsync flattened multiple source directories

**Symptom.** `rsync af_perception/ af_bringup/ af_navigation/ pi@...:dst/`
merged all three packages into one directory tree.

**Fix.** rsync each package separately with explicit destination paths.

### 2.4 Pi workspace not directly accessible via rsync

**Symptom.** Cannot rsync into `/home/ubuntu/workspace/ros2_ws/` because
it is inside the Docker container, not on the host filesystem.

**Fix.** Use the bind mount: rsync to host path `/home/pi/docker/tmp/`,
then `docker exec ... cp` from `/home/ubuntu/shared/` into the
container workspace. rsync is not installed inside the container.

### 2.5 `pkill -f` kills own shell wrapper (exit 137)

**Symptom.** Every `pkill -f <pattern>` invoked via `docker exec ...
bash -lc "pkill -f ekf_node ..."` killed the enclosing bash process
with SIGKILL (exit 137), terminating the SSH session.

**Root cause.** `pkill -f` matches the full command line of all
processes, including the `bash -lc "..."` wrapper whose argv contains
all the target patterns. pkill kills itself before reaching the real
targets.

**Fix.** Use 15-char comm prefix approach from `bot_testing_instructions.md`
§3.2: write kills to `/tmp/af_kill.sh` and run `bash /tmp/af_kill.sh`,
or use `pgrep -f <pat> | xargs -r kill`. Example comm prefixes:
```
pkill -KILL ros_robot_contr    # ros_robot_controller_node
pkill -KILL usb_cam_node_ex    # usb_cam_node_exe
pkill -KILL ekf_node
pkill -KILL yolo_onnx_node
```

### 2.6 usb_cam swscaler warning (non-fatal)

**Symptom.** `[swscaler] No accelerated colorspace conversion found from
yuv422p to rgb24.` printed at startup.

**Root cause.** Known warning from usb_cam with `pixel_format: yuyv`.
Software fallback works correctly. Non-blocking.

### 2.7 Battery sag during extended sessions

**Symptom.** After ~30 min of testing, SSH connections started timing
out mid-channel (TCP/ping OK, but shell exec unresponsive). Latency
rose from ~18 ms to ~48 ms.

**Root cause.** 2S LiPo sag under sustained CPU load (HAL + YOLO
inference + camera). Battery was at 8.04 V early in session; likely
dropped below threshold during extended testing.

**Fix.** Power cycle and recharge between test sessions. Check
`hardware_watchdog` battery voltage before starting compute-heavy
workloads.

---

## 3. Partial test results (session interrupted by battery)

### 3.1 yolo_onnx_node standalone (PASS)

Launched `yolo_onnx_node` standalone against a running usb_cam:

- Model loaded successfully: `yolov8n_320.onnx`, input `images`
  `[1,3,320,320]`, output `output0`, CPUExecutionProvider
- **Inference rate: 6.2 FPS** (target was ≥3 Hz) — **EXCEEDED**
- Published `vision_msgs/Detection2DArray` on `/detections`
- Log confirmed detections with COCO class names and bounding boxes

### 3.2 depth_estimator_node (PARTIAL)

- Received `/camera/camera_info` (640×480, fx=473.5, fy=474.2)
- Published on `/detected_objects_3d` and `/vision_obstacles`
- Functional but not yet validated with simultaneous YOLO detections in
  the integrated launch

### 3.3 Integrated launch with enable_perception:=true (INCOMPLETE — session 1)

- `robot.launch.py enable_perception:=true` launched HAL stack (10
  processes) successfully
- Perception nodes (yolo_onnx_node, depth_estimator_node) and throttle
  node did not appear in process list
- Root cause: `docker exec -d` defaulted to root user; perception launch
  requires `-u ubuntu` to access the workspace overlays
- **Battery sag forced session end**

### 3.4 CPU saturation (sessions 2–4)

After fixing the docker user issue, the integrated launch (HAL + perception)
saturated the Pi 5's 4 cores to the point where SSH became completely
unresponsive. The startup burst — all 13+ processes loading Python imports,
ONNX model JIT, usb_cam colorspace init simultaneously — was the primary
trigger.

**Root cause.** yolo_onnx_node was processing every frame at 15 FPS.
Each inference takes ~107 ms, so YOLO alone consumed ~1.6 cores
continuously. Combined with HAL (usb_cam swscaler, EKF, IMU filter,
odom, controller), total CPU exceeded 100%.

**Fixes applied (cumulative):**
1. `num_threads: 2 → 1` — single ONNX inference thread
2. `framerate: 15 → 10` — camera at 10 FPS
3. `process_every_n: 3` — YOLO skips 2 of every 3 frames (~3.3 Hz input)
4. **Staggered launch** — HAL starts first, perception 20 s later (avoids
   startup burst)

### 3.5 Integrated launch — staggered (PASS — session 5)

With all four fixes applied, the full stack ran stably:

- HAL launched first (10 processes), load settled to ~1.0
- Perception added 20 s later (yolo_onnx_node, depth_estimator_node,
  throttle), load rose to ~1.3
- SSH remained responsive throughout
- All topics present and publishing

**Steady-state resource usage:**

| Process | CPU% | RAM (MB) |
|---|---|---|
| yolo_onnx_node | 36 | 183 |
| depth_estimator_node | 12 | 124 |
| usb_cam_node_exe | 5 | 84 |
| ros_robot_controller_node | 7 | 63 |
| odom_publisher_node | 5 | 61 |
| ekf_node | 5 | 29 |
| Other (imu, watchdog, etc.) | ~10 | ~100 |
| **Total** | **~80** | **~1209** |

System-level: load 1.3 (4 cores), temp 50.7°C, RAM 1209/4045 MB,
battery 8.06 V.

---

## 4. Acceptance tests — status

| # | Test | Status | Notes |
|---|---|---|---|
| T3 | yolo_onnx_node ≥3 Hz on Pi | **PASS** | 3.0–3.4 FPS integrated (avg inference 107 ms); 6.2 FPS standalone |
| T4 | /detections publishes Detection2DArray | **PASS** | Confirmed in standalone and integrated runs |
| T5 | /camera/demo/compressed at ~4 Hz | **PASS** | 3.3 Hz measured via `ros2 topic hz` |
| T6 | /vision_obstacles PointCloud2 | **PASS** | Topic present and publishing in integrated launch |

---

## 5. Lessons learned

1. **Always kill all processes on every Pi connect** — leftover processes
   from prior sessions cause cascading failures.
2. **Stagger launch on resource-constrained hardware** — simultaneous
   startup of 13+ ROS nodes overwhelms the Pi 5's scheduler; HAL first,
   then perception after settling.
3. **Frame skipping is essential** — processing every camera frame at
   10+ FPS is infeasible on the Pi; `process_every_n` reduces YOLO CPU
   from ~160% to ~36% of a core.
4. **`docker exec -d` runs as root by default** — must pass `-u ubuntu`
   to access the ROS2 workspace overlays.
