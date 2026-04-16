# Phase 4 — Distributed perception pipeline

End-to-end walkthrough for the Phase 4 perception stack. The robot (Pi) only
has to keep publishing its existing compressed camera topic; all of the heavy
lifting — YOLO inference, 3D back-projection, costmap observation — happens
on the Dev PC so the Pi 5's 4 GB RAM stays free for Nav2 + SLAM + HAL.

```
                               WiFi (2-4 Mbps)
  ┌──────── Pi ────────┐     /camera/image_raw/compressed       ┌──── Dev PC ───┐
  │ usb_cam_node       │ ──────────────────────────────────────▶│ decompressor  │
  │ (yuyv, 15 FPS,     │     /camera/camera_info                │ yolo_detector │
  │  JPEG transport)   │ ──────────────────────────────────────▶│ depth_estim.  │
  └────────────────────┘                                        └──────┬────────┘
          ▲                                                            │
          │ /vision_obstacles (sensor_msgs/PointCloud2)                │
          └────────────────────────────────────────────────────────────┘
                (stock Nav2 ObstacleLayer on the Pi consumes it)
```

No custom C++ costmap plugin is written. The local costmap is already wired
for `vision` in `af_navigation/config/nav2_params.yaml` (Phase 3); Phase 4
just supplies the publisher.

---

## 1. What Phase 4 adds

`af_perception` — Dev-PC pipeline, three Python nodes:

| File | Purpose |
|---|---|
| `af_perception/af_perception/image_decompressor_node.py` | CompressedImage → Image (bgr8) via cv_bridge |
| `af_perception/af_perception/yolo_detector_node.py`      | Ultralytics YOLO on CUDA → vision_msgs/Detection2DArray |
| `af_perception/af_perception/depth_estimator_node.py`    | Detection2DArray → Detection3DArray + /vision_obstacles PointCloud2 |
| `af_perception/launch/perception.launch.py`              | Runs all three nodes on the Dev PC |
| `af_perception/config/perception.yaml`                   | Default topic/parameter configuration |
| `af_perception/models/`                                  | Drop a custom-trained `best.pt` here (Ultralytics zoo used by default) |

`af_bringup/launch/robot.launch.py` — `usb_cam` now publishes on
`/camera/image_raw{/compressed,/theora}` with a `camera_info_url` pointing at
`af_bringup/config/camera_info.yaml` so CameraInfo is emitted at boot.

---

## 2. Prerequisites

**Pi** (already provided by Phase 1):
* `af_bringup/robot.launch.py` running — publishes `/camera/image_raw/compressed` + `/camera/camera_info`.
* WiFi routing DDS traffic cleanly between Pi and Dev PC (same `ROS_DOMAIN_ID`, multicast reachable, or a configured `FASTRTPS_DEFAULT_PROFILES_FILE`).

**Dev PC** — one-time setup:

```bash
# ROS dependencies
sudo apt install -y \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-tf2-geometry-msgs \
    python3-opencv

# Ultralytics (YOLO) + PyTorch with CUDA — on the RTX 3060 host
pip install --upgrade ultralytics
# If PyTorch wasn't installed by ultralytics with CUDA, install explicitly:
# pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

Verify GPU availability:

```bash
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# Expected: True NVIDIA GeForce RTX 3060
```

Check Ultralytics can auto-fetch zoo weights (first run caches them):

```bash
python3 -c "from ultralytics import YOLO; YOLO('yolov5su.pt')"
```

Build:

```bash
cd ~/ros2_ws
colcon build --packages-select af_bringup af_perception
source install/setup.bash
```

---

## 3. Camera calibration

The file shipped at `af_bringup/config/camera_info.yaml` has working defaults
pulled from the matching HiWonder module, but each camera differs. Run a fresh
calibration before trusting `depth_estimator_node` 3D positions:

```bash
# On the Dev PC (view the Pi's image)
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 \
    image:=/camera/image_raw camera:=/camera
# Click "Commit" when the bars are all green; then copy the saved YAML into
# af_bringup/config/camera_info.yaml and rebuild af_bringup.
```

---

## 4. Bring up the Pi side

On the Pi, inside the MentorPi container:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_ws/install/local_setup.bash
ros2 launch af_bringup robot.launch.py
```

Check the compressed stream is alive from the Dev PC:

```bash
ros2 topic hz /camera/image_raw/compressed
# Expected: ~15 Hz
ros2 topic echo --once /camera/camera_info | head
```

---

## 5. Bring up the Dev PC pipeline

```bash
# ground-plane mode works with just the USB camera (no depth sensor)
ros2 launch af_perception perception.launch.py mode:=ground

# depth mode once the depth-camera driver is publishing /camera/depth/image_raw
ros2 launch af_perception perception.launch.py mode:=depth

# Supply custom weights
ros2 launch af_perception perception.launch.py \
    model_path:=/abs/path/to/best.pt

# CPU fallback when running on a laptop without CUDA
ros2 launch af_perception perception.launch.py device:=cpu
```

Expected log lines:

```
[image_decompressor_node] decompressed 75 frames (15.0 FPS), drops=0
[yolo_detector_node]      yolo: 22.4 FPS, avg inference 11.2 ms (cuda:0)
[depth_estimator_node]    depth_estimator: ground-plane mode (fallback_h=0.15 m, ...)
```

---

## 6. Acceptance tests

All tests run on the Dev PC unless noted.

### T1 — Compressed stream end-to-end

```bash
ros2 topic bw /camera/image_raw/compressed
ros2 topic hz /camera/image_raw/compressed
```

Pass: ≥10 Hz, bandwidth ≤ 6 Mbps. The HiWonder USB camera's V4L2 driver is
the actual bottleneck on this hardware — the MentorPi usb_cam node uses
<1% CPU even while nominally configured at 15 FPS, so the camera itself
only sources ~10 FPS. JPEG at q=80 yields ~67 KiB/frame → ~5.9 Mbps. This
is well above the LiDAR's 7-10 Hz refresh, so Nav2 obstacle avoidance
consumes vision updates at a rate that matches the costmap cycle.

### T2 — YOLO throughput (≥20 FPS on RTX 3060)

Check the `yolo_detector_node` stats line emitted every 5 s. Pass: ≥20 FPS
avg, inference ≤ 20 ms/frame on CUDA.

### T3 — 3D position accuracy (≤15 cm at 1 m)

Place a known object (e.g. a chair) at a tape-measured 1.00 m in front of
`base_link`. Capture its 3D centre:

```bash
ros2 topic echo /detected_objects_3d --once
```

Pass: `|z - 1.00| ≤ 0.15 m`, `|x| ≤ 0.05 m`.

### T4 — Nav2 ObstacleLayer consumes /vision_obstacles

```bash
# On the Pi
ros2 launch af_bringup nav2.launch.py   # Phase 3 stack
```

Open RViz2 on the Dev PC with the `local_costmap/costmap` topic enabled and
place a vision-only obstacle (e.g. a cardboard box short enough that LiDAR
misses it — below the MS200 scan plane). The local costmap should mark a
cluster at its position within ~1 s.

Pass: marked cells visible in RViz; `ros2 topic echo /local_costmap/costmap`
shows non-zero cells at the obstacle location.

### T5 — Vision-only obstacle avoidance

Point-goal navigation past a vision-only obstacle. Send a goal with the `2D
Goal Pose` RViz tool that would otherwise drive through the obstacle.

Pass: robot routes around it. Compare against the same scene with perception
stopped — robot either collides or the LiDAR misses the obstacle entirely.

### T6 — Graceful degradation (link drop)

While Nav2 is executing a goal, kill the perception stack on the Dev PC
(`Ctrl-C`). Confirm:
* Nav2 does **not** crash.
* Controller keeps running on LiDAR-only obstacle avoidance.
* `ros2 topic hz /vision_obstacles` returns `no new messages` within ~2 s.

Pass: robot completes the goal on LiDAR alone, no SIGABRT from
`bt_navigator` or `controller_server`.

---

## 7. Known gotchas

* **`vision_msgs` not installed**. `apt install ros-humble-vision-msgs` on
  the Dev PC — the package is not pulled in by `ros-humble-desktop`.
* **Ultralytics first run downloads weights** (~15 MB for YOLOv5su) under
  `~/.config/Ultralytics/`. If the Dev PC is air-gapped, ship the `.pt`
  yourself and set `model_path`.
* **`cv_bridge` on compressedDepth**. MentorPi's depth-camera driver
  publishes `/camera/depth/image_raw/compressedDepth`. Current Phase 4 only
  consumes the raw `/camera/depth/image_raw` topic — if you want to pull
  depth over WiFi too, add a second `image_decompressor_node` with
  `output_encoding: 16UC1` remapped onto the depth topics.
* **TF lookup failure**. `depth_estimator_node` warns
  `TF camera_color_optical_frame -> base_link unavailable` and falls back to
  publishing `/vision_obstacles` in the optical frame. Nav2's ObstacleLayer
  will then transform the cloud using `tf_tolerance`, but positions will be
  noisy until the correct TF is published (either by the depth-camera
  driver or by a static_transform_publisher added to `robot.launch.py`).
* **`classes` filter is a list of integers** matching the model's class
  index, not names. With the stock COCO model, `0 = person`. Never write
  `classes: []` in YAML — rclpy can't infer the type from an empty list
  and the node raises `ParameterUninitializedException`. Omit the key to
  mean "all classes".
* **cv_bridge compressed decoder returns 8UC1 sometimes**. `cv_bridge`
  0.x in Humble occasionally hands back a single-channel buffer from
  `compressed_imgmsg_to_cv2(..., desired_encoding='bgr8')`. The
  decompressor sidesteps this by going straight through
  `cv2.imdecode(..., IMREAD_COLOR)` and then wrapping the resulting BGR
  array with `cv2_to_imgmsg`. If you see `encoding specified as bgr8, but
  image has incompatible type 8UC1`, that's the bug — keep imdecode.
* **usb_cam camera frame param is `frame_id`, not `camera_frame_id`**.
  Humble's usb_cam 0.6 ignores `camera_frame_id` silently and tags every
  frame with `frame_id: default_cam`, which means no TF exists to
  transform `/vision_obstacles` from the camera into `base_link`. The
  bringup launch sets `frame_id: depth_cam` — the URDF already publishes
  a rigid `base_link -> depth_cam` static TF, and the USB + depth sensors
  are colocated on the front plate, so reusing that frame is accurate to
  within a couple of centimetres. If the depth camera is later mounted
  separately, add a dedicated `camera_link` joint in
  `af_description/urdf/` and change this param to match.
* **QoS mismatch eats frames**. Set the compressed-image subscriber to
  BEST_EFFORT + KEEP_LAST depth=10. Going tighter (depth=1-2) causes DDS
  to discard incoming fragments before the frame is fully reassembled
  and you see long 0-FPS stretches. The publisher side of
  `image_transport`'s compressed plugin is RELIABLE by default; the
  current decompressor's BE + depth=10 combination is the proven one.
* **Battery sag masquerades as a QoS bug**. During long validation
  sessions, the MentorPi 2S pack drains enough under continuous camera +
  HAL load (no motor usage) that WiFi drops out — SSH times out and all
  ROS 2 topics hit 0 FPS. Before chasing a software fix, check
  reachability (`ping`) and the `hardware_watchdog` "battery X.XX V"
  log. Anything < 7.4 V on a 2S pack is suspect. For multi-hour sessions
  run the robot on the charger.
