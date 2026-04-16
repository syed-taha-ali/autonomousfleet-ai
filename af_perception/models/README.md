# YOLO model files

Two runtime backends are supported, one per deployment topology:

## Phase 4 — Dev-PC (Ultralytics PyTorch CUDA)

`yolo_detector_node` loads `.pt` weights via Ultralytics. Drop a custom
`best.pt` into this directory and point the detector at it:

```bash
ros2 launch af_perception perception.launch.py \
    model_path:=$(ros2 pkg prefix af_perception)/share/af_perception/models/best.pt
```

Without a custom file it falls back to the Ultralytics zoo model named by
`model_name` (default `yolov5su.pt`, COCO-pretrained) which Ultralytics
fetches automatically on first run.

## Phase 4.1 — On-robot (ONNX Runtime, Pi 5 CPU)

`yolo_onnx_node` loads a YOLOv8n ONNX graph and runs it on the Pi with
the CPUExecutionProvider. The default filename the launcher looks for is
`yolov8n_320.onnx` in this directory.

### Exporting

Run the helper on the Dev PC (requires `ultralytics` + a copy of the base
YOLOv8n `.pt`, which Ultralytics fetches on first use):

```bash
./scripts/export_onnx.sh                  # → models/yolov8n_320.onnx
./scripts/export_onnx.sh best.pt          # custom weights → best_320.onnx
./scripts/export_onnx.sh best.pt 416      # different input size
```

The produced `.onnx` is ~12 MB (fp32). Commit it alongside the source so
the Pi container can `pip install --user onnxruntime` and start
detecting without needing ultralytics installed on-robot.

### Pointing the node at a custom ONNX

```bash
ros2 launch af_perception perception_pi.launch.py \
    model_path:=/home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai/af_perception/models/best_320.onnx
```

If `model_path` is empty the node falls back to
`<package_share>/models/<model_name>` (default `yolov8n_320.onnx`).
