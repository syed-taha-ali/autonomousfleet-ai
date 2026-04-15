# Custom YOLO weights

Drop `best.pt` (or any Ultralytics-compatible `.pt`) in this directory and
point `yolo_detector_node` at it via the `model_path` parameter:

```bash
ros2 launch af_perception perception.launch.py \
    model_path:=$(ros2 pkg prefix af_perception)/share/af_perception/models/best.pt
```

Without a custom file, `yolo_detector_node` falls back to the Ultralytics
zoo model named by `model_name` (default `yolov5su.pt`, COCO-pretrained)
which Ultralytics fetches automatically on first run.
