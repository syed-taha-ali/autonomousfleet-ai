#!/usr/bin/env bash
# Staggered bringup for the MentorPi M1 inside the Docker container.
#
# The Pi 5's 4 cores cannot absorb the startup burst of HAL + perception
# launching simultaneously (13+ processes importing Python, loading ONNX
# model, opening /dev/video0 — all at once). This script starts HAL first,
# waits for it to settle, then adds perception on top.
#
# Usage (from the Pi host):
#   docker exec -d -u ubuntu -w /home/ubuntu MentorPi bash /tmp/start_robot.sh
#
# Or copy into the container first:
#   docker cp af_bringup/scripts/start_robot.sh MentorPi:/tmp/
#   docker exec -d -u ubuntu -w /home/ubuntu MentorPi bash /tmp/start_robot.sh
#
# Arguments are forwarded to robot.launch.py. Defaults:
#   enable_perception:=true  enable_demo_stream:=false
#
# Logs:
#   /tmp/robot_hal.log         — HAL + camera
#   /tmp/robot_perception.log  — yolo_onnx_node + depth_estimator_node
#   /tmp/robot_throttle.log    — topic_tools throttle (demo stream)
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/ubuntu/workspace/ros2_ws/install/local_setup.bash

SETTLE_S="${SETTLE_S:-20}"
DEMO_HZ="${DEMO_HZ:-4.0}"

# --- Phase 1: HAL + camera (no perception, no demo stream) ---------------
ros2 launch af_bringup robot.launch.py \
    enable_perception:=false \
    enable_demo_stream:=false \
    "$@" \
    > /tmp/robot_hal.log 2>&1 &
HAL_PID=$!
echo "[start_robot] HAL launched (PID $HAL_PID), settling for ${SETTLE_S}s ..."
sleep "$SETTLE_S"

# --- Phase 2: perception pipeline ----------------------------------------
ros2 launch af_perception perception_pi.launch.py \
    > /tmp/robot_perception.log 2>&1 &
PERC_PID=$!
echo "[start_robot] perception launched (PID $PERC_PID)"

# --- Phase 3: throttled demo stream for RViz on the Dev PC ---------------
ros2 run topic_tools throttle messages \
    /camera/image_raw/compressed "$DEMO_HZ" /camera/demo/compressed \
    > /tmp/robot_throttle.log 2>&1 &
THROT_PID=$!
echo "[start_robot] demo throttle launched (PID $THROT_PID) at ${DEMO_HZ} Hz"

wait $HAL_PID $PERC_PID $THROT_PID
