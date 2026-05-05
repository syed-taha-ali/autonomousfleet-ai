#!/usr/bin/env bash
# Kill all ROS2 / monitoring / NLP processes on the Dev PC.
# Run this before any bringup to guarantee a clean slate.

echo "[nuke] Killing all Dev PC ROS processes..."

pkill -9 -f "nlp_command_node"      2>/dev/null || true
pkill -9 -f "perf_monitor_node"     2>/dev/null || true
pkill -9 -f "detection_overlay_node" 2>/dev/null || true
pkill -9 -f "rviz2"                 2>/dev/null || true
pkill -9 -f "rosbag2"               2>/dev/null || true
pkill -9 -f "ros2 launch"           2>/dev/null || true
pkill -9 -f "ros2 run"              2>/dev/null || true

sleep 1

REMAINING=$(pgrep -af "ros2\|rviz2\|nlp_command_node\|perf_monitor\|detection_overlay\|rosbag2" \
            | grep -v "grep\|claude\|code\|snap\|networkd\|unattended" || true)

if [[ -n "$REMAINING" ]]; then
    echo "[nuke] WARNING: some processes still alive:"
    echo "$REMAINING"
else
    echo "[nuke] Clean — no ROS processes running."
fi
