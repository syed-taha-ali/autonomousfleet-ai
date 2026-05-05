#!/usr/bin/env bash
# Launch the full robot stack on the Pi from the Dev PC via SSH.
#
# This script SSHs into the Pi, enters the MentorPi Docker container,
# kills any leftover ROS processes, sources all three workspaces, and
# launches explore.launch.py with the given arguments.
#
# Usage (from Dev PC):
#   ./start_pi.sh                          # Full stack with auto-explorer
#   ./start_pi.sh enable_explore:=false    # Full stack, NLP-controlled (no auto-explorer)
#   ./start_pi.sh enable_perception:=false # No YOLO (lighter CPU)
#
# The launch log is written to /tmp/explore.log inside the container.
# Monitor it with:
#   ssh pi@192.168.149.1 "docker exec -u ubuntu MentorPi tail -f /tmp/explore.log"

set -euo pipefail

PI_HOST="${PI_HOST:-pi@192.168.149.1}"
CONTAINER="${CONTAINER:-MentorPi}"
LAUNCH_ARGS="${*:-}"

echo "[start_pi] Connecting to ${PI_HOST}..."

# Step 1: Kill all leftover ROS/Python processes inside the container
ssh "${PI_HOST}" "docker exec ${CONTAINER} pkill -9 -f ros2 2>/dev/null; \
                  docker exec ${CONTAINER} pkill -9 -f python3 2>/dev/null" || true
sleep 2
echo "[start_pi] Cleaned old processes"

# Step 2: Launch explore stack (detached) with all three workspaces sourced
#   - /opt/ros/humble                                    (ROS 2 base)
#   - /home/ubuntu/ros2_ws/install                       (HiWonder stock packages)
#   - /home/ubuntu/third_party_ros2/third_party_ws/install (oradar_lidar, etc.)
#   - /home/ubuntu/workspace/ros2_ws/install             (our af_* packages)
ssh "${PI_HOST}" "docker exec -d -u ubuntu -w /home/ubuntu/workspace/ros2_ws ${CONTAINER} bash -c '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/local_setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/local_setup.bash
source /home/ubuntu/workspace/ros2_ws/install/local_setup.bash
ros2 launch af_bringup explore.launch.py ${LAUNCH_ARGS} > /tmp/explore.log 2>&1
'"

echo "[start_pi] Explore stack launched (detached)"
echo "[start_pi] Staggered bringup timeline:"
echo "           t=0s   HAL + LiDAR + camera"
echo "           t=5s   SLAM (slam_toolbox)"
echo "           t=12s  Nav2"
echo "           t=15s  Mission control"
echo "           t=22s  Perception (YOLO)"
echo "           t=25s  Explorer (if enabled)"
echo ""
echo "[start_pi] Waiting 35s for full bringup..."
sleep 35

# Step 3: Verify critical nodes are running
NODES=$(ssh "${PI_HOST}" "docker exec -u ubuntu ${CONTAINER} bash -c \
    'source /opt/ros/humble/setup.bash && \
     source /home/ubuntu/workspace/ros2_ws/install/local_setup.bash && \
     ros2 node list 2>/dev/null'" 2>/dev/null || echo "")

EXPECTED_NODES=("mission_manager" "bt_navigator" "slam_toolbox" "controller_server")
ALL_OK=true
for node in "${EXPECTED_NODES[@]}"; do
    if echo "$NODES" | grep -q "$node"; then
        echo "[start_pi] ✓ ${node}"
    else
        echo "[start_pi] ✗ ${node} NOT FOUND"
        ALL_OK=false
    fi
done

if $ALL_OK; then
    echo ""
    echo "[start_pi] Robot stack is ready."
else
    echo ""
    echo "[start_pi] WARNING: Some nodes missing. Check /tmp/explore.log on the Pi."
fi
