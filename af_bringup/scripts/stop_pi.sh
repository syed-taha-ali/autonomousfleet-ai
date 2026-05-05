#!/usr/bin/env bash
# Kill all ROS processes on the Pi and verify clean state.
#
# Usage (from Dev PC):
#   ./stop_pi.sh

set -euo pipefail

PI_HOST="${PI_HOST:-pi@192.168.149.1}"
CONTAINER="${CONTAINER:-MentorPi}"

echo "[stop_pi] Killing all ROS processes on ${PI_HOST}..."
ssh "${PI_HOST}" "docker exec ${CONTAINER} pkill -9 -f ros2 2>/dev/null" || true
ssh "${PI_HOST}" "docker exec ${CONTAINER} pkill -9 -f python3 2>/dev/null" || true
sleep 2

REMAINING=$(ssh "${PI_HOST}" "docker exec ${CONTAINER} ps aux 2>/dev/null" | grep -cE "ros2|python3" | grep -v grep || echo "0")
echo "[stop_pi] Remaining ROS/Python processes: ${REMAINING}"
echo "[stop_pi] Done."
