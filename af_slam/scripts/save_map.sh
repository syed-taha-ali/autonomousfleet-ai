#!/usr/bin/env bash
# Save the current SLAM map to af_slam/maps/.
#
# Usage:
#   ./save_map.sh                     # saves as af_slam/maps/map
#   ./save_map.sh my_room             # saves as af_slam/maps/my_room
#   MAP_DIR=/tmp ./save_map.sh test   # saves to /tmp/test

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="${MAP_DIR:-$SCRIPT_DIR/../maps}"
MAP_NAME="${1:-map}"
OUTPUT="$MAP_DIR/$MAP_NAME"

mkdir -p "$MAP_DIR"

echo "Saving map to: $OUTPUT (.pgm + .yaml)"
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT" --ros-args -p save_map_timeout:=5.0

echo "Map saved:"
ls -lh "$OUTPUT".*
