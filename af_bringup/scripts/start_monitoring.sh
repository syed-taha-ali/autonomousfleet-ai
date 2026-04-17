#!/usr/bin/env bash
# Launch the Dev PC monitoring stack: RViz2 + perf_monitor + rosbag recording.
#
# Usage (from Dev PC):
#   ./start_monitoring.sh                    # Full view + recording
#   ./start_monitoring.sh --mode navigation  # Nav-specific RViz
#   ./start_monitoring.sh --no-record        # Skip rosbag recording
#   ./start_monitoring.sh --no-rviz          # Perf monitor + recording only

set -eo pipefail

# RViz2 crashes if snap's libpthread leaks into LD_LIBRARY_PATH.
unset LD_LIBRARY_PATH 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/../../../.."
RVIZ_MODE="full"
DO_RECORD=true
DO_RVIZ=true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)       RVIZ_MODE="$2"; shift 2 ;;
        --no-record)  DO_RECORD=false; shift ;;
        --no-rviz)    DO_RVIZ=false; shift ;;
        *)            echo "Unknown arg: $1"; exit 1 ;;
    esac
done

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

BAG_DIR="${SCRIPT_DIR}/../../rosbags/$(date +%Y%m%d_%H%M%S)"

echo "[monitoring] Starting monitoring stack on Dev PC"
echo "             RViz mode : ${RVIZ_MODE} (rviz=$DO_RVIZ)"
echo "             Recording : ${DO_RECORD}"
if $DO_RECORD; then
    echo "             Bag dir   : ${BAG_DIR}"
fi
echo ""

PIDS=()

ros2 launch af_monitoring monitor.launch.py &
PIDS+=($!)
echo "[monitoring] perf_monitor started (PID ${PIDS[-1]})"

ros2 run af_monitoring detection_overlay_node &
PIDS+=($!)
echo "[monitoring] detection_overlay started (PID ${PIDS[-1]})"

if $DO_RVIZ; then
    RVIZ_CFG="$(ros2 pkg prefix af_monitoring)/share/af_monitoring/config/${RVIZ_MODE}.rviz"
    if [[ ! -f "${RVIZ_CFG}" ]]; then
        echo "[monitoring] ERROR: RViz config not found: ${RVIZ_CFG}"
        exit 1
    fi
    # env -i clears the snap-contaminated environment that causes a glibc
    # symbol conflict with rviz2 on Ubuntu 22.04 (VS Code snap sets vars
    # that leak /snap/core20 libpthread into the linker search path).
    env -i HOME="$HOME" DISPLAY="${DISPLAY:-:0}" \
        XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
        XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
        bash -c "source /opt/ros/humble/setup.bash && \
                 source ${WS_DIR}/install/setup.bash && \
                 rviz2 -d '${RVIZ_CFG}'" &
    PIDS+=($!)
    echo "[monitoring] RViz2 started (PID ${PIDS[-1]})"
fi

if $DO_RECORD; then
    mkdir -p "${BAG_DIR}"
    ros2 launch af_monitoring record_session.launch.py bag_dir:="${BAG_DIR}" &
    PIDS+=($!)
    echo "[monitoring] Rosbag recording started (PID ${PIDS[-1]})"
fi

echo ""
echo "[monitoring] All components running. Press Ctrl+C to stop."

cleanup() {
    echo ""
    echo "[monitoring] Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[monitoring] Done."
}
trap cleanup INT TERM

wait
