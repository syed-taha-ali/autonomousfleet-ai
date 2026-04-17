#!/usr/bin/env bash
# Launch the NLP interface on the Dev PC.
#
# Starts nlp_command_node (Ollama LLM translator) and optionally the
# interactive CLI in a second terminal.
#
# Usage:
#   ./start_nlp.sh              # NLP node only (use CLI separately)
#   ./start_nlp.sh --with-cli   # Also opens CLI in a new terminal
#
# Prerequisites:
#   - Ollama running locally (ollama serve)
#   - Model pulled (ollama pull qwen2.5:7b-instruct-q4_K_M)
#   - Robot stack already running on the Pi (start_pi.sh)
#
# To send commands manually:
#   ros2 run af_nlp cli
#   > find the suitcase
#   > go to the door
#   > stop

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/../../../.."
LOG_DIR="${SCRIPT_DIR}/../../logs/phase5b"

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/nl_log.jsonl"

# Check Ollama is running
if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "[start_nlp] ERROR: Ollama is not running. Start it with: ollama serve"
    exit 1
fi

echo "[start_nlp] Launching NLP command node (model=qwen2.5:7b-instruct-q4_K_M)"
echo "[start_nlp] Log file: ${LOG_FILE}"
echo ""

if [[ "${1:-}" == "--with-cli" ]]; then
    ros2 launch af_nlp nlp.launch.py log_file:="${LOG_FILE}" &
    NLP_PID=$!
    sleep 3
    echo "[start_nlp] Opening CLI..."
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && ros2 run af_nlp cli; exec bash"
    elif command -v xterm &> /dev/null; then
        xterm -e "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && ros2 run af_nlp cli" &
    else
        echo "[start_nlp] No terminal emulator found. Open a new terminal and run:"
        echo "  source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && ros2 run af_nlp cli"
    fi
    wait $NLP_PID
else
    ros2 launch af_nlp nlp.launch.py log_file:="${LOG_FILE}"
fi
