#!/usr/bin/env bash
# Export a YOLOv8 `.pt` to an ONNX graph ready for yolo_onnx_node on the Pi.
#
# Usage:
#   ./scripts/export_onnx.sh                  # yolov8n.pt @ imgsz 320
#   ./scripts/export_onnx.sh best.pt          # custom .pt @ imgsz 320
#   ./scripts/export_onnx.sh best.pt 416      # custom .pt @ imgsz 416
#
# Output lands in `<repo>/af_perception/models/<stem>_<imgsz>.onnx`. Run on
# the Dev PC (requires `pip install ultralytics`). Ultralytics will fetch
# the base yolov8n.pt on first use and cache it under ~/.config/Ultralytics.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODELS_DIR="${PKG_DIR}/models"

WEIGHTS="${1:-yolov8n.pt}"
IMGSZ="${2:-320}"

if ! command -v yolo >/dev/null 2>&1; then
    echo "error: ultralytics CLI 'yolo' not found. Install with:" >&2
    echo "    pip install ultralytics" >&2
    exit 1
fi

mkdir -p "${MODELS_DIR}"

# ultralytics drops the ONNX next to the source .pt, so run the export in a
# temp dir then move the result into af_perception/models/ with a filename
# that encodes the input size (the node parser keys off `_<imgsz>.onnx`).
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT

if [[ -f "${WEIGHTS}" ]]; then
    cp "${WEIGHTS}" "${WORK_DIR}/"
    STEM="$(basename "${WEIGHTS}" .pt)"
else
    STEM="$(basename "${WEIGHTS}" .pt)"
fi

pushd "${WORK_DIR}" >/dev/null
echo ">>> exporting ${WEIGHTS} -> ONNX (imgsz=${IMGSZ}, opset=17, simplify)"
yolo export model="${WEIGHTS}" format=onnx imgsz="${IMGSZ}" opset=17 simplify=True
popd >/dev/null

SRC="${WORK_DIR}/${STEM}.onnx"
DST="${MODELS_DIR}/${STEM}_${IMGSZ}.onnx"
if [[ ! -f "${SRC}" ]]; then
    echo "error: expected ${SRC} but ultralytics did not produce it" >&2
    exit 1
fi
mv "${SRC}" "${DST}"
echo ">>> ${DST} ($(du -h "${DST}" | cut -f1))"
