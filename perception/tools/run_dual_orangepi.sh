#!/bin/sh
set -eu

project_directory=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
vision_python=${VISION_PYTHON:-/home/orangepi/venvs/yolo-rknn-board/bin/python}
if [ ! -x "$vision_python" ]; then
    vision_python=python3
fi

cd "$project_directory"
run_directory="$project_directory/recordings/dual_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run_directory"
exec "$vision_python" -m rescue_vision.dashboard \
    --config "$project_directory/config/dual_orangepi.json" \
    --host 0.0.0.0 --port 8080 --cameras \
    --record "$run_directory/observations.jsonl" "$@"
