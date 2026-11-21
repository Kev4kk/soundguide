#!/usr/bin/env bash
# Rebuild the checked-in Core ML model on macOS with uv and Xcode installed.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
cd "$work"

curl --fail --silent --show-error --location \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt \
  --output yolo26n.pt
printf '%s  %s\n' 9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef yolo26n.pt | shasum -a 256 --check

uv venv --python 3.12
uv pip install --python .venv/bin/python ultralytics==8.4.216 torch==2.7.0 \
  torchvision==0.22.0 coremltools==9.0 numpy==2.3.5
# Core ML NMS makes Vision return recognized object observations instead of raw tensors.
.venv/bin/yolo export model=yolo26n.pt format=coreml imgsz=640,480 nms=True conf=0.25 iou=0.45 device=cpu

# Replace the package as a whole; its manifest IDs can change between exports.
ditto yolo26n.mlpackage "$work/output/YOLO26n.mlpackage"
rm -rf "$root/SoundGuide/YOLO26n.mlpackage"
mv "$work/output/YOLO26n.mlpackage" "$root/SoundGuide/YOLO26n.mlpackage"
