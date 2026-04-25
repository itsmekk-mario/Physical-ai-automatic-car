#!/usr/bin/env bash
set -euo pipefail

DATA_YAML="${1:-/home/kown/jetcar_ws/datasets/jetcar_lane_dual/lane.yaml}"
MODEL="${MODEL:-yolov8n-seg.pt}"
EPOCHS="${EPOCHS:-30}"
IMGSZ="${IMGSZ:-320}"
BATCH="${BATCH:-2}"
DEVICE="${DEVICE:-0}"
PROJECT="${PROJECT:-/home/kown/jetcar_ws/runs/segment}"
NAME="${NAME:-jetcar_lane_dual}"

yolo segment train \
  model="${MODEL}" \
  data="${DATA_YAML}" \
  epochs="${EPOCHS}" \
  imgsz="${IMGSZ}" \
  batch="${BATCH}" \
  device="${DEVICE}" \
  project="${PROJECT}" \
  name="${NAME}" \
  exist_ok=True \
  workers=2 \
  cache=False
