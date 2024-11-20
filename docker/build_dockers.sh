#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
IMAGE_PREFIX="python-common-utils"
IMAGE_TAGS=("python38" "noetic" "foxy")

for TAG in "${IMAGE_TAGS[@]}"; do
  docker build -f "$SCRIPT_DIR/Dockerfile-${TAG}" -t "${IMAGE_PREFIX}:${TAG}" "$SCRIPT_DIR"
done