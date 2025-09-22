#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
IMAGE_PREFIX_LOCAL="python-common-utils"
IMAGE_PREFIX_REMOTE="xirhxq/python-common-utils"
IMAGE_TAGS=("python38" "noetic" "foxy")

USE_ONLINE=false

if [[ "$1" == "--online" ]]; then
  USE_ONLINE=true
fi

for TAG in "${IMAGE_TAGS[@]}"; do
  if $USE_ONLINE; then
    IMAGE="${IMAGE_PREFIX_REMOTE}:${TAG}"
  else
    IMAGE="${IMAGE_PREFIX_LOCAL}:${TAG}"
  fi
  docker run --rm -v "$SCRIPT_DIR/..":/app "$IMAGE"
done