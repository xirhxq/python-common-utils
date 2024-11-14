#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
docker run -it --rm -v "$SCRIPT_DIR/..":/app python_common_utils_ros_test
