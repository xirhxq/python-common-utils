#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
docker build -t python_common_utils_ros_test "$SCRIPT_DIR"
