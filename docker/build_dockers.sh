#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LIB_PREFIX="python-common-utils-ros"
docker build -f "$SCRIPT_DIR/Dockerfile-python38" -t "$LIB_PREFIX-python-test" "$SCRIPT_DIR"
docker build -f "$SCRIPT_DIR/Dockerfile-noetic" -t "$LIB_PREFIX-noetic-test" "$SCRIPT_DIR"
docker build -f "$SCRIPT_DIR/Dockerfile-foxy" -t "$LIB_PREFIX-foxy-test" "$SCRIPT_DIR"
