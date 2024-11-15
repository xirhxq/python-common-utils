#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LIB_PREFIX="python-common-utils-ros"
docker run -it --rm -v "$SCRIPT_DIR/..":/app "$LIB_PREFIX-python-test"
docker run -it --rm -v "$SCRIPT_DIR/..":/app "$LIB_PREFIX-noetic-test"
docker run -it --rm -v "$SCRIPT_DIR/..":/app "$LIB_PREFIX-foxy-test"
