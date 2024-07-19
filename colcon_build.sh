#!/bin/bash
set -e

# Clear docker logs
# sudo sh -c "truncate -s 0 /var/lib/docker/containers/*/*-json.log"

source /opt/ros/humble/setup.sh

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic

source install/setup.sh