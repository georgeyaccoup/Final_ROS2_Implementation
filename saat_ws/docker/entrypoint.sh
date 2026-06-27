#!/bin/bash
# SAAT Docker entrypoint
# Sources ROS2 Foxy and the built workspace overlay, then executes CMD.

set -e

# Source ROS2 Foxy base
source /opt/ros/foxy/setup.bash

# Source the SAAT workspace overlay
if [ -f "/saat_ws/install/setup.bash" ]; then
    source /saat_ws/install/setup.bash
fi

# Execute the provided command (default: ros2 launch saat_core saat_launch.py)
exec "$@"
