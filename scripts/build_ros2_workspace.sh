#!/bin/bash
# build_ros2_workspace.sh - Build ROS 2 workspace

echo "🔨 Building ROS 2 workspace..."

cd ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build with colcon
colcon build --symlink-install \
             --cmake-args -DCMAKE_BUILD_TYPE=Release \
             --parallel-workers 2 \
             --event-handlers console_direct+

echo "✅ ROS 2 workspace built successfully!"
echo "To use: source ros2_ws/install/setup.bash"
