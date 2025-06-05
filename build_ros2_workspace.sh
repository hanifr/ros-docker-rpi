#!/bin/bash
# build_ros2_workspace.sh - Build ROS 2 workspace

set -e  # Exit on error

echo "🔧 Updating package list..."
sudo apt update

echo "📦 Installing colcon and build tools..."
sudo apt install colcon -y

cd ros2_ws

echo "🔨 Building ROS 2 workspace..."
colcon build --packages-select my_robot_description my_robot_gazebo --symlink-install

echo "✅ ROS 2 workspace built successfully!"
echo "👉 To use the workspace: source ros2_ws/install/setup.bash"
