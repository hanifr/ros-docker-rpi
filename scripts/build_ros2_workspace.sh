#!/bin/bash
# build_ros2_workspace.sh - Build ROS 2 workspace

set -e  # Exit on error

echo "🔧 Updating package list..."
sudo apt update

echo "📦 Installing colcon and build tools..."
sudo apt install colcon -y

# Source ROS 2 setup (assuming Humble is installed)
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "🟢 Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS 2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Ensure ros2_ws exists
if [ ! -d "ros2_ws" ]; then
    echo "❌ 'ros2_ws' directory not found. Please make sure you're in the right place."
    exit 1
fi

cd ros2_ws

echo "🔨 Building ROS 2 workspace..."
colcon build --packages-select my_robot_description my_robot_gazebo --symlink-install

echo "✅ ROS 2 workspace built successfully!"
echo "👉 To use the workspace: source ros2_ws/install/setup.bash"
