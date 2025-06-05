#!/bin/bash
# build_ros2_workspace.sh - Build ROS 2 workspace

echo "🔨 Building ROS 2 workspace..."

apt install colcon -y

cd ros2_ws


# Build with colcon
colcon build --packages-select my_robot_description my_robot_gazebo --symlink-install

echo "✅ ROS 2 workspace built successfully!"
echo "To use: source ros2_ws/install/setup.bash"
