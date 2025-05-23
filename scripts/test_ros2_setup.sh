#!/bin/bash
# test_ros2_setup.sh - Test ROS 2 installation and workspace

echo "🔧 Testing ROS 2 Setup..."

# Test ROS 2 installation
echo "1. Testing ROS 2 installation..."
ros2 --version

echo "2. Testing ROS 2 workspace..."
cd ros2_ws
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo "✅ ROS 2 workspace built and ready"
    
    echo "3. Listing available packages..."
    ros2 pkg list | grep my_robot || echo "⚠️  No my_robot packages found - build workspace first"
else
    echo "❌ ROS 2 workspace not built - run ./scripts/build_ros2_workspace.sh first"
fi

echo "🎉 ROS 2 setup test completed!"
