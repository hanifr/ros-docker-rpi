#!/bin/bash
# scripts/start_robot_system.sh - Main robot startup script

set -e
source /opt/ros/humble/setup.bash

echo "🚀 Starting Robot System..."

# Build workspace if needed
if [ -d /workspace/src ] && [ ! -d /workspace/install ]; then
    echo "🔨 Building ROS 2 workspace..."
    cd /workspace
    colcon build --packages-select my_robot_description
fi

# Source workspace if built
if [ -d /workspace/install ]; then
    echo "✅ Sourcing workspace"
    cd /workspace
    source install/setup.bash
fi

# Load robot URDF
echo "🤖 Loading robot URDF..."
ROBOT_URDF_FILE="/workspace/src/my_robot_description/urdf/robot.urdf"

if [ -f "$ROBOT_URDF_FILE" ]; then
    echo "✅ Found robot URDF: $ROBOT_URDF_FILE"
    ros2 param set robot_description "$(cat $ROBOT_URDF_FILE)"
    echo "✅ Robot description loaded"
else
    echo "❌ Robot URDF not found: $ROBOT_URDF_FILE"
    echo "📁 Available URDF files:"
    find /workspace -name "*.urdf" -type f
    exit 1
fi

# Start robot_state_publisher
echo "🔗 Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher &
RSP_PID=$!

# Start joint_state_publisher  
echo "🦾 Starting joint_state_publisher..."
ros2 run joint_state_publisher joint_state_publisher &
JSP_PID=$!

# Start ROSBridge
echo "🔌 Starting ROSBridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
ROSBRIDGE_PID=$!

# Start virtual robot
echo "🎮 Starting virtual robot..."
python3 /scripts/virtual_robot.py &
ROBOT_PID=$!

echo "✅ All services started!"
echo "  - robot_state_publisher (PID: $RSP_PID)"
echo "  - joint_state_publisher (PID: $JSP_PID)" 
echo "  - rosbridge_server (PID: $ROSBRIDGE_PID)"
echo "  - virtual_robot (PID: $ROBOT_PID)"

echo "🎉 differential_drive_robot ready!"

# Keep running
tail -f /dev/null