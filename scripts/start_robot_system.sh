#!/bin/bash
# scripts/start_robot_system.sh - Fixed version

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

# Load robot URDF with proper error handling
echo "🤖 Loading robot URDF..."
ROBOT_URDF_FILE="/workspace/src/my_robot_description/urdf/robot.urdf"

if [ -f "$ROBOT_URDF_FILE" ]; then
    echo "✅ Found robot URDF: $ROBOT_URDF_FILE"
    
    # Check if file has content
    if [ -s "$ROBOT_URDF_FILE" ]; then
        echo "📏 URDF file size: $(du -h $ROBOT_URDF_FILE | cut -f1)"
        
        # Read URDF content and set parameter
        URDF_CONTENT=$(cat "$ROBOT_URDF_FILE")
        
        if [ -n "$URDF_CONTENT" ]; then
            echo "🔧 Setting robot_description parameter..."
            ros2 param set robot_description "$URDF_CONTENT"
            echo "✅ Robot description loaded successfully"
        else
            echo "❌ URDF file is empty"
            exit 1
        fi
    else
        echo "❌ URDF file exists but is empty"
        exit 1
    fi
else
    echo "❌ Robot URDF not found: $ROBOT_URDF_FILE"
    echo "📁 Available URDF files:"
    find /workspace -name "*.urdf" -type f 2>/dev/null || echo "No URDF files found"
    exit 1
fi

# Wait a moment for parameter to be set
sleep 2

# Start robot_state_publisher
echo "🔗 Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher &
RSP_PID=$!

# Wait for robot_state_publisher to start
sleep 3

# Start joint_state_publisher  
echo "🦾 Starting joint_state_publisher..."
ros2 run joint_state_publisher joint_state_publisher &
JSP_PID=$!

# Start ROSBridge
echo "🔌 Starting ROSBridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
ROSBRIDGE_PID=$!

# Wait for ROSBridge to start
sleep 5

# Start virtual robot
echo "🎮 Starting virtual robot..."
python3 /scripts/virtual_robot.py &
ROBOT_PID=$!

echo "✅ All services started!"
echo "  - robot_state_publisher (PID: $RSP_PID)"
echo "  - joint_state_publisher (PID: $JSP_PID)" 
echo "  - rosbridge_server (PID: $ROSBRIDGE_PID)"
echo "  - virtual_robot (PID: $ROBOT_PID)"

echo ""
echo "🔍 Checking system status..."

# Check if robot_description parameter was set
sleep 2
if ros2 param list 2>/dev/null | grep -q robot_description; then
    echo "✅ robot_description parameter confirmed"
else
    echo "⚠️ robot_description parameter not found"
fi

# Check ROS topics
echo "📡 Available topics:"
ros2 topic list | head -10

echo ""
echo "🎉 differential_drive_robot system ready!"

# Keep running
tail -f /dev/null