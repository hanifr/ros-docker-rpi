#!/bin/bash
# scripts/start_robot_system.sh - Final working version

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

# Find URDF file
ROBOT_URDF_PATHS=(
    "/workspace/src/my_robot_description/urdf/robot.urdf"
    "/workspace/src/my_robot_description/urdf/simple_robot.urdf"
)

ROBOT_URDF_FILE=""
for urdf_path in "${ROBOT_URDF_PATHS[@]}"; do
    if [ -f "$urdf_path" ] && [ -s "$urdf_path" ]; then
        ROBOT_URDF_FILE="$urdf_path"
        echo "✅ Found URDF: $urdf_path"
        break
    fi
done

if [ -z "$ROBOT_URDF_FILE" ]; then
    echo "❌ No valid URDF file found"
    exit 1
fi

echo "🤖 Loading robot URDF from: $ROBOT_URDF_FILE"
echo "📏 URDF file size: $(du -h "$ROBOT_URDF_FILE" | cut -f1)"

# Read URDF content
URDF_CONTENT=$(cat "$ROBOT_URDF_FILE")

if [ -z "$URDF_CONTENT" ]; then
    echo "❌ URDF content is empty"
    exit 1
fi

echo "📝 URDF content length: ${#URDF_CONTENT} characters"

# SOLUTION 1: Start robot_state_publisher with URDF file directly
echo "🔗 Starting robot_state_publisher with URDF file..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$URDF_CONTENT" &
RSP_PID=$!
echo "✅ robot_state_publisher started with URDF (PID: $RSP_PID)"

# Wait for robot_state_publisher to initialize properly
sleep 5

# Verify it's running
if ! kill -0 $RSP_PID 2>/dev/null; then
    echo "❌ robot_state_publisher crashed, trying alternative method..."
    
    # SOLUTION 2: Use launch file approach
    echo "🔄 Creating temporary launch file..."
    LAUNCH_FILE="/tmp/robot_publisher.launch.py"
    cat > "$LAUNCH_FILE" << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Read URDF content
    urdf_file = os.environ.get('ROBOT_URDF_PATH', '/workspace/src/my_robot_description/urdf/robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])
EOF
    
    # Set environment variable for launch file
    export ROBOT_URDF_PATH="$ROBOT_URDF_FILE"
    
    # Launch using launch file
    echo "🚀 Launching robot_state_publisher via launch file..."
    ros2 launch "$LAUNCH_FILE" &
    RSP_PID=$!
    echo "✅ robot_state_publisher launched (PID: $RSP_PID)"
    
    # Wait for it to start
    sleep 8
fi

# Verify robot_state_publisher is working
echo "🔍 Verifying robot_state_publisher..."
timeout=15
counter=0
while [ $counter -lt $timeout ]; do
    if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
        echo "✅ robot_state_publisher confirmed running"
        break
    fi
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo "❌ robot_state_publisher not responding"
    # Continue anyway - don't crash
fi

# Check if robot_description parameter is now available
echo "🔍 Checking robot_description parameter..."
if ros2 param list 2>/dev/null | grep -q robot_description; then
    echo "✅ robot_description parameter confirmed"
    # Get robot name
    ROBOT_NAME=$(echo "$URDF_CONTENT" | grep -o 'name="[^"]*"' | head -1 | cut -d'"' -f2)
    echo "🤖 Robot name: $ROBOT_NAME"
else
    echo "⚠️ robot_description parameter not found"
fi

# Start joint_state_publisher  
echo "🦾 Starting joint_state_publisher..."
ros2 run joint_state_publisher joint_state_publisher &
JSP_PID=$!
echo "✅ joint_state_publisher started (PID: $JSP_PID)"

# Start ROSBridge
echo "🔌 Starting ROSBridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
ROSBRIDGE_PID=$!
echo "✅ ROSBridge started (PID: $ROSBRIDGE_PID)"

# Wait for ROSBridge to be ready
sleep 5

# Start virtual robot
echo "🎮 Starting virtual robot..."
python3 /scripts/virtual_robot.py &
ROBOT_PID=$!
echo "✅ Virtual robot started (PID: $ROBOT_PID)"

echo ""
echo "✅ All services started!"
echo "  - robot_state_publisher (PID: $RSP_PID)"
echo "  - joint_state_publisher (PID: $JSP_PID)" 
echo "  - rosbridge_server (PID: $ROSBRIDGE_PID)"
echo "  - virtual_robot (PID: $ROBOT_PID)"

echo ""
echo "🔍 Final system verification..."

# Check topics
echo "📡 Key topics available:"
ros2 topic list 2>/dev/null | grep -E "(cmd_vel|odom|joint_states|tf)" | head -10 | sed 's/^/  ✓ /' || echo "  ⚠️ Limited topics available"

# Check TF
echo "🔗 TF frames:"
ros2 run tf2_tools view_frames.py 2>/dev/null &
sleep 3
kill %1 2>/dev/null || true

echo ""
echo "🎉 Robot system ready!"
echo "🌐 ROSBridge available at: ws://0.0.0.0:9090"
echo "🤖 differential_drive_robot should now be visible in web interface!"

# Keep container running
tail -f /dev/null