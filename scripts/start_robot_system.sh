#!/bin/bash
# scripts/start_robot_system.sh - Working version with correct paths

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

# Try both URDF files (since you have robot.urdf and simple_robot.urdf)
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
    echo "📁 Checked paths:"
    for path in "${ROBOT_URDF_PATHS[@]}"; do
        echo "  - $path: $([ -f "$path" ] && echo "exists" || echo "missing")"
    done
    exit 1
fi

echo "🤖 Loading robot URDF from: $ROBOT_URDF_FILE"
echo "📏 URDF file size: $(du -h "$ROBOT_URDF_FILE" | cut -f1)"

# Read URDF content
URDF_CONTENT=$(cat "$ROBOT_URDF_FILE")

if [ -n "$URDF_CONTENT" ]; then
    echo "📝 URDF content length: ${#URDF_CONTENT} characters"
    
    # Set robot_description parameter
    echo "🔧 Setting robot_description parameter..."
    ros2 param set robot_description "$URDF_CONTENT"
    echo "✅ Robot description loaded successfully"
else
    echo "❌ URDF content is empty"
    exit 1
fi

# Wait for parameter to be available
sleep 2

# Start robot_state_publisher
echo "🔗 Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher &
RSP_PID=$!
echo "✅ robot_state_publisher started (PID: $RSP_PID)"

# Wait for robot_state_publisher to initialize
sleep 3

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
echo "✅ All services started successfully!"
echo "  - robot_state_publisher (PID: $RSP_PID)"
echo "  - joint_state_publisher (PID: $JSP_PID)" 
echo "  - rosbridge_server (PID: $ROSBRIDGE_PID)"
echo "  - virtual_robot (PID: $ROBOT_PID)"

echo ""
echo "🔍 System verification..."

# Verify robot_description parameter
sleep 2
if ros2 param list 2>/dev/null | grep -q robot_description; then
    echo "✅ robot_description parameter confirmed"
    
    # Get robot name from URDF
    ROBOT_NAME=$(echo "$URDF_CONTENT" | grep -o 'name="[^"]*"' | head -1 | cut -d'"' -f2)
    echo "🤖 Robot name: $ROBOT_NAME"
else
    echo "⚠️ robot_description parameter not found"
fi

# Check topics
echo "📡 Key topics available:"
ros2 topic list 2>/dev/null | grep -E "(cmd_vel|odom|joint_states|tf)" | head -10 | sed 's/^/  ✓ /'

echo ""
echo "🎉 Robot system ready!"
echo "🌐 ROSBridge available at: ws://0.0.0.0:9090"
echo "🕸️ Web interface should now load the robot model!"

# Keep container running
tail -f /dev/null