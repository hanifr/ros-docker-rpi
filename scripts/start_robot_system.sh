#!/bin/bash
# scripts/start_robot_system.sh - Fixed stable version

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

# Read URDF content and validate
URDF_CONTENT=$(cat "$ROBOT_URDF_FILE")

if [ -z "$URDF_CONTENT" ]; then
    echo "❌ URDF content is empty"
    exit 1
fi

echo "📝 URDF content length: ${#URDF_CONTENT} characters"

# Validate URDF has basic robot structure
if ! echo "$URDF_CONTENT" | grep -q "<robot"; then
    echo "❌ URDF file does not contain valid robot definition"
    exit 1
fi

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
    
    try:
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
    except Exception as e:
        print(f"Error reading URDF file {urdf_file}: {e}")
        robot_description = ""
    
    if not robot_description.strip():
        print("Error: Empty or invalid URDF content")
        return LaunchDescription([])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            arguments=['--ros-args', '--log-level', 'info']
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
    echo "⚠️ robot_state_publisher not responding after ${timeout}s"
    echo "🔄 Continuing with other services..."
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

# Verify ROSBridge is working
echo "🔍 Verifying ROSBridge..."
if netstat -tuln 2>/dev/null | grep -q ":9090"; then
    echo "✅ ROSBridge confirmed listening on port 9090"
else
    echo "⚠️ ROSBridge may not be listening on port 9090"
fi

# Start virtual robot
echo "🎮 Starting virtual robot..."
if [ -f "/scripts/virtual_robot.py" ]; then
    python3 /scripts/virtual_robot.py &
    ROBOT_PID=$!
    echo "✅ Virtual robot started (PID: $ROBOT_PID)"
else
    echo "⚠️ Virtual robot script not found at /scripts/virtual_robot.py"
    ROBOT_PID=""
fi

echo ""
echo "✅ All services started!"
echo "  - robot_state_publisher (PID: $RSP_PID)"
echo "  - joint_state_publisher (PID: $JSP_PID)" 
echo "  - rosbridge_server (PID: $ROSBRIDGE_PID)"
if [ -n "$ROBOT_PID" ]; then
    echo "  - virtual_robot (PID: $ROBOT_PID)"
fi

echo ""
echo "🔍 Final system verification..."

# Check topics
echo "📡 Available topics:"
timeout 5s ros2 topic list 2>/dev/null | head -10 | sed 's/^/  ✓ /' || echo "  ⚠️ Could not list topics"

# Check nodes
echo "🤖 Running nodes:"
timeout 5s ros2 node list 2>/dev/null | sed 's/^/  ✓ /' || echo "  ⚠️ Could not list nodes"

# Test parameter server
echo "📋 Setting up default physics parameters..."
ros2 param set /robot max_linear_velocity 0.5 2>/dev/null || echo "  ⚠️ Could not set default parameters"
ros2 param set /robot max_angular_velocity 1.0 2>/dev/null || true
ros2 param set /robot mass 15.0 2>/dev/null || true

echo ""
echo "🎉 Robot system ready!"
echo "🌐 ROSBridge available at: ws://0.0.0.0:9090"
echo "🤖 differential_drive_robot should now be visible in web interface!"
echo ""
echo "📋 Available interfaces:"
echo "  - Physics Parameter Tuner with ROS Parameter Server"
echo "  - Real-time robot control and monitoring"
echo "  - TF and odometry data streams"

# Function to handle graceful shutdown
cleanup() {
    echo ""
    echo "🛑 Shutting down robot system..."
    kill $RSP_PID 2>/dev/null || true
    kill $JSP_PID 2>/dev/null || true
    kill $ROSBRIDGE_PID 2>/dev/null || true
    if [ -n "$ROBOT_PID" ]; then
        kill $ROBOT_PID 2>/dev/null || true
    fi
    echo "✅ Cleanup complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Keep container running
echo "📡 System running - press Ctrl+C to stop"
tail -f /dev/null