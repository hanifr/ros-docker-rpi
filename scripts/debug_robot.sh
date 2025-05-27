#!/bin/bash
# scripts/debug_robot.sh - Debug robot loading

echo "🔍 Robot Loading Debug Script"
echo "============================"

source /opt/ros/humble/setup.bash

# Check URDF file
ROBOT_URDF_FILE="/workspace/src/my_robot_description/urdf/robot.urdf"

echo "1. Checking URDF file..."
if [ -f "$ROBOT_URDF_FILE" ]; then
    echo "   ✅ File exists: $ROBOT_URDF_FILE"
    echo "   📏 Size: $(du -h $ROBOT_URDF_FILE | cut -f1)"
    echo "   📝 Lines: $(wc -l < $ROBOT_URDF_FILE)"
    
    # Show first few lines
    echo "   📄 First 5 lines:"
    head -5 "$ROBOT_URDF_FILE" | sed 's/^/      /'
    
    # Check for robot name
    if grep -q "differential_drive_robot" "$ROBOT_URDF_FILE"; then
        echo "   ✅ Contains 'differential_drive_robot'"
    else
        echo "   ⚠️ Does not contain 'differential_drive_robot'"
        echo "   🔍 Robot name found:"
        grep -o 'name="[^"]*"' "$ROBOT_URDF_FILE" | head -3 | sed 's/^/      /'
    fi
else
    echo "   ❌ File not found: $ROBOT_URDF_FILE"
    echo "   📁 Searching for URDF files:"
    find /workspace -name "*.urdf" -type f | sed 's/^/      /'
fi

echo ""
echo "2. Checking ROS parameters..."
if ros2 param list 2>/dev/null | grep -q robot_description; then
    echo "   ✅ robot_description parameter exists"
    
    # Get parameter size
    PARAM_SIZE=$(ros2 param get robot_description robot_description 2>/dev/null | wc -c)
    echo "   📏 Parameter size: $PARAM_SIZE characters"
    
    if [ "$PARAM_SIZE" -gt 100 ]; then
        echo "   ✅ Parameter has substantial content"
    else
        echo "   ⚠️ Parameter seems small or empty"
    fi
else
    echo "   ❌ robot_description parameter not found"
fi

echo ""
echo "3. Checking ROS nodes..."
ros2 node list 2>/dev/null | grep -E "(robot_state|joint_state)" | sed 's/^/   ✓ /' || echo "   ❌ No robot nodes found"

echo ""
echo "4. Checking ROS topics..."
ros2 topic list 2>/dev/null | grep -E "(tf|cmd_vel|odom|joint)" | sed 's/^/   ✓ /' || echo "   ❌ No robot topics found"

echo ""
echo "5. Workspace structure..."
echo "   📁 /workspace contents:"
ls -la /workspace/ | sed 's/^/      /'

if [ -d /workspace/src ]; then
    echo "   📁 /workspace/src contents:"
    ls -la /workspace/src/ | sed 's/^/      /'
    
    if [ -d /workspace/src/my_robot_description ]; then
        echo "   📁 my_robot_description contents:"
        ls -la /workspace/src/my_robot_description/ | sed 's/^/      /'
    fi
fi