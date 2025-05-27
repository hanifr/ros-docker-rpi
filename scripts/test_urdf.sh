#!/bin/bash
# scripts/test_urdf.sh - Quick URDF test

ROBOT_URDF_FILE="/workspace/src/my_robot_description/urdf/robot.urdf"

echo "🔍 Quick URDF Test"
echo "=================="

if [ -f "$ROBOT_URDF_FILE" ]; then
    echo "✅ URDF file found"
    echo "📏 Size: $(du -h $ROBOT_URDF_FILE | cut -f1)"
    
    # Test reading content
    CONTENT=$(cat "$ROBOT_URDF_FILE")
    if [ -n "$CONTENT" ]; then
        echo "✅ URDF content readable"
        echo "📝 Content length: ${#CONTENT} characters"
        
        # Test parameter setting
        echo "🔧 Testing parameter setting..."
        if ros2 param set test_robot_description "$CONTENT" 2>/dev/null; then
            echo "✅ Parameter setting works"
            ros2 param delete test_robot_description 2>/dev/null
        else
            echo "❌ Parameter setting failed"
        fi
    else
        echo "❌ URDF content is empty"
    fi
else
    echo "❌ URDF file not found"
fi