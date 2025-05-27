#!/bin/bash
# scripts/start_tf2_web.sh - Fixed version

set -e
source /opt/ros/humble/setup.bash

echo "🔄 Starting TF2 Web Republisher..."

# Wait for main system
echo "⏳ Waiting for robot system to be ready..."
sleep 15

# Check if workspace exists and is built
if [ -d /workspace/install ]; then
    echo "✅ Found built workspace, sourcing it"
    cd /workspace
    source install/setup.bash
else
    echo "⚠️ No built workspace found, using base ROS environment"
fi

# Wait for TF topics to be available
echo "⏳ Waiting for TF topics..."
timeout=30
counter=0

while [ $counter -lt $timeout ]; do
    if ros2 topic list 2>/dev/null | grep -q "/tf"; then
        echo "✅ TF topics available"
        break
    fi
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo "⚠️ TF topics not available after ${timeout}s, starting anyway"
fi

echo "✅ Starting simple TF2 web republisher"

# Start TF2 republisher
python3 /scripts/tf2_web_republisher.py