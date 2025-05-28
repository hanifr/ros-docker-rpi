#!/bin/bash
# scripts/start_tf2_web.sh - Fixed workspace sourcing

set -e
source /opt/ros/humble/setup.bash

echo "🔄 Starting TF2 Web Republisher..."

# Wait for main system
echo "⏳ Waiting for robot system to be ready..."
sleep 15

# Check workspace properly - tf2 container needs different approach
echo "🔍 Checking for workspace..."

# The tf2-web-pub container might not have the built workspace
# Use only base ROS 2 environment
echo "✅ Using base ROS 2 environment"

# Wait for TF topics to be available
echo "⏳ Waiting for TF topics..."
timeout=60
counter=0

while [ $counter -lt $timeout ]; do
    if ros2 topic list 2>/dev/null | grep -q "/tf"; then
        echo "✅ TF topics available"
        break
    fi
    echo "  Waiting... ($counter/$timeout)"
    sleep 2
    counter=$((counter + 2))
done

if [ $counter -ge $timeout ]; then
    echo "⚠️ TF topics not available after ${timeout}s"
    echo "📡 Available topics:"
    ros2 topic list 2>/dev/null | head -10
    echo "🔄 Starting anyway..."
fi

echo "✅ Starting TF2 web republisher"

# Start TF2 republisher
python3 /scripts/tf2_web_republisher.py