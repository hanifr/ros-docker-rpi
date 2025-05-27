#!/bin/bash
# scripts/start_tf2_web.sh - TF2 web republisher startup

set -e
source /opt/ros/humble/setup.bash

echo "🔄 Starting TF2 Web Republisher..."

# Wait for main system
sleep 10

# Source workspace if available
if [ -d /workspace/install ]; then
    cd /workspace
    source install/setup.bash
fi

echo "✅ Starting simple TF2 web republisher"

# Start TF2 republisher
python3 /scripts/tf2_web_republisher.py