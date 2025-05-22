#!/bin/bash
echo "Starting headless Gazebo simulation on Pi..."

# Check Pi temperature
TEMP=$(vcgencmd measure_temp | cut -d= -f2)
echo "Pi Temperature: $TEMP"

# Start containers
docker-compose -f docker-compose.pi.yml up -d

echo "Simulation started!"
echo "Access web interface at: http://$(hostname -I | awk '{print $1}'):3000"
echo "ROS Bridge at: ws://$(hostname -I | awk '{print $1}'):9090"