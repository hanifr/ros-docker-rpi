#!/bin/bash
# scripts/start_gazebo_sim.sh - MINIMAL - Works without Gazebo packages
set -e

echo "🚀 Starting Minimal ROS 2 Simulation Environment..."

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
echo "✅ ROS 2 environment sourced"

cd /workspace

# Set basic environment variables
export ROS_DOMAIN_ID=0

# Create FastRTPS profile for Pi optimization
cat > fastrtps_profile.xml << 'EOXML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <sendBufferSize>1048576</sendBufferSize>
      <receiveBufferSize>1048576</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOXML

export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/fastrtps_profile.xml"

# Show available ROS 2 resources
echo "🔍 Available ROS 2 packages: $(ros2 pkg list | wc -l)"
echo "📋 Message types available:"
ros2 interface list | grep -E "(geometry_msgs|std_msgs|sensor_msgs|nav_msgs)" | head -5 || echo "Limited message types available"

# Start ROSBridge (using our flexible script)
echo "🔌 Starting ROSBridge WebSocket Server..."
/start_rosbridge.sh &
ROSBRIDGE_PID=$!
echo "✅ ROSBridge started (PID: $ROSBRIDGE_PID)"

# Wait for ROSBridge to start
sleep 5

# Create and start a virtual robot simulation
echo "🤖 Starting Virtual Robot Simulation..."

# Create virtual robot script
cat > virtual_robot.py << 'EOPY'
#!/usr/bin/env python3
"""
Virtual Robot - Responds to cmd_vel commands and publishes odometry
Works without Gazebo or any simulation packages
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import threading
import time
import math

class VirtualRobot(Node):
    def __init__(self):
        super().__init__('virtual_robot')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Robot state
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.linear_vel, self.angular_vel = 0.0, 0.0
        self.last_cmd_time = time.time()
        
        # Timers
        self.odom_timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        self.scan_timer = self.create_timer(0.2, self.publish_laser_scan)  # 5 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1 Hz
        
        self.get_logger().info('🤖 Virtual Robot started - listening on /cmd_vel')
        print('🤖 Virtual Robot started - responds to /cmd_vel commands')
    
    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        self.last_cmd_time = time.time()
        
        self.get_logger().info(f'📤 Received cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
        print(f'📤 Virtual robot moving: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def publish_odometry(self):
        # Simulate robot movement
        dt = 0.1
        
        # Decay velocity if no recent commands (simulate friction)
        if time.time() - self.last_cmd_time > 0.5:
            self.linear_vel *= 0.9
            self.angular_vel *= 0.9
        
        # Update position
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt
        
        # Keep angle in range [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        # Velocity
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom)
    
    def publish_laser_scan(self):
        # Create simple laser scan (360 degree scan)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180  # 1 degree increments
        scan.time_increment = 0.0
        scan.scan_time = 0.2
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Simple obstacle pattern (walls at 5m distance)
        ranges = []
        for i in range(360):
            angle = (i - 180) * math.pi / 180
            # Create some variation in the ranges
            base_range = 5.0 + math.sin(angle * 4) * 0.5
            ranges.append(base_range)
        
        scan.ranges = ranges
        scan.intensities = [100.0] * len(ranges)  # Constant intensity
        
        self.scan_pub.publish(scan)
    
    def publish_status(self):
        status_msg = String()
        status_msg.data = f'Virtual Robot: pos=({self.x:.2f},{self.y:.2f},{self.theta:.2f}), vel=({self.linear_vel:.2f},{self.angular_vel:.2f})'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    robot = VirtualRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info('🛑 Virtual Robot shutting down')
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOPY

# Start the virtual robot
python3 virtual_robot.py &
ROBOT_PID=$!
echo "✅ Virtual Robot started (PID: $ROBOT_PID)"

# Wait for robot to initialize
sleep 5

# Show diagnostic information
echo ""
echo "🔍 === SIMULATION STATUS ==="
echo "📋 Available ROS topics:"
ros2 topic list

echo ""
echo "🤖 Available ROS nodes:"
ros2 node list

echo ""
echo "🎯 Essential robot topics:"
ESSENTIAL_TOPICS=("/cmd_vel" "/odom" "/scan" "/robot_status")
for topic in "${ESSENTIAL_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo "   ✅ $topic available"
    else
        echo "   ⚠️  $topic not found"
    fi
done

# Test robot responsiveness
echo ""
echo "🧪 Testing robot control..."
echo "📤 Sending test command to virtual robot..."
timeout 3s ros2 topic pub --rate 2 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.2}, angular: {z: 0.1}}' >/dev/null 2>&1 &
TEST_PID=$!
sleep 2
kill $TEST_PID 2>/dev/null || true

echo "✅ Test command sent"

echo ""
echo "🎯 === MINIMAL SIMULATION READY ==="
echo "   🌉 ROSBridge: $(kill -0 $ROSBRIDGE_PID 2>/dev/null && echo "Running on ws://localhost:9090" || echo "Failed")"
echo "   🤖 Virtual Robot: $(kill -0 $ROBOT_PID 2>/dev/null && echo "Running and responding to /cmd_vel" || echo "Failed")"
echo "   📊 Topics: $(ros2 topic list | wc -l) total"
echo "   🔗 Nodes: $(ros2 node list | wc -l) active"

echo ""
echo "✅ === READY FOR WEB INTERFACE ==="
echo "   Your web interface should now be able to control the virtual robot!"
echo "   Test at: http://your-pi-ip:5000"
echo "   Robot will respond to movement commands and publish odometry."

# Health monitoring loop
echo ""
echo "💓 Starting health monitoring..."
while true; do
    sleep 30
    
    # Check ROSBridge
    if ! kill -0 $ROSBRIDGE_PID 2>/dev/null; then
        echo "🔄 Restarting ROSBridge..."
        /start_rosbridge.sh &
        ROSBRIDGE_PID=$!
    fi
    
    # Check Virtual Robot
    if ! kill -0 $ROBOT_PID 2>/dev/null; then
        echo "🔄 Restarting Virtual Robot..."
        python3 virtual_robot.py &
        ROBOT_PID=$!
    fi
    
    # Check topics
    TOPICS_COUNT=$(ros2 topic list 2>/dev/null | wc -l)
    CMD_VEL_OK=$(ros2 topic list | grep -q "/cmd_vel" && echo "OK" || echo "MISSING")
    
    echo "💓 Health: $(date '+%H:%M:%S') | ROSBridge: $(kill -0 $ROSBRIDGE_PID 2>/dev/null && echo "OK" || echo "FAILED") | Robot: $(kill -0 $ROBOT_PID 2>/dev/null && echo "OK" || echo "FAILED") | /cmd_vel: $CMD_VEL_OK | Topics: $TOPICS_COUNT"
done