#!/usr/bin/env python3
"""
Virtual differential drive robot simulation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time

class VirtualDifferentialRobot(Node):
    def __init__(self):
        super().__init__('virtual_robot')
        
        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot parameters (from your URDF)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.wheel_separation = 0.35  # From your URDF
        self.wheel_radius = 0.1       # From your URDF
        
        # Wheel positions
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        
        # Timer
        self.timer = self.create_timer(0.05, self.update_robot)  # 20Hz
        self.last_time = time.time()
        
        self.get_logger().info('Virtual differential drive robot started')
    
    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
    def update_robot(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Differential drive kinematics
        if abs(self.angular_vel) < 1e-6:
            self.x += self.linear_vel * math.cos(self.theta) * dt
            self.y += self.linear_vel * math.sin(self.theta) * dt
        else:
            radius = self.linear_vel / self.angular_vel
            dtheta = self.angular_vel * dt
            self.x += radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += radius * (math.cos(self.theta) - math.cos(self.theta + dtheta))
            self.theta += dtheta
        
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Update wheel positions
        left_wheel_vel = (self.linear_vel - self.angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (self.linear_vel + self.angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt
        
        # Publish everything
        self.publish_odometry()
        self.publish_joint_states(left_wheel_vel, right_wheel_vel)
        self.publish_tf()
    
    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom)
    
    def publish_joint_states(self, left_vel, right_vel):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [left_vel, right_vel]
        joint_state.effort = [0.0, 0.0]
        
        self.joint_pub.publish(joint_state)
    
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    robot = VirtualDifferentialRobot()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()