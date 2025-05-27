#!/usr/bin/env python3
"""
Simple TF2 Web Republisher for ros3djs
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage

class SimpleTF2WebRepublisher(Node):
    def __init__(self):
        super().__init__('tf2_web_republisher')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for web-friendly TF
        self.web_tf_publisher = self.create_publisher(TFMessage, '/tf_web', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.publish_transforms)  # 10Hz
        
        # Robot frames from your URDF
        self.source_frames = ['base_link', 'left_wheel', 'right_wheel', 'caster_wheel']
        self.target_frame = 'odom'
        
        self.get_logger().info('TF2 Web Republisher started for differential_drive_robot')
        
    def publish_transforms(self):
        try:
            web_transforms = []
            
            for source_frame in self.source_frames:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame, 
                        source_frame, 
                        rclpy.time.Time()
                    )
                    web_transforms.append(transform)
                except Exception:
                    continue
            
            if web_transforms:
                tf_msg = TFMessage()
                tf_msg.transforms = web_transforms
                self.web_tf_publisher.publish(tf_msg)
                
        except Exception:
            pass

def main():
    rclpy.init()
    node = SimpleTF2WebRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()