#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.bridge = CvBridge()
        self.get_logger().info('Test image publisher started')
        
    def timer_callback(self):
        # Create a test image (640x480, BGR)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw some colored rectangles
        cv2.rectangle(img, (100, 100), (200, 200), (255, 0, 0), -1)  # Blue
        cv2.rectangle(img, (300, 200), (400, 300), (0, 0, 255), -1)  # Red
        
        # Add text
        cv2.putText(img, 'Test Image', (250, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (255, 255, 255), 2)
        
        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
