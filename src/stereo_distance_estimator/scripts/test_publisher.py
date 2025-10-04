#!/usr/bin/env python3
"""
Simple test script to publish dummy data to stereo_distance_estimator
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from rm_interfaces.msg import Target2D, Target2DArray
import numpy as np
from cv_bridge import CvBridge
import struct


class StereoDistanceEstimatorTest(Node):
    def __init__(self):
        super().__init__('stereo_distance_estimator_test')
        
        # Publishers
        self.target2d_pub = self.create_publisher(
            Target2DArray, '/filter/target2d_array', 10)
        self.disparity_pub = self.create_publisher(
            Image, '/stereo/disparity', 10)
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/stereo/points2', 10)
        
        # Timer to publish test data
        self.timer = self.create_timer(1.0, self.publish_test_data)
        self.bridge = CvBridge()
        
        self.get_logger().info('Stereo Distance Estimator Test Node started')
        
    def publish_test_data(self):
        """Publish dummy test data"""
        timestamp = self.get_clock().now().to_msg()
        
        # 1. Publish 2D targets
        target2d_array = Target2DArray()
        target2d_array.header.stamp = timestamp
        target2d_array.header.frame_id = 'camera_left'
        
        # Create a dummy target at center of image
        target = Target2D()
        target.header = target2d_array.header
        target.x = 320.0
        target.y = 240.0
        target.width = 50.0
        target.height = 80.0
        target.confidence = 0.95
        target.class_name = 'test_target'
        target.id = 1
        target.is_filtered = True
        
        target2d_array.targets.append(target)
        self.target2d_pub.publish(target2d_array)
        
        # 2. Publish dummy disparity image
        # Create a 640x480 disparity image with depth at 2 meters
        # Disparity = (fx * baseline) / depth
        # Assuming fx=600, baseline=0.12m, depth=2m -> disparity=36
        disparity_image = np.ones((480, 640), dtype=np.float32) * 36.0
        disparity_msg = self.bridge.cv2_to_imgmsg(disparity_image, encoding='32FC1')
        disparity_msg.header = target2d_array.header
        self.disparity_pub.publish(disparity_msg)
        
        # 3. Publish dummy point cloud
        pointcloud_msg = self.create_dummy_pointcloud(timestamp)
        self.pointcloud_pub.publish(pointcloud_msg)
        
        self.get_logger().info('Published test data')
        
    def create_dummy_pointcloud(self, timestamp):
        """Create a dummy organized point cloud (640x480)"""
        width = 640
        height = 480
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'camera_link'
        msg.height = height
        msg.width = width
        msg.is_dense = False
        msg.is_bigendian = False
        
        # Define point fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * width
        
        # Create point cloud data
        # For simplicity, create a plane at z=2.0m
        points = []
        for v in range(height):
            for u in range(width):
                # Simple pinhole projection (assuming fx=fy=600, cx=320, cy=240)
                z = 2.0  # meters
                x = (u - 320) * z / 600.0
                y = (v - 240) * z / 600.0
                
                # Pack as binary data
                points.append(struct.pack('fff', x, y, z))
        
        msg.data = b''.join(points)
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = StereoDistanceEstimatorTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
