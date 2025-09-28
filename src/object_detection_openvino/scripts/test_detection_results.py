#!/usr/bin/env python3

"""
简单的测试脚本，用于验证目标检测节点的输出
订阅 /detector/target2d_array 话题并打印检测结果
"""

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import Target2DArray


class DetectionResultViewer(Node):
    def __init__(self):
        super().__init__('detection_result_viewer')
        
        # 订阅检测结果话题
        self.subscription = self.create_subscription(
            Target2DArray,
            '/detector/target2d_array',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Detection Result Viewer started')
        self.get_logger().info('Waiting for detection results on /detector/target2d_array...')
    
    def detection_callback(self, msg):
        """处理检测结果回调"""
        if len(msg.targets) > 0:
            self.get_logger().info(f'Received {len(msg.targets)} detections:')
            
            for i, target in enumerate(msg.targets):
                self.get_logger().info(
                    f'  Target {i+1}: '
                    f'class="{target.class_name}", '
                    f'confidence={target.confidence:.3f}, '
                    f'center=({target.x:.1f}, {target.y:.1f}), '
                    f'size=({target.width:.1f}x{target.height:.1f}), '
                    f'id={target.id}'
                )
        else:
            self.get_logger().info('No detections in current frame')


def main(args=None):
    rclpy.init(args=args)
    
    viewer = DetectionResultViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()