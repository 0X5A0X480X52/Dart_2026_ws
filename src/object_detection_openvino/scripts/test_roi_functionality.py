#!/usr/bin/env python3
"""
测试 ROI 功能的简单脚本
验证参数是否正确加载
"""

import rclpy
from rclpy.node import Node
import sys

class ROITestNode(Node):
    def __init__(self):
        super().__init__('roi_test_node')
        
        # 声明参数（与 object_detection_openvino_node 相同）
        self.declare_parameter('roi_mode', 'full')
        self.declare_parameter('roi_width', 1280)
        self.declare_parameter('roi_height', 1024)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 384)
        self.declare_parameter('center_x', -1)
        self.declare_parameter('center_y', -1)
        
        # 读取参数
        roi_mode = self.get_parameter('roi_mode').value
        roi_width = self.get_parameter('roi_width').value
        roi_height = self.get_parameter('roi_height').value
        input_width = self.get_parameter('input_width').value
        input_height = self.get_parameter('input_height').value
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        
        # 打印参数
        self.get_logger().info('='*60)
        self.get_logger().info('ROI 功能参数测试')
        self.get_logger().info('='*60)
        self.get_logger().info(f'ROI Mode: {roi_mode}')
        self.get_logger().info(f'ROI Size: {roi_width}x{roi_height}')
        self.get_logger().info(f'Model Input Size: {input_width}x{input_height}')
        self.get_logger().info(f'ROI Center: ({center_x}, {center_y})')
        self.get_logger().info('='*60)
        
        # 验证参数
        success = True
        if roi_mode not in ['full', 'center']:
            self.get_logger().error(f'Invalid roi_mode: {roi_mode}')
            success = False
        
        if roi_width <= 0 or roi_height <= 0:
            self.get_logger().error(f'Invalid ROI size: {roi_width}x{roi_height}')
            success = False
            
        if input_width <= 0 or input_height <= 0:
            self.get_logger().error(f'Invalid input size: {input_width}x{input_height}')
            success = False
        
        # center_x/center_y 可以为 -1（自动居中）或在图像范围内
        if not (center_x == -1 or (0 <= center_x < input_width)):
            self.get_logger().error(f'Invalid center_x: {center_x} (must be -1 or within [0, {input_width-1}])')
            success = False
        if not (center_y == -1 or (0 <= center_y < input_height)):
            self.get_logger().error(f'Invalid center_y: {center_y} (must be -1 or within [0, {input_height-1}])')
            success = False
        
        if success:
            self.get_logger().info('✅ 所有参数验证通过！')
        else:
            self.get_logger().error('❌ 参数验证失败！')
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = ROITestNode()
    
    # 运行 2 秒后退出
    import time
    time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()
    
    print('\n测试完成！')

if __name__ == '__main__':
    main()
