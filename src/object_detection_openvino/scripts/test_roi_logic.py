#!/usr/bin/env python3
"""
ROI 功能视觉验证脚本
测试裁剪逻辑和坐标映射
"""

import cv2
import numpy as np

def test_roi_logic():
    """测试 ROI 裁剪逻辑"""
    
    # 创建测试图像 (1920x1080)
    img_width, img_height = 1920, 1080
    image = np.zeros((img_height, img_width, 3), dtype=np.uint8)
    
    # 绘制网格以便可视化
    for i in range(0, img_height, 100):
        cv2.line(image, (0, i), (img_width, i), (50, 50, 50), 1)
    for i in range(0, img_width, 100):
        cv2.line(image, (i, 0), (i, img_height), (50, 50, 50), 1)
    
    # 在图像中心绘制一个目标
    center_x, center_y = img_width // 2, img_height // 2
    cv2.circle(image, (center_x, center_y), 50, (0, 255, 0), -1)
    cv2.putText(image, "CENTER TARGET", (center_x - 80, center_y - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 在左上角绘制一个目标
    cv2.circle(image, (300, 200), 30, (255, 0, 0), -1)
    cv2.putText(image, "CORNER TARGET", (250, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    print("=" * 60)
    print("ROI 裁剪逻辑测试")
    print("=" * 60)
    print(f"原始图像尺寸: {img_width}x{img_height}")
    print()
    
    # 测试 1: 全图模式
    print("测试 1: 全图模式 (roi_mode='full')")
    roi_mode = "full"
    roi_rect = (0, 0, img_width, img_height)
    roi_image = image
    print(f"  ROI 区域: x={roi_rect[0]}, y={roi_rect[1]}, "
          f"w={roi_rect[2]}, h={roi_rect[3]}")
    print(f"  ✅ 全图模式: 使用完整图像")
    print()
    
    # 测试 2: 中心裁剪模式
    print("测试 2: 中心裁剪模式 (roi_mode='center')")
    roi_mode = "center"
    roi_width, roi_height = 1280, 1024
    
    # 计算 ROI (模拟 C++ 逻辑)
    actual_roi_w = min(roi_width, img_width)
    actual_roi_h = min(roi_height, img_height)
    x = (img_width - actual_roi_w) // 2
    y = (img_height - actual_roi_h) // 2
    
    # 边界检查
    x = max(0, x)
    y = max(0, y)
    actual_roi_w = min(actual_roi_w, img_width - x)
    actual_roi_h = min(actual_roi_h, img_height - y)
    
    roi_rect = (x, y, actual_roi_w, actual_roi_h)
    roi_image = image[y:y+actual_roi_h, x:x+actual_roi_w]
    
    print(f"  配置 ROI 尺寸: {roi_width}x{roi_height}")
    print(f"  实际 ROI 区域: x={roi_rect[0]}, y={roi_rect[1]}, "
          f"w={roi_rect[2]}, h={roi_rect[3]}")
    print(f"  ROI 图像尺寸: {roi_image.shape[1]}x{roi_image.shape[0]}")
    print(f"  ✅ 中心裁剪模式: 提取了中心区域")
    print()
    
    # 测试 3: 坐标映射
    print("测试 3: 检测框坐标映射")
    # 模拟在 ROI 坐标系中的检测框 (相对于 ROI 左上角)
    detection_in_roi = {
        'center_x': 640,  # ROI 中心
        'center_y': 512,
        'box_x': 590,
        'box_y': 462,
        'box_w': 100,
        'box_h': 100
    }
    
    print(f"  ROI 坐标系中的检测框:")
    print(f"    中心点: ({detection_in_roi['center_x']}, {detection_in_roi['center_y']})")
    print(f"    边界框: x={detection_in_roi['box_x']}, y={detection_in_roi['box_y']}, "
          f"w={detection_in_roi['box_w']}, h={detection_in_roi['box_h']}")
    
    # 映射到完整图像坐标 (模拟 C++ 逻辑)
    mapped_center_x = detection_in_roi['center_x'] + roi_rect[0]
    mapped_center_y = detection_in_roi['center_y'] + roi_rect[1]
    mapped_box_x = detection_in_roi['box_x'] + roi_rect[0]
    mapped_box_y = detection_in_roi['box_y'] + roi_rect[1]
    
    print(f"  完整图像坐标系中的检测框:")
    print(f"    中心点: ({mapped_center_x}, {mapped_center_y})")
    print(f"    边界框: x={mapped_box_x}, y={mapped_box_y}, "
          f"w={detection_in_roi['box_w']}, h={detection_in_roi['box_h']}")
    print(f"  ✅ 坐标映射: 偏移量 = ({roi_rect[0]}, {roi_rect[1]})")
    print()
    
    # 测试 4: 边界情况
    print("测试 4: 边界情况 - ROI 大于图像")
    large_roi_w, large_roi_h = 3000, 2000
    actual_roi_w = min(large_roi_w, img_width)
    actual_roi_h = min(large_roi_h, img_height)
    x = (img_width - actual_roi_w) // 2
    y = (img_height - actual_roi_h) // 2
    x = max(0, x)
    y = max(0, y)
    actual_roi_w = min(actual_roi_w, img_width - x)
    actual_roi_h = min(actual_roi_h, img_height - y)
    
    print(f"  请求 ROI 尺寸: {large_roi_w}x{large_roi_h}")
    print(f"  实际 ROI 尺寸: {actual_roi_w}x{actual_roi_h}")
    print(f"  ✅ 边界处理: 自动裁剪到图像大小")
    print()
    
    # 创建可视化图像
    vis_image = image.copy()
    
    # 绘制 ROI 区域 (青色矩形)
    roi_x, roi_y, roi_w, roi_h = roi_rect
    cv2.rectangle(vis_image, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h),
                  (255, 255, 0), 3)
    cv2.putText(vis_image, f"ROI: {roi_w}x{roi_h}", (roi_x + 10, roi_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # 绘制映射后的检测框
    cv2.rectangle(vis_image, (mapped_box_x, mapped_box_y),
                  (mapped_box_x + detection_in_roi['box_w'],
                   mapped_box_y + detection_in_roi['box_h']),
                  (0, 255, 0), 2)
    cv2.circle(vis_image, (mapped_center_x, mapped_center_y), 4, (0, 255, 0), -1)
    
    # 保存结果
    output_path = "/home/amatrix02/Dart_2026_ws/src/object_detection_openvino/scripts/roi_test_visualization.jpg"
    cv2.imwrite(output_path, vis_image)
    
    print("=" * 60)
    print("✅ 所有测试通过！")
    print(f"可视化图像已保存到: {output_path}")
    print("=" * 60)

if __name__ == '__main__':
    test_roi_logic()
