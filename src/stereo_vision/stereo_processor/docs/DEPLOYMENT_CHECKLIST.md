# Stereo Processor 部署检查清单

## 📋 部署前检查

### 1. 编译检查
- [ ] 代码编译成功，无错误
- [ ] 所有依赖项已安装
- [ ] install 目录包含所有必要文件

```bash
colcon build --packages-select stereo_processor
# 检查输出: "Finished <<< stereo_processor"
```

### 2. 文件完整性检查
- [ ] 可执行文件存在
  ```bash
  ls -l install/stereo_processor/lib/stereo_processor/stereo_processor_node
  ```
- [ ] 配置文件存在
  ```bash
  ls -l install/stereo_processor/share/stereo_processor/config/stereo_processor.yaml
  ```
- [ ] Launch 文件存在
  ```bash
  ls -l install/stereo_processor/share/stereo_processor/launch/*.py
  ```

### 3. 环境检查
- [ ] ROS 2 已正确安装
  ```bash
  ros2 --version
  ```
- [ ] 工作空间已 source
  ```bash
  echo $ROS_DISTRO  # 应显示 "humble"
  ```
- [ ] OpenCV 已安装
  ```bash
  pkg-config --modversion opencv4
  ```

## 🔧 运行前检查

### 4. 相机驱动检查
- [ ] 相机驱动节点正在运行
  ```bash
  ros2 node list | grep camera
  ```
- [ ] 左相机图像发布正常
  ```bash
  ros2 topic hz /camera/left/image_raw
  # 应显示稳定的帧率
  ```
- [ ] 右相机图像发布正常
  ```bash
  ros2 topic hz /camera/right/image_raw
  ```
- [ ] 相机标定信息发布正常
  ```bash
  ros2 topic echo /camera/left/camera_info --once
  ros2 topic echo /camera/right/camera_info --once
  ```

### 5. 参数配置检查
- [ ] 配置文件已根据实际情况调整
- [ ] 视差参数适合场景距离
  - 近距离 (< 2m): `num_disparities: 256`
  - 中距离 (2-5m): `num_disparities: 128`
  - 远距离 (> 5m): `num_disparities: 64`
- [ ] Topic 名称与相机驱动匹配

## ✅ 启动检查

### 6. 节点启动检查
- [ ] 节点成功启动，无错误信息
  ```bash
  ros2 launch stereo_processor stereo_processor.launch.py
  ```
- [ ] 日志显示 "Stereo Processor Node initialized successfully"
- [ ] 日志显示 "Stereo camera model initialized"
- [ ] 日志显示 "Rectification maps initialized"

### 7. Topic 发布检查
- [ ] 所有输出 topics 都在发布
  ```bash
  ros2 topic list | grep -E "(rect|disparity|points2)"
  ```
  应看到:
  - `/camera/left/image_rect`
  - `/camera/right/image_rect`
  - `/stereo/disparity`
  - `/stereo/points2`

- [ ] 输出频率正常
  ```bash
  ros2 topic hz /camera/left/image_rect
  ros2 topic hz /stereo/disparity
  ros2 topic hz /stereo/points2
  ```

### 8. 数据质量检查
- [ ] 校正图像无明显畸变
  ```bash
  ros2 run rqt_image_view rqt_image_view /camera/left/image_rect
  ```
- [ ] 视差图有明显的深度信息（非全黑/全白）
  ```bash
  ros2 run rqt_image_view rqt_image_view /stereo/disparity/image
  ```
- [ ] 点云包含有效数据
  ```bash
  ros2 topic echo /stereo/points2 --once
  # 检查是否有非 NaN 的点
  ```

## 🔍 性能检查

### 9. 系统资源检查
- [ ] CPU 使用率在可接受范围内 (< 80%)
  ```bash
  top -p $(pgrep -f stereo_processor_node)
  ```
- [ ] 内存使用正常
- [ ] 没有频繁的内存分配/释放

### 10. 延迟检查
- [ ] 端到端延迟 < 100ms（对于实时应用）
  ```bash
  ros2 topic delay /stereo/points2
  ```
- [ ] 处理帧率满足需求
  ```bash
  ros2 topic hz /stereo/points2
  ```

## 🔗 系统集成检查

### 11. 下游节点集成
- [ ] object_detection_openvino 正确订阅 `/camera/left/image_rect`
- [ ] stereo_distance_estimator 正确订阅 `/stereo/disparity` 和 `/stereo/points2`
- [ ] 整个数据流正常
  ```bash
  ros2 topic list
  # 检查所有相关 topics 都在发布
  ```

### 12. 可视化检查
- [ ] RViz2 可以正确显示点云
- [ ] Foxglove Studio 可以正确显示所有数据流
- [ ] 图像和点云时间戳同步

## 🐛 问题排查

如果任何检查项失败，参考以下文档：
- 基本问题: `QUICKSTART.md`
- 详细配置: `MIGRATION_GUIDE.md`
- 实现细节: `IMPLEMENTATION_SUMMARY.md`

## 📝 部署记录

部署日期: _______________
部署人员: _______________
环境信息:
- ROS 2 版本: _______________
- OpenCV 版本: _______________
- 相机型号: _______________

检查结果:
- [ ] 所有检查项通过
- [ ] 部分检查项失败（记录详情）: _______________
- [ ] 需要进一步调优

备注:
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________

## ✨ 优化建议

根据检查结果，考虑以下优化：

### 如果处理速度慢:
- [ ] 降低输入图像分辨率
- [ ] 减小 `num_disparities`
- [ ] 减小 `block_size`
- [ ] 设置 `use_color: false`

### 如果视差质量差:
- [ ] 重新标定相机
- [ ] 调整 SGBM 参数
- [ ] 改善光照条件
- [ ] 增加场景纹理

### 如果点云噪点多:
- [ ] 增大 `uniqueness_ratio`
- [ ] 调整 `speckle_window_size` 和 `speckle_range`
- [ ] 添加点云后处理滤波器

---

**检查完成后，请保存此清单以备将来参考！**
