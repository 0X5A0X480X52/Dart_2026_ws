# 坐标缩放配置快速指南

## 🚨 重要提示

**`stereo_distance_estimator` 的 `image_scale` 参数必须与 `stereo_image_proc_wrapper` 的 `image_scale` 保持一致！**

## 为什么需要此参数？

```
目标检测 (原始图像 1280x720)     →  2D坐标 (640, 360)
                                          ↓
                                    需要缩放！
                                          ↓
点云/视差 (缩小后 640x360)       →  3D坐标 (320, 180)
```

## 配置步骤

### 1. 检查立体视觉配置

```bash
cat src/stereo_vision/stereo_image_proc_wrapper/config/stereo_params.yaml | grep image_scale
```

输出示例：
```yaml
image_scale: 0.5
```

### 2. 设置距离估计器相同的值

编辑 `src/stereo_distance_estimator/config/stereo_distance_estimator.yaml`：

```yaml
stereo_distance_estimator:
  ros__parameters:
    # ... 其他参数 ...
    
    # ⚠️ 必须与 stereo_image_proc_wrapper 保持一致
    image_scale: 0.5  # 如果立体视觉是 0.5，这里也是 0.5
```

### 3. 编译并运行

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_distance_estimator
source install/setup.bash
ros2 launch stereo_distance_estimator stereo_distance_estimator_config.launch.py
```

## 常见配置场景

| 立体视觉设置 | 距离估计设置 | 结果 |
|------------|------------|------|
| `image_scale: 0.5` | `image_scale: 0.5` | ✅ 正确 |
| `image_scale: 0.5` | `image_scale: 1.0` | ❌ 坐标越界 |
| `image_scale: 1.0` | `image_scale: 0.5` | ❌ 错误的 3D 点 |
| `image_scale: 1.0` | `image_scale: 1.0` | ✅ 正确 |

## 验证配置是否正确

### 运行时日志

✅ **正确的日志**：
```
[stereo_distance_estimator]: Published: 3 2D targets -> 3 3D targets
[stereo_distance_estimator]: Target at orig(640, 360) scaled(320, 180) has valid 3D point
```

❌ **错误的日志**：
```
[stereo_distance_estimator]: Pixel coordinates (640, 360) out of cloud bounds (640 x 360)
[stereo_distance_estimator]: Published: 3 2D targets -> 0 3D targets
```

### 检查输出

```bash
# 查看是否有 3D 目标输出
ros2 topic echo /stereo/target3d_array_raw

# 如果没有输出或全是空数组，说明配置可能不匹配
```

## 性能建议

| image_scale | 性能 | 精度 | 推荐场景 |
|------------|------|------|----------|
| 0.25 | 🚀🚀🚀 最快 | ⭐⭐ 低 | 实时性要求极高 |
| 0.5 | 🚀🚀 快 | ⭐⭐⭐ 中 | **推荐：平衡性能和精度** |
| 0.75 | 🚀 较慢 | ⭐⭐⭐⭐ 高 | 精度要求较高 |
| 1.0 | 🐢 慢 | ⭐⭐⭐⭐⭐ 最高 | 离线分析 |

## 故障排查

### 问题：没有 3D 目标输出

**检查清单**：
1. ✓ 检查 2D 目标是否有输入：`ros2 topic echo /detector/target2d_array`
2. ✓ 检查点云是否有输入：`ros2 topic echo /stereo/points2`
3. ✓ **检查 image_scale 是否一致**
4. ✓ 查看节点日志是否有 "out of bounds" 错误

### 问题：距离估计不准确

**可能原因**：
- image_scale 配置不匹配
- 相机标定参数不准确
- 点云质量差

**解决方法**：
1. 首先确保 image_scale 配置一致
2. 使用 RViz 可视化点云质量
3. 检查相机标定结果

## 快速命令

```bash
# 检查立体视觉配置
grep -A 5 "image_scale" src/stereo_vision/stereo_image_proc_wrapper/config/*.yaml

# 检查距离估计配置
grep -A 5 "image_scale" src/stereo_distance_estimator/config/*.yaml

# 对比两者是否一致
echo "Stereo processing:"
grep "image_scale:" src/stereo_vision/stereo_image_proc_wrapper/config/stereo_params.yaml
echo "Distance estimator:"
grep "image_scale:" src/stereo_distance_estimator/config/stereo_distance_estimator.yaml
```

## 总结

🔑 **核心原则**：两个 `image_scale` 参数必须完全一致！

📝 **修改流程**：
1. 决定立体视觉的缩放比例（推荐 0.5）
2. 在两个配置文件中设置相同的值
3. 编译并测试
4. 验证输出是否正常

💡 **记住**：如果修改了一处的 `image_scale`，必须同步修改另一处！
