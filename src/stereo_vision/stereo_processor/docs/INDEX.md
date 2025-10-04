# Stereo Processor 文档索引

欢迎使用 Stereo Processor！这是一个完整的 ROS 2 双目立体视觉处理包。

## 📚 文档导航

### 🚀 快速开始
如果您是第一次使用，请按以下顺序阅读：

1. **[QUICKSTART.md](QUICKSTART.md)** - 5分钟快速入门
   - 编译、安装、启动
   - 基本验证
   - 常见问题快速解决

2. **[README.md](README.md)** - 包概述和功能说明
   - 功能特性
   - Topic 列表
   - 参数说明

### 📖 详细指南

3. **[MIGRATION_GUIDE.md](MIGRATION_GUIDE.md)** - 迁移和使用指南
   - 从 target_matcher 迁移
   - 系统集成方法
   - 参数调优详解
   - 故障排除
   - 相机标定指南

4. **[DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)** - 部署检查清单
   - 编译前检查
   - 运行前检查
   - 启动检查
   - 性能检查
   - 系统集成检查

### 🔧 技术文档

5. **[IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)** - 实现总结
   - 技术架构
   - 核心算法实现
   - 性能特性
   - 后续改进建议

6. **[docs/system_architecture.dot](docs/system_architecture.dot)** - 系统架构图
   - 可视化数据流
   - 节点关系
   - Topic 映射

### 📁 配置文件

7. **[config/stereo_processor.yaml](config/stereo_processor.yaml)** - 参数配置
   - SGBM 算法参数
   - 视差计算参数
   - 点云生成参数

### 🚦 Launch 文件

8. **[launch/stereo_processor.launch.py](launch/stereo_processor.launch.py)** - 单节点启动
9. **[launch/full_stereo_system.launch.py](launch/full_stereo_system.launch.py)** - 完整系统启动

## 🎯 按需求查找

### 我想...

#### 快速开始使用
→ [QUICKSTART.md](QUICKSTART.md)

#### 了解功能和 Topics
→ [README.md](README.md)

#### 集成到现有系统
→ [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - "系统集成"章节

#### 调整参数以提高性能
→ [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - "参数调优"章节

#### 解决视差图质量问题
→ [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - "故障排除"章节

#### 重新标定相机
→ [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - "相机标定"章节

#### 部署到生产环境
→ [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)

#### 了解实现细节
→ [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)

#### 查看系统架构
→ [docs/system_architecture.dot](docs/system_architecture.dot)
```bash
# 生成 PNG 图像
dot -Tpng docs/system_architecture.dot -o architecture.png
```

## 🔍 按问题类型查找

### 编译/安装问题
- [QUICKSTART.md](QUICKSTART.md) - 步骤 1
- [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) - 部署前检查

### 运行时错误
- [QUICKSTART.md](QUICKSTART.md) - 常见问题 Q1
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - 故障排除

### 视差图/点云质量问题
- [QUICKSTART.md](QUICKSTART.md) - 常见问题 Q2
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - 故障排除 & 参数调优
- [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) - 优化建议

### 性能问题
- [QUICKSTART.md](QUICKSTART.md) - 常见问题 Q3
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - 性能优化
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - 性能特性

### 系统集成问题
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - 系统集成
- [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) - 系统集成检查

## 📦 文件结构

```
target_matcher/ (stereo_processor 包)
│
├── 文档
│   ├── INDEX.md                        # 本文件 - 文档索引
│   ├── QUICKSTART.md                   # 快速开始
│   ├── README.md                       # 包说明
│   ├── MIGRATION_GUIDE.md              # 详细使用指南
│   ├── DEPLOYMENT_CHECKLIST.md         # 部署检查清单
│   ├── IMPLEMENTATION_SUMMARY.md       # 实现总结
│   └── docs/system_architecture.dot    # 系统架构图
│
├── 源代码
│   ├── include/stereo_processor/
│   │   └── stereo_processor_node.hpp   # 节点头文件
│   └── src/
│       └── stereo_processor_node.cpp   # 节点实现
│
├── 配置
│   ├── config/
│   │   └── stereo_processor.yaml       # 参数配置
│   └── launch/
│       ├── stereo_processor.launch.py  # 单节点启动
│       └── full_stereo_system.launch.py# 完整系统启动
│
├── 构建配置
│   ├── CMakeLists.txt                  # CMake 配置
│   └── package.xml                     # ROS 2 包描述
│
└── 工具
    └── test_node.sh                    # 测试脚本
```

## 🎓 学习路径

### 初学者
1. 阅读 [QUICKSTART.md](QUICKSTART.md)
2. 运行 `test_node.sh` 验证安装
3. 启动节点并在 RViz2 中查看结果
4. 尝试调整基本参数

### 中级用户
1. 阅读 [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md)
2. 将节点集成到您的系统中
3. 根据场景调优参数
4. 使用 [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) 验证部署

### 高级用户
1. 阅读 [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)
2. 理解核心算法实现
3. 考虑性能优化
4. 贡献代码改进

## 🔗 外部资源

- [OpenCV Stereo Vision](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)
- [ROS 2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [Camera Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)
- [SGBM Algorithm Paper](https://core.ac.uk/download/pdf/11134866.pdf)

## 🆘 需要帮助？

1. **查看文档**: 按上述索引查找相关章节
2. **运行测试**: 使用 `test_node.sh` 诊断问题
3. **检查日志**: 节点会输出详细的状态信息
4. **使用检查清单**: [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)
5. **联系维护者**: amatrix02 (3432900546@qq.com)

## 📝 文档更新日志

- **2025-10-04**: 初始文档创建
  - 完整的使用指南
  - 部署检查清单
  - 实现技术文档
  - 系统架构图

## 📄 许可证

Apache-2.0 License

---

**提示**: 建议从 [QUICKSTART.md](QUICKSTART.md) 开始，它只需要 5 分钟！
