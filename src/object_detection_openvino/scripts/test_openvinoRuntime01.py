import cv2
import numpy as np

# 1. 加载模型
model_xml = "./Katrin.xml"
model_bin = "./Katrin.bin"
net = cv2.dnn.readNetFromModelOptimizer(model_xml, model_bin)
# 或者也可以使用
# net = cv2.dnn.readNet(model_xml, model_bin)

# 设置计算后台和目标设备（可选）
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_INFERENCE_ENGINE)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)  # 或 DNN_TARGET_OPENCL, DNN_TARGET_MYRIAD等

# 2. 准备输入数据
# 假设模型需要224x224的BGR图像作为输入
image = cv2.imread("input.jpg")  # 读取输入图像
blob = cv2.dnn.blobFromImage(image, 
                             scalefactor=1.0, 
                             size=(224, 224), 
                             mean=(104, 117, 123), 
                             swapRB=True, 
                             crop=False)

# 3. 进行推理
net.setInput(blob)
output = net.forward()

# 4. 处理输出
# 输出格式取决于模型类型
# 分类模型示例：
if len(output.shape) == 4:  # 有时输出是4维的
    output = output[0]  # 去掉第一个维度
    
if len(output.shape) == 2:  # 典型的分类输出 [1, N]
    output = output[0]
    
# 获取前N个预测结果
N = 5
top_indices = output.argsort()[-N:][::-1]
for i in top_indices:
    print(f"Class {i}: {output[i]:.4f}")

# 目标检测模型示例：
# 输出可能是多个检测框，每个框包含[x, y, w, h, confidence, class1, class2, ...]
# 需要根据具体模型结构解析