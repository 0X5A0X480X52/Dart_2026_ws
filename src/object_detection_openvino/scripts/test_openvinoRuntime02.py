import cv2
import numpy as np
from openvino.runtime import Core

# 1. 加载模型
model_xml = "./Katrin.xml"
model_bin = "./Katrin.bin"
core = Core()
model = core.read_model(model=model_xml)
compiled_model = core.compile_model(model=model, device_name="CPU")

# 2. 输入预处理
input_layer = compiled_model.input(0)
input_shape = input_layer.shape  # 例如 [1,3,640,640] (NCHW)
image = cv2.imread(r"/home/amatrix/Dart_2026_ws/src/object_detection_openvino/scripts/test02.png")
resized_image = cv2.resize(image, (input_shape[3], input_shape[2]))  # 调整到模型输入尺寸
input_data = np.expand_dims(resized_image.transpose(2,0,1), axis=0).astype(np.float32)  # HWC → NCHW

# 3. 推理
output_layer = compiled_model.output(0)
detections = compiled_model([input_data])[output_layer]  # 形状 (1, 15120, 27)

print(f"Output shape: {detections.shape}")  # 输出形状 (1, 15120, 27)
print(f"Output data: {detections[0][:5]}")  # 输出数据

# 4. 解析检测结果
conf_threshold = 0.5  # 置信度阈值
class_threshold = 0.5  # 类别概率阈值
boxes = []
scores = []
class_ids = []

# 遍历所有候选框
for detection in detections[0]:  # detections[0] 形状 (15120, 27)
    confidence = detection[4]
    if confidence < conf_threshold:
        continue  # 跳过低置信度框
    
    # 提取类别概率
    class_probs = detection[5:]
    class_id = np.argmax(class_probs)
    class_score = class_probs[class_id]
    
    if class_score < class_threshold:
        continue  # 跳过低类别概率框
    
    # 提取边界框坐标（假设格式为 x_center, y_center, width, height）
    x_center, y_center, width, height = detection[0], detection[1], detection[2], detection[3]
    
    # 转换为图像坐标（x_min, y_min, x_max, y_max）
    x_min = int((x_center - width/2) * image.shape[1])
    y_min = int((y_center - height/2) * image.shape[0])
    x_max = int((x_center + width/2) * image.shape[1])
    y_max = int((y_center + height/2) * image.shape[0])
    
    boxes.append([x_min, y_min, x_max, y_max])
    scores.append(float(class_score))
    class_ids.append(class_id)

# 5. 应用非极大值抑制 (NMS)
nms_threshold = 0.5
indices = cv2.dnn.NMSBoxes(boxes, scores, conf_threshold, nms_threshold)

# 6. 绘制最终检测框
for i in indices:
    box = boxes[i]
    x_min, y_min, x_max, y_max = box
    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
    label = f"Class {class_ids[i]}: {scores[i]:.2f}"
    print(label)
    cv2.putText(image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

cv2.imwrite("output2.jpg", image)