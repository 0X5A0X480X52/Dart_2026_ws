# Header: timestamp and coordinate frame (e.g., "camera_link")
std_msgs/Header header

# 相机坐标系下三维坐标 (m)
geometry_msgs/Point position

# 估计距离 (m)
float32 distance

# 检测置信度
float32 confidence

# 目标类别 ID（0=未知, 1=装甲板, 2=能量机关等，根据实际定义）
int32 class_id

# 目标唯一 ID（可选，跟踪使用）
int32 id

# 是否经过滤波（0=raw, 1=filtered）
bool is_filtered
