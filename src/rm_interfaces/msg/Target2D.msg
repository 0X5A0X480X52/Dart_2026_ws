# Header: timestamp and coordinate frame (e.g., "image_left")
std_msgs/Header header

# 中心坐标（像素，float32 保留精度）
float32 x
float32 y

# 目标框宽高（像素）
float32 width
float32 height

# 检测置信度
float32 confidence

# 目标类别
string class_name

# 目标唯一 ID（可选，跟踪使用）
int32 id

# 是否经过滤波（0=raw, 1=filtered）
bool is_filtered
