# Header: timestamp and coordinate frame
std_msgs/Header header

# 统一的目标 ID（同一目标在多相机下的标识）
int32 id

# 不同相机视角的检测结果
Target2D[] views

# 目标类别
string class_name