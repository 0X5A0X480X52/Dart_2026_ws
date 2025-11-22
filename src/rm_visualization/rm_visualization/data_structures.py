from dataclasses import dataclass

# 定义输入参数结构体
@dataclass
class DartParams:
    dart1_power: str = ""
    desiredPosX: str = ""

# 定义显示参数结构体
@dataclass
class DartDisplayParams:
    # 初始拉力值
    dart1_init: str = ""
    
    # 当前拉力值
    dart1_cur: str = ""
    
    # 上次发射速度
    dart1_vel: str = ""

    curr_pos_x: str = ""