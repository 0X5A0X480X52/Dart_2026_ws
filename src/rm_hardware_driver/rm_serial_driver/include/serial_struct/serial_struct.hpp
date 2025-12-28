#ifndef SERIAL_STRUCT_HPP
#define SERIAL_STRUCT_HPP

#include <string>
#include "rm_interfaces/msg/target3_d.hpp"

#pragma pack(push, 1)
struct SerialSendData {
    // 目标距离
    float Distance;

    // 像素误差
    // 计算公式：检测框的水平像素-理想点的水平像素
    // 即偏左时<0，偏右时>0
    int Pixel_Error;

    // 是否启用调试模式（true: 启用，false: 不启用）
    bool Debug_Mode;

    // 设定的发射拉力值
    int Shoot_Force_Set;
};
#pragma pack(pop) // <-- 取消字节对齐设置

#pragma pack(push, 1)
struct SerialReceiveData {
    // 初始拉力值
    int Shoot_Force_Init;

    // 当前拉力值
    int Shoot_Force_Cur;
    
    //发射速度
    float Shoot_Vel;
};
#pragma pack(pop) // <-- 取消字节对齐设置

#endif