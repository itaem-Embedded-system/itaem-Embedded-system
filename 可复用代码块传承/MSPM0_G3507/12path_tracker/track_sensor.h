/**
 * @file    track_sensor.h
 * @brief   12 路循迹传感器逻辑封装（无硬件依赖）
 * @note    仅调用抽象接口，可跨 MCU 复用
 */
#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H

#include "interfaces.h"
#include <stdint.h>

// 循迹传感器状态结构体
typedef struct {
    uint8_t levels[12];    // 12 路电平（0=白，1=黑）
    int8_t  line_position;  // 黑线位置（-5~+5，0=正中间，负数偏左，正数偏右）
    uint8_t is_on_line;     // 是否检测到黑线（1=是，0=否）
} track_sensor_state_t;

// 初始化循迹传感器
uint8_t track_sensor_init(void);

// 更新传感器状态（读取 12 路电平并计算黑线位置）
uint8_t track_sensor_update(track_sensor_state_t* state);

#endif // TRACK_SENSOR_H