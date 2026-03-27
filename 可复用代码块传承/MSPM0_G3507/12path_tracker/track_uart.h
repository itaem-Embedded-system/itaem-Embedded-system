/**
 * @file    track_uart.h
 * @brief   循迹数据 UART 发送封装
 */
#ifndef TRACK_UART_H
#define TRACK_UART_H

#include "track_sensor.h"

// 初始化 UART
uint8_t track_uart_init(void);

// 通过 UART 发送循迹数据
void track_uart_send(const track_sensor_state_t* state);
    
#endif // TRACK_UART_H