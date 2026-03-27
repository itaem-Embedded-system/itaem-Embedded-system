/**
 * @file    track_uart.c
 * @brief   循迹数据 UART 发送实现
 */
#include <stdio.h>
#include "track_uart.h"
#include "uart_port_ti.h"
#include "ti_msp_dl_config.h"

uart_interface_t* s_uart_if = NULL;

//串口初始化函数
uint8_t track_uart_init(void) {
    s_uart_if = uart_get_ti_if();
    return 0;
}

void track_uart_send(const track_sensor_state_t* state)
{
//发送传感器数据
for (int i = 0; i < 12; i++) {
        uint8_t val = state->levels[i];
        s_uart_if->send_byte(val ? '1' : '0');
        while (DL_UART_Main_isBusy(UART_0_INST)) {}
    }
    while (DL_UART_Main_isBusy(UART_0_INST)) {}
    s_uart_if->send_byte('\r');
    while (DL_UART_Main_isBusy(UART_0_INST)) {}
    s_uart_if->send_byte('\n');
}
