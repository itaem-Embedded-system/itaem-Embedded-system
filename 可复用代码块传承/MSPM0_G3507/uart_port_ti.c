/**
 * @file    uart_port_ti.c
 * @brief   TI MSPM0 UART 驱动实现（格式化发送）
 */
#include "uart_port_ti.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <ti/driverlib/dl_uart.h>

// 正确宏定义（和你工程一致）
#define UART0_INST    UART_0_INST

uint8_t ti_uart_init(void)
{
    // SysConfig 已完成硬件初始化
    return 0;
}

// 发送单个字节（无报错版本！）
void ti_uart_send_byte(uint8_t data)
{
    // 直接阻塞发送，兼容所有 MSPM0 库
    DL_UART_Main_transmitData(UART0_INST, data);

    // 等待发送完成（兼容写法）
    while(DL_UART_Main_getPendingInterrupt(UART0_INST) == DL_UART_MAIN_IIDX_TX){};
}

// 发送字符串
void ti_uart_send_string(const char* str)
{
    while (*str != '\0')
    {
        ti_uart_send_byte(*str++);
    }
}

// 格式化发送（printf 功能）
void ti_uart_send_format(const char* format, ...)
{
    char buf[128];
    va_list args;

    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    ti_uart_send_string(buf);
}

// 接口结构体
const uart_interface_t g_ti_uart_if = {
    .init        = ti_uart_init,
    .send_byte   = ti_uart_send_byte,
    .send_string = ti_uart_send_string,
    .send_format = ti_uart_send_format,
};

uart_interface_t* uart_get_ti_if(void)
{
    return (uart_interface_t*)&g_ti_uart_if;
}