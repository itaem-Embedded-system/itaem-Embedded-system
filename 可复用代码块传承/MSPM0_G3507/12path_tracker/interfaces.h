/**
 * @file    interfaces.h
 * @brief   硬件抽象接口定义（循迹模块专用）
 * @note    仅定义接口，不包含任何硬件实现
 */
#ifndef INTERFACES_H
#define INTERFACES_H

#include <stdint.h>

// 端口枚举（支持 A/B 端口）
typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_MAX
} gpio_port_e;

// GPIO 数字接口结构体
typedef struct {
    /**
     * @brief  初始化指定端口的指定引脚为输入模式
     * @param  port     端口标识
     * @param  pin_idx  引脚编号（0~11，对应 PB1~PB12）
     * @return 0=成功，1=端口错误，2=引脚越界
     */
    uint8_t (*init_input)(gpio_port_e port, uint8_t pin_idx);

    /**
     * @brief  读取单个引脚的数字电平
     * @param  port     端口标识
     * @param  pin_idx  引脚编号
     * @return 0=低电平（白），1=高电平（黑），0xFF=读取失败
     */
    uint8_t (*read_level)(gpio_port_e port, uint8_t pin_idx);

    /**
     * @brief  批量读取指定端口的 12 个引脚电平
     * @param  port     端口标识
     * @param  levels   输出数组（长度 12，0=白，1=黑）
     * @return 0=成功，非0=失败
     */
    uint8_t (*read_level_batch)(gpio_port_e port, uint8_t levels[12]);
} gpio_digital_interface_t;

// UART 串口接口
typedef struct {
    uint8_t (*init)(void);
    void (*send_byte)(uint8_t data);
    void (*send_string)(const char* str);
    void (*send_format)(const char* format, ...); // 格式化发送（类似 printf）
} uart_interface_t;

#endif // INTERFACES_H