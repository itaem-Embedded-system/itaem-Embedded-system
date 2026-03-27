/**
 * @file    gpio_port_ti.c
 * @brief   TI MSPM0G3507 GPIO 驱动实现（PB1~PB12 对应 12 路循迹）
 */
#include <stdio.h>
#include "gpio_port_ti.h"
#include "ti_msp_dl_config.h"

/**
 * @brief 12路循迹传感器对应的GPIO引脚编号数组（PB1 ~ PB12）
 * @note 索引0~11 分别对应 PB1 ~ PB12
 */
static const uint32_t g_ti_track_pins[12] = {
    DL_GPIO_PIN_1,
    DL_GPIO_PIN_2,
    DL_GPIO_PIN_3,
    DL_GPIO_PIN_4,
    DL_GPIO_PIN_5,
    DL_GPIO_PIN_6,
    DL_GPIO_PIN_7,
    DL_GPIO_PIN_8,
    DL_GPIO_PIN_9,
    DL_GPIO_PIN_10,
    DL_GPIO_PIN_11,
    DL_GPIO_PIN_12
};

/**
 * @brief  获取循迹传感器使用的GPIO端口基地址（固定为GPIOB）
 * @return GPIOB端口基地址指针
 * @note 静态内部函数，仅本文件可调用
 */
//返回GPIOB的端口号
static GPIO_Regs* get_track_port_base(void) {
    return GPIOB;
}

/**
 * @brief  初始化指定GPIO引脚为数字输入模式
 * @param  port: 端口号，本驱动仅支持GPIO_PORT_B
 * @param  pin_idx: 引脚索引（0~11 对应 PB1~PB12）
 * @retval 0=成功，1=端口错误，2=引脚索引越界
 */
//初始化 GPIO 引脚为数字输入模式
uint8_t ti_gpio_init_input(gpio_port_e port, uint8_t pin_idx) {
    if (port != GPIO_PORT_B) return 1;
    if (pin_idx >= 12) return 2;

    DL_GPIO_initDigitalInput(g_ti_track_pins[pin_idx]);
    return 0;
}

/**
 * @brief  读取单个循迹GPIO引脚的电平
 * @param  port: 端口号，仅支持GPIO_PORT_B
 * @param  pin_idx: 引脚索引（0~11）
 * @retval 0=低电平，1=高电平，0xFF=参数错误
 */
//读取单个引脚电平
uint8_t ti_gpio_read_level(gpio_port_e port, uint8_t pin_idx) {
    if (port != GPIO_PORT_B || pin_idx >= 12) return 0xFF;
    GPIO_Regs* port_base = get_track_port_base();
    return DL_GPIO_readPins(port_base, g_ti_track_pins[pin_idx]) ? 1 : 0;
}

/**
 * @brief  批量读取12路循迹传感器的全部引脚电平
 * @param  port: 端口号，仅支持GPIO_PORT_B
 * @param  levels: 存储12路电平的输出数组（uint8_t类型）
 * @retval 0=成功，1=参数错误（端口错误/指针为空）
 */
//批量读取12个引脚的电平
uint8_t ti_gpio_read_level_batch(gpio_port_e port, uint8_t levels[12])
{
    if (port != GPIO_PORT_B || levels == NULL) {
        return 1;
    }

    for (int i = 0; i < 12; i++) {
        levels[i] = ti_gpio_read_level(port, i);
    }

    return 0;
}

/**
 * @brief  GPIO数字操作接口实例（绑定初始化、单路读取、批量读取函数）
 * @note  供上层统一调用，实现硬件无关化
 */
const gpio_digital_interface_t g_ti_gpio_if = {
    .init_input = ti_gpio_init_input,
    .read_level = ti_gpio_read_level,
    .read_level_batch = ti_gpio_read_level_batch
};

/**
 * @brief  获取TI平台GPIO数字接口的指针
 * @return 指向g_ti_gpio_if的指针
 * @note  上层通过该函数获取操作接口，实现跨平台兼容
 */
gpio_digital_interface_t* gpio_digital_get_ti_if(void) {
    return (gpio_digital_interface_t*)&g_ti_gpio_if;
}
