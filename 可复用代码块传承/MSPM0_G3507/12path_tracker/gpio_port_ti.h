/**
 * @file    gpio_port_ti.h
 * @brief   TI MSPM0G3507 GPIO 驱动头文件
 */
#ifndef GPIO_PORT_TI_H
#define GPIO_PORT_TI_H

#include "interfaces.h"

// 获取 TI 版本的 GPIO 接口
gpio_digital_interface_t* gpio_digital_get_ti_if(void);
uint8_t ti_gpio_init_input(gpio_port_e port, uint8_t pin_idx);
uint8_t ti_gpio_read_level(gpio_port_e port, uint8_t pin_idx);
uint8_t ti_gpio_read_level_batch(gpio_port_e port, uint8_t levels[12]);


#endif // GPIO_PORT_TI_H