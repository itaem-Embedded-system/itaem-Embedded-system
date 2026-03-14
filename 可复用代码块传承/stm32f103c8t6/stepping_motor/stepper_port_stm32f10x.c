/**
 * @file stepper_port_stm32f10x.c
 * @brief STM32F10x平台步进电机适配层
 */

#include "interfaces.h"
#include "stepper_core.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stddef.h>  /* 添加这行，定义NULL */

/*==================== 引脚映射表 ====================*/

static const uint16_t pin_map[16] = {
    GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3,
    GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7,
    GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10, GPIO_Pin_11,
    GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15
};

/*==================== GPIO接口实现 ====================*/

static void stm32_gpio_write(uint8_t pin, gpio_level_t level)
{
    if (level == GPIO_LEVEL_HIGH) {
        GPIOA->BSRR = pin_map[pin];
    } else {
        GPIOA->BRR = pin_map[pin];
    }
}

static gpio_level_t stm32_gpio_read(uint8_t pin)
{
    return (GPIOA->IDR & pin_map[pin]) ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
}

static void stm32_gpio_toggle(uint8_t pin)
{
    if (GPIOA->ODR & pin_map[pin]) {
        GPIOA->BRR = pin_map[pin];
    } else {
        GPIOA->BSRR = pin_map[pin];
    }
}

/*==================== 定时器接口实现 ====================*/

static void simple_delay_us(uint32_t us)
{
    for (volatile uint32_t i = 0; i < us * 10; i++) {
        __NOP();
    }
}

static void stm32_delay_us(uint32_t us)
{
    simple_delay_us(us);
}

static void stm32_delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms; i++) {
        simple_delay_us(1000);
    }
}

static uint32_t stm32_get_tick(void)
{
    return 0;
}

/*==================== 接口实例 ====================*/

static const gpio_if_t stm32_gpio_if = {
    .write = stm32_gpio_write,
    .read = stm32_gpio_read,
    .toggle = stm32_gpio_toggle
};

static const timer_if_t stm32_timer_if = {
    .delay_us = stm32_delay_us,
    .delay_ms = stm32_delay_ms,
    .get_tick = stm32_get_tick
};

/*==================== 引脚初始化函数 ====================*/

/**
 * @brief 初始化单个电机的三个引脚
 * @param step_pin STEP引脚号
 * @param dir_pin DIR引脚号
 * @param enable_pin ENA引脚号
 */
void stepper_port_pins_init(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t pins = 0;
    
    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /* 组合引脚 */
    pins = pin_map[step_pin] | pin_map[dir_pin] | pin_map[enable_pin];
    
    /* 配置为推挽输出 */
    GPIO_InitStructure.GPIO_Pin = pins;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 初始状态全部拉低 */
    GPIO_ResetBits(GPIOA, pins);
}

/*==================== 平台初始化 ====================*/

/**
 * @brief 平台初始化（只需调用一次）
 */
void stepper_platform_init(void)
{
    stepper_core_register_interfaces(&stm32_gpio_if, &stm32_timer_if);
}
