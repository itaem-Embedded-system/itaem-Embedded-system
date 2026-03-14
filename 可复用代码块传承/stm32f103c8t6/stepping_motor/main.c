/**
 * @file main.c
 * @brief 应用层:多电机复用
 */

#include "stm32f10x.h"
#include "interfaces.h"
#include "stepper_core.h"  /* 包含这个头文件 */
#include <stddef.h>        /* 定义NULL */

/* 外部声明 */
extern void stepper_platform_init(void);
extern void stepper_port_pins_init(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin);

/* 简单延时 */
static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 8000; i++);
}

int main(void)
{
    /* 电机句柄 - 想用几个定义几个 */
    stepper_handle_t motor1 = NULL;  /* 现在NULL有定义了 */
    stepper_handle_t motor2 = NULL;
    
    /* 系统初始化 */
    SystemInit();
    
    /* 平台初始化 */
    stepper_platform_init();
    
    /*==================== 想用几个电机就创建几个 ====================*/
    
    /* 初始化引脚 */
    stepper_port_pins_init(0, 1, 2);     // 电机1: PA0,PA1,PA2
    stepper_port_pins_init(3, 4, 5);     // 电机2: PA3,PA4,PA5
    
   
    motor1 = stepper_core_create(0, 1, 2, 800);
    motor2 = stepper_core_create(3, 4, 5, 800);
    
    /* 检查创建是否成功 */
    if (motor1 == NULL || motor2 == NULL) {
        while(1);  // 创建失败
    }
    
    /*==================== 电机控制 ====================*/
    
    /* 设置速度 */
    stepper_core_set_speed(motor1, 60.0f);
    stepper_core_set_speed(motor2, 120.0f);
    
    /* 使能电机 */
    stepper_core_enable(motor1, true);
    stepper_core_enable(motor2, true);
    
    delay_ms(100);
    
    /* 主循环 */
    while(1)
    {
        /* 电机1正转1圈 */
        stepper_core_move(motor1, 800);
        delay_ms(500);
        
        /* 电机2正转2圈 */
        stepper_core_move(motor2, 1600);
        delay_ms(500);
        
        /* 电机1反转1圈 */
        stepper_core_move(motor1, -800);
        delay_ms(500);
        
        /* 电机2反转2圈 */
        stepper_core_move(motor2, -1600);
        delay_ms(1000);
    }
}
