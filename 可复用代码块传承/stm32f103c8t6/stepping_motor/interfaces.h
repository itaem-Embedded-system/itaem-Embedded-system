/**
 * @file interfaces.h
 * @brief 全局抽象接口定义（完全不用改）
 */
#ifndef __INTERFACES_H
#define __INTERFACES_H

#include <stdint.h>
#include <stdbool.h>

/* 错误码 */
typedef enum {
    ERR_OK = 0,
    ERR_PARAM = -1,
    ERR_BUSY = -2,
    ERR_HARDWARE = -3,
    ERR_NOT_SUPPORTED = -4
} error_t;

/* GPIO电平 */
typedef enum {
    GPIO_LEVEL_LOW = 0,
    GPIO_LEVEL_HIGH
} gpio_level_t;

/* 电机方向 */
typedef enum {
    STEPPER_DIR_CW = 0,
    STEPPER_DIR_CCW
} stepper_dir_t;

/* 电机状态 */
typedef struct {
    int32_t position;
    float speed_rpm;
    stepper_dir_t direction;
    bool enabled;
    bool moving;
} stepper_state_t;

/* GPIO接口 */
typedef struct {
    void (*write)(uint8_t pin, gpio_level_t level);
    gpio_level_t (*read)(uint8_t pin);
    void (*toggle)(uint8_t pin);
} gpio_if_t;

/* 定时器接口 */
typedef struct {
    void (*delay_us)(uint32_t us);
    void (*delay_ms)(uint32_t ms);
    uint32_t (*get_tick)(void);
} timer_if_t;

/* 步进电机接口 */
typedef struct {
    error_t (*init)(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint32_t steps_per_rev);
    error_t (*enable)(bool en);
    error_t (*move_steps)(int32_t steps);
    error_t (*set_speed)(float rpm);
    error_t (*stop)(void);
    error_t (*get_state)(stepper_state_t* state);
    int32_t (*get_position)(void);
    void (*reset_position)(void);
} stepper_if_t;

#endif
