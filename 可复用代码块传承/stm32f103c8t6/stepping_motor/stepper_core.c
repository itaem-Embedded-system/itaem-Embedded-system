/**
 * @file stepper_core.c
 * @brief 步进电机核心驱动（支持动态创建多个电机）
 */

#include "interfaces.h"
#include <stdlib.h>
#include <string.h>

/*==================== 电机私有数据结构 ====================*/

typedef struct {
    /* 硬件接口 */
    const gpio_if_t* gpio;
    const timer_if_t* timer;
    
    /* 引脚配置 */
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    
    /* 配置参数 */
    struct {
        uint32_t steps_per_rev;
        float max_speed_rpm;
        float min_speed_rpm;
        uint32_t pulse_width_us;
    } config;
    
    /* 运行状态 */
    struct {
        int32_t position;
        float speed_rpm;
        stepper_dir_t direction;
        bool enabled;
        bool moving;
    } state;
    
    uint8_t initialized;
} motor_ctx_t;

/*==================== 全局接口（所有电机共享）====================*/

static const gpio_if_t* g_gpio = NULL;
static const timer_if_t* g_timer = NULL;

/* 注册硬件接口 */
void stepper_core_register_interfaces(const gpio_if_t* gpio_if, const timer_if_t* timer_if)
{
    if (gpio_if) g_gpio = gpio_if;
    if (timer_if) g_timer = timer_if;
}

/*==================== 内部函数 ====================*/

static uint32_t rpm_to_interval(float rpm, uint32_t steps_per_rev)
{
    if (rpm <= 0 || steps_per_rev == 0) return 0;
    float pulses_per_sec = (rpm / 60.0f) * steps_per_rev;
    return (uint32_t)(1000000.0f / pulses_per_sec);
}

static error_t motor_generate_pulse(motor_ctx_t* motor)
{
    if (!motor->state.enabled) return ERR_BUSY;
    
    /* 共阴极：高电平有效 */
    motor->gpio->write(motor->step_pin, GPIO_LEVEL_HIGH);
    motor->timer->delay_us(motor->config.pulse_width_us);
    motor->gpio->write(motor->step_pin, GPIO_LEVEL_LOW);
    
    return ERR_OK;
}

/*==================== 创建/销毁电机 ====================*/

/**
 * @brief 动态创建一个电机实例
 * @return 电机句柄，失败返回NULL
 */
void* stepper_core_create(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, 
                          uint32_t steps_per_rev)
{
    motor_ctx_t* motor;
    
    if (!g_gpio || !g_timer) return NULL;
    
    motor = (motor_ctx_t*)malloc(sizeof(motor_ctx_t));
    if (!motor) return NULL;
    
    /* 清零 */
    memset(motor, 0, sizeof(motor_ctx_t));
    
    /* 保存接口 */
    motor->gpio = g_gpio;
    motor->timer = g_timer;
    
    /* 引脚配置 */
    motor->step_pin = step_pin;
    motor->dir_pin = dir_pin;
    motor->enable_pin = enable_pin;
    
    /* 参数配置 */
    motor->config.steps_per_rev = steps_per_rev;
    motor->config.max_speed_rpm = 300.0f;
    motor->config.min_speed_rpm = 10.0f;
    motor->config.pulse_width_us = 10;
    
    /* 初始状态 */
    motor->state.speed_rpm = 60.0f;
    motor->initialized = 1;
    
    /* 初始引脚状态 */
    motor->gpio->write(step_pin, GPIO_LEVEL_LOW);
    motor->gpio->write(dir_pin, GPIO_LEVEL_LOW);
    motor->gpio->write(enable_pin, GPIO_LEVEL_LOW);
    
    return (void*)motor;
}

/**
 * @brief 销毁电机实例
 */
void stepper_core_destroy(void* handle)
{
    if (handle) {
        free(handle);
    }
}

/*==================== 电机操作函数 ====================*/

error_t stepper_core_enable(void* handle, bool en)
{
    motor_ctx_t* motor = (motor_ctx_t*)handle;
    if (!motor || !motor->initialized) return ERR_PARAM;
    
    motor->gpio->write(motor->enable_pin, en ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
    motor->state.enabled = en;
    return ERR_OK;
}

error_t stepper_core_move(void* handle, int32_t steps)
{
    motor_ctx_t* motor = (motor_ctx_t*)handle;
    if (!motor || !motor->initialized) return ERR_PARAM;
    if (!motor->state.enabled) return ERR_BUSY;
    
    /* 设置方向 */
    if (steps > 0) {
        motor->gpio->write(motor->dir_pin, GPIO_LEVEL_HIGH);
        motor->state.direction = STEPPER_DIR_CW;
    } else {
        motor->gpio->write(motor->dir_pin, GPIO_LEVEL_LOW);
        motor->state.direction = STEPPER_DIR_CCW;
        steps = -steps;
    }
    
    /* 计算脉冲间隔 */
    uint32_t interval_us = rpm_to_interval(motor->state.speed_rpm, motor->config.steps_per_rev);
    if (interval_us == 0) return ERR_PARAM;
    
    /* 执行运动 */
    motor->state.moving = true;
    for (uint32_t i = 0; i < (uint32_t)steps; i++) {
        error_t err = motor_generate_pulse(motor);
        if (err != ERR_OK) {
            motor->state.moving = false;
            return err;
        }
        
        if (steps > 0) motor->state.position++;
        else motor->state.position--;
        
        if (interval_us > motor->config.pulse_width_us) {
            motor->timer->delay_us(interval_us - motor->config.pulse_width_us);
        }
    }
    motor->state.moving = false;
    
    return ERR_OK;
}

error_t stepper_core_set_speed(void* handle, float rpm)
{
    motor_ctx_t* motor = (motor_ctx_t*)handle;
    if (!motor || !motor->initialized) return ERR_PARAM;
    
    if (rpm > motor->config.max_speed_rpm) rpm = motor->config.max_speed_rpm;
    if (rpm < motor->config.min_speed_rpm && rpm > 0) rpm = motor->config.min_speed_rpm;
    
    motor->state.speed_rpm = rpm;
    return ERR_OK;
}

error_t stepper_core_stop(void* handle)
{
    motor_ctx_t* motor = (motor_ctx_t*)handle;
    if (!motor) return ERR_PARAM;
    motor->state.moving = false;
    return ERR_OK;
}

int32_t stepper_core_get_position(void* handle)
{
    motor_ctx_t* motor = (motor_ctx_t*)handle;
    return motor ? motor->state.position : 0;
}

error_t stepper_core_get_state(void* handle, stepper_state_t* state)
{
    motor_ctx_t* motor = (motor_ctx_t*)handle;
    if (!motor || !state) return ERR_PARAM;
    
    state->position = motor->state.position;
    state->speed_rpm = motor->state.speed_rpm;
    state->direction = motor->state.direction;
    state->enabled = motor->state.enabled;
    state->moving = motor->state.moving;
    return ERR_OK;
}
