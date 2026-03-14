/**
 * @file stepper_core.h
 * @brief 步进电机核心驱动头文件（支持动态创建多个电机）
 */

#ifndef __STEPPER_CORE_H
#define __STEPPER_CORE_H

#include "interfaces.h"

/*==================== 电机句柄类型 ====================*/
typedef void* stepper_handle_t;

/*==================== 全局接口注册 ====================*/

/**
 * @brief 注册硬件接口（必须在使用前调用一次）
 * @param gpio_if GPIO接口指针
 * @param timer_if 定时器接口指针
 */
void stepper_core_register_interfaces(const gpio_if_t* gpio_if, const timer_if_t* timer_if);

/*==================== 电机创建/销毁 ====================*/

/**
 * @brief 创建一个新的电机实例
 * @param step_pin STEP引脚号
 * @param dir_pin DIR引脚号
 * @param enable_pin ENA引脚号
 * @param steps_per_rev 每转步数（如800）
 * @return 电机句柄，失败返回NULL
 */
stepper_handle_t stepper_core_create(uint8_t step_pin, uint8_t dir_pin, 
                                     uint8_t enable_pin, uint32_t steps_per_rev);

/**
 * @brief 销毁电机实例
 * @param handle 电机句柄
 */
void stepper_core_destroy(stepper_handle_t handle);

/*==================== 电机控制函数 ====================*/

/**
 * @brief 使能/失能电机
 * @param handle 电机句柄
 * @param en true-使能 false-失能
 * @return error_t 错误码
 */
error_t stepper_core_enable(stepper_handle_t handle, bool en);

/**
 * @brief 移动指定步数
 * @param handle 电机句柄
 * @param steps 步数（正数正转，负数反转）
 * @return error_t 错误码
 */
error_t stepper_core_move(stepper_handle_t handle, int32_t steps);

/**
 * @brief 设置目标速度
 * @param handle 电机句柄
 * @param rpm 转速(RPM)
 * @return error_t 错误码
 */
error_t stepper_core_set_speed(stepper_handle_t handle, float rpm);

/**
 * @brief 紧急停止
 * @param handle 电机句柄
 * @return error_t 错误码
 */
error_t stepper_core_stop(stepper_handle_t handle);

/*==================== 状态查询函数 ====================*/

/**
 * @brief 获取当前位置
 * @param handle 电机句柄
 * @return 当前位置（步数）
 */
int32_t stepper_core_get_position(stepper_handle_t handle);

/**
 * @brief 获取电机状态
 * @param handle 电机句柄
 * @param state 状态输出指针
 * @return error_t 错误码
 */
error_t stepper_core_get_state(stepper_handle_t handle, stepper_state_t* state);

/**
 * @brief 复位位置计数器
 * @param handle 电机句柄
 */
void stepper_core_reset_position(stepper_handle_t handle);

#endif /* __STEPPER_CORE_H */
