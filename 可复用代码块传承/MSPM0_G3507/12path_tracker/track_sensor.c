/**
 * @file    track_sensor.c
 * @brief   12 路循迹传感器逻辑实现
 */
 #include <stdio.h>
#include "track_sensor.h"
#include "gpio_port_ti.h"

// 全局接口对象（仅初始化一次） 
static gpio_digital_interface_t* s_gpio_if = NULL;

uint8_t track_sensor_init(void) {
    // 绑定TI的GPIO接口（从gpio_port_ti.c获取）
    s_gpio_if = gpio_digital_get_ti_if();
    return (s_gpio_if == NULL) ? 1 : 0;
}

// 更新传感器状态
uint8_t track_sensor_update(track_sensor_state_t* state) {
    if (s_gpio_if == NULL || state == NULL) return 1;

    // 批量读取 12 路电平
    uint8_t ret = s_gpio_if->read_level_batch(GPIO_PORT_B, state->levels);
    if (ret != 0) return ret;

    int32_t sum_weight = 0;
    int32_t sum_level = 0;
    const int8_t weights[12] = {-5, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5};

    for (int i = 0; i < 12; i++) {
        // 循迹传感器：黑线 = 低电平 0 ，白色 = 高电平 1
        uint8_t has_black = (state->levels[i] == 0) ? 1 : 0 ;

        sum_weight += has_black * weights[i];
        sum_level += has_black;
    }

    if (sum_level == 0) {
        state->is_on_line = 0;
        state->line_position = 0;
    } else {
        state->is_on_line = 1;
        state->line_position = (int8_t)(sum_weight / sum_level);
    }

    return 0;
}