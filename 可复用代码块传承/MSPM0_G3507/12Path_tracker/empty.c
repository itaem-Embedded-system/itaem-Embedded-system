/**
 * @file    app_track.c
 * @brief   循迹应用层（仅 UART 打印）
 */
#include "empty.h"
#include "track_sensor.h"
#include "track_uart.h"
#include "gpio_port_ti.h"
#include "ti_msp_dl_config.h"

static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __NOP();
    }
}

void app_track_main(void) {
    track_sensor_state_t sensor_state;
    
    // 初始化 UART
    uint8_t ret = track_uart_init();
    if (ret != 0) {
        while (1);
    }
    
    // 初始化传感器
    ret = track_sensor_init();
    if (ret != 0) {
        while (1);
    }
        while (1) {
        DL_GPIO_togglePins(LED1_PORT, LED1_PIN_22_PIN);

    // 更新传感器状态（会内部调用批量读取）
        track_sensor_update(&sensor_state);
    
    // 通过 UART 发送传感器状态
        track_uart_send(&sensor_state);
        
        delay_ms(500);  // 建议改为 500ms
        }        
    }

int main(void) {
  // 初始化全部（时钟+GPIO+UART+波特率9600+PA28 TX）
    SYSCFG_DL_init();
    app_track_main();
    return 0;
}
