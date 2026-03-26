#ifndef GNSS_MAXM10S_PORT_H
#define GNSS_MAXM10S_PORT_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* 绑定底层串口并清空接收缓冲。 */
void GNSS_maxm10s_port_init(UART_HandleTypeDef *uart);
/* 从底层串口缓存中提取一整行 NMEA 文本，成功时返回 true。 */
bool GNSS_maxm10s_port_read_line(char *line, uint16_t max_len);

#endif
