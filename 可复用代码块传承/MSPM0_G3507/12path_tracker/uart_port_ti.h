/**
 * @file    uart_port_ti.h
 * @brief   TI MSPM0 UART 驱动头文件
 */
#ifndef UART_PORT_TI_H
#define UART_PORT_TI_H

#include "interfaces.h"

uart_interface_t* uart_get_ti_if(void);

#endif // UART_PORT_TI_H