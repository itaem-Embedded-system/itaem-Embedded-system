#ifndef PLATFORM_STM32OFSPL06_H
#define PLATFORM_STM32OFSPL06_H

#include <stdint.h>
#include "spl06.h"

#ifdef __cplusplus
extern "C" {
#endif

// 获取 STM32 HAL SPI 实现的 SPL06 上下文
SPL06_Ctx_t* STM32OF_GetSPL06Ctx(void);

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_STM32OFSPL06_H