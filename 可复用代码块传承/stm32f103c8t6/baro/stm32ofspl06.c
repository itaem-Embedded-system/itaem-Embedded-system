#include "stm32ofspl06.h"
#include "main.h"  // for HAL handles (hspi1)
#include "spi.h"

#ifndef SPL06_CS_GPIO_Port
#define SPL06_CS_GPIO_Port GPIOA
#endif

#ifndef SPL06_CS_Pin
#define SPL06_CS_Pin GPIO_PIN_10
#endif

// 超时时间定义
#define SPL06_SPI_TIMEOUT_MS   10
#define SPL06_SPI_RETRY_COUNT  3  // SPI 通信重试次数

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} SPL06_PlatformHandle_t;

static SPL06_PlatformHandle_t spl06PlatformHandle = {
    .spi = &hspi1,
    .cs_port = SPL06_CS_GPIO_Port,
    .cs_pin = SPL06_CS_Pin,
};

static void HAL_Delay_Wrapper(uint32_t ms)
{
    HAL_Delay(ms);
}

static void SPL06_CS_Select(SPL06_PlatformHandle_t *platform)
{
    HAL_GPIO_WritePin(platform->cs_port, platform->cs_pin, GPIO_PIN_RESET);
}

static void SPL06_CS_Deselect(SPL06_PlatformHandle_t *platform)
{
    HAL_GPIO_WritePin(platform->cs_port, platform->cs_pin, GPIO_PIN_SET);
}

static int32_t SPI_WriteReg(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    SPL06_PlatformHandle_t *platform = (SPL06_PlatformHandle_t *)handle;

    if (platform == NULL || platform->spi == NULL || data == NULL || len == 0U) {
        return -1;
    }

    for (int retry = 0; retry < SPL06_SPI_RETRY_COUNT; retry++) {
        SPL06_CS_Select(platform);

        uint8_t cmd = reg & 0x7F; // write bit = 0
        if (HAL_SPI_Transmit(platform->spi, &cmd, 1, SPL06_SPI_TIMEOUT_MS) != HAL_OK) {
            SPL06_CS_Deselect(platform);
            continue; // 重试
        }

        if (HAL_SPI_Transmit(platform->spi, (uint8_t*)data, len, SPL06_SPI_TIMEOUT_MS) != HAL_OK) {
            SPL06_CS_Deselect(platform);
            continue; // 重试
        }

        SPL06_CS_Deselect(platform);
        return 0; // 成功
    }
    return -1; // 所有重试失败
}

static int32_t SPI_ReadReg(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    SPL06_PlatformHandle_t *platform = (SPL06_PlatformHandle_t *)handle;

    if (platform == NULL || platform->spi == NULL || data == NULL || len == 0U) {
        return -1;
    }

    for (int retry = 0; retry < SPL06_SPI_RETRY_COUNT; retry++) {
        SPL06_CS_Select(platform);

        uint8_t cmd = reg | 0x80; // read bit = 1
        if (HAL_SPI_Transmit(platform->spi, &cmd, 1, SPL06_SPI_TIMEOUT_MS) != HAL_OK) {
            SPL06_CS_Deselect(platform);
            continue; // 重试
        }

        if (HAL_SPI_Receive(platform->spi, data, len, SPL06_SPI_TIMEOUT_MS) != HAL_OK) {
            SPL06_CS_Deselect(platform);
            continue; // 重试
        }

        SPL06_CS_Deselect(platform);
        return 0; // 成功
    }
    return -1; // 所有重试失败
}

SPL06_Ctx_t* STM32OF_GetSPL06Ctx(void)
{
    static SPL06_Ctx_t ctx = {
        .write_reg = SPI_WriteReg,
        .read_reg  = SPI_ReadReg,
        .delay_ms  = HAL_Delay_Wrapper,
        .handle    = &spl06PlatformHandle,
    };
    return &ctx;
}