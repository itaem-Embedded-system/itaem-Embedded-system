#ifndef SPL06_H
#define SPL06_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// SPL06 register definition
#define SPL_CHIP_ID            0x10
#define SPL_CHIP_ADDRESS       0x76

#define SPL_PRS_B2             0x00
#define SPL_PRS_B1             0x01
#define SPL_PRS_B0             0x02

#define SPL_TMP_B2             0x03
#define SPL_TMP_B1             0x04
#define SPL_TMP_B0             0x05

#define SPL_PRS_CFG            0x06
#define SPL_TMP_CFG            0x07
#define SPL_MEAS_CFG           0x08
#define SPL_CFG_REG            0x09
#define SPL_INT_STS            0x0A
#define SPL_FIFO_STS           0x0B
#define SPL_RESET_REG          0x0C

#define SPL_ID_REG             0x0D

#define COEF_C0                0x10
#define COEF_C0_C1             0x11
#define COEF_C1                0x12
#define COEF_C00_H             0x13
#define COEF_C00_L             0x14
#define COEF_C00_C10           0x15
#define COEF_C10_M             0x16
#define COEF_C10_L             0x17
#define COEF_C01_H             0x18
#define COEF_C01_L             0x19
#define COEF_C11_H             0x1A
#define COEF_C11_L             0x1B
#define COEF_C20_H             0x1C
#define COEF_C20_L             0x1D
#define COEF_C21_H             0x1E
#define COEF_C21_L             0x1F
#define COEF_C30_H             0x20
#define COEF_C30_L             0x21

// SPL06校准系数结构体定义
typedef enum {
    SPL06_ERR_NONE = 0,
    SPL06_ERR_PARAM = -1,    // 参数错误
    SPL06_ERR_COMM = -2,     // 通信错误
    SPL06_ERR_TIMEOUT = -3,  // 超时
    SPL06_ERR_CONFIG = -4,   // 配置错误
    SPL06_ERR_ID = -5,       // ID 不匹配
} SPL06_Error_t;

// SPL06校准系数结构体定义
typedef struct {
    int16_t c0;
    int16_t c1;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
    int32_t c00;
    int32_t c10;
} SPL06_Calib_t;

// SPL06上下文结构体 - 实现上下文注入
typedef struct {
    // 读写接口注入
    int32_t (*write_reg)(void *handle, uint8_t reg, const uint8_t *data, uint16_t len);
    int32_t (*read_reg)(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
    // 延时接口注入
    void (*delay_ms)(uint32_t ms);
    // 硬件句柄（透传给读写接口）
    void *handle;
} SPL06_Ctx_t;

/**
 * @brief 初始化 SPL06 传感器并读取校准数据
 * @param[in] ctx SPL06上下文结构体指针
 * @param[out] calib 校准数据结构体指针
 * @return 0 表示成功，负值表示错误
 */
int32_t SPL06_Init(SPL06_Ctx_t* ctx, SPL06_Calib_t* calib);

int32_t SPL06_GetID(SPL06_Ctx_t* ctx);
int32_t SPL06_Sleep(SPL06_Ctx_t* ctx);
int32_t SPL06_Wakeup(SPL06_Ctx_t* ctx);

int32_t SPL06_ReadRawTemp(SPL06_Ctx_t* ctx, int32_t* temp_raw);
int32_t SPL06_ReadRawPress(SPL06_Ctx_t* ctx, int32_t* press_raw);

#ifdef __cplusplus
}
#endif

#endif // SPL06_H