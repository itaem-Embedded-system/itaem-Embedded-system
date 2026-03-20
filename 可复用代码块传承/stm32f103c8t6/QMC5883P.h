/**
  ******************************************************************************
  * @file    qmc5883p_reg.h
  * @author  ST-Style Sensor Driver Architecture
  * @brief   This file contains all the functions prototypes for the
  *          qmc5883p_reg.c driver.
  ******************************************************************************
  */

#ifndef QMC5883P__H
#define QMC5883P__H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ========================================================================== */
/* 1. 跨平台运行上下文定义 (Context Definition)                               */
/* ========================================================================== */
// 无论你是STM32、ESP32还是Linux驱动，只需把平台的读写函数挂载到这个结构体上
typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t);

typedef struct {
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  stmdev_mdelay_ptr mdelay;
  void              *handle;
} stmdev_ctx_t;

#ifndef DRV_LITTLE_ENDIAN
#define DRV_LITTLE_ENDIAN 1234
#endif

#ifndef DRV_BIG_ENDIAN
#define DRV_BIG_ENDIAN 4321
#endif

#ifndef DRV_BYTE_ORDER
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#define DRV_BYTE_ORDER DRV_BIG_ENDIAN
#else
#define DRV_BYTE_ORDER DRV_LITTLE_ENDIAN
#endif
#endif

/* ========================================================================== */
/* 2. 器件信息与寄存器地址                                                    */
/* ========================================================================== */
#define QMC5883P_I2C_ADDRESS    0x2C
#define QMC5883P_CHIP_ID_VAL    0x80
#define QMC5883P_SET_RESET_INIT_VAL 0x06U
#define QMC5883P_I2C_TIMEOUT_MS 100U

#define QMC5883P_CHIP_ID        0x00U
#define QMC5883P_OUT_X_L        0x01U
#define QMC5883P_OUT_X_H        0x02U
#define QMC5883P_OUT_Y_L        0x03U
#define QMC5883P_OUT_Y_H        0x04U
#define QMC5883P_OUT_Z_L        0x05U
#define QMC5883P_OUT_Z_H        0x06U

/* ========================================================================== */
/* 3. 寄存器位域结构体定义 (Bitfield Structs)                                 */
/* ========================================================================== */

#define QMC5883P_STATUS         0x09U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy             : 1;  /* Data Ready */
  uint8_t ovfl             : 1;  /* Overflow */
  uint8_t not_used_01      : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01      : 6;
  uint8_t ovfl             : 1;  /* Overflow */
  uint8_t drdy             : 1;  /* Data Ready */
#endif
} qmc5883p_status_t;

#define QMC5883P_CTRL1          0x0AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t mode             : 2;  /* Mode Control */
  uint8_t odr              : 2;  /* Output Data Rate */
  uint8_t osr1             : 2;  /* Over sample Ratio 1 */
  uint8_t osr2             : 2;  /* Down sampling rate */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t osr2             : 2;  /* Down sampling rate */
  uint8_t osr1             : 2;  /* Over sample Ratio 1 */
  uint8_t odr              : 2;  /* Output Data Rate */
  uint8_t mode             : 2;  /* Mode Control */
#endif
} qmc5883p_ctrl1_t;

#define QMC5883P_CTRL2          0x0BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t set_reset_mode   : 2;
  uint8_t rng              : 2;  /* Full Scale Range */
  uint8_t not_used_01      : 2;
  uint8_t self_test        : 1;
  uint8_t soft_rst         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t soft_rst         : 1;
  uint8_t self_test        : 1;
  uint8_t not_used_01      : 2;
  uint8_t rng              : 2;  /* Full Scale Range */
  uint8_t set_reset_mode   : 2;
#endif
} qmc5883p_ctrl2_t;

#define QMC5883P_SET_RESET      0x29U

/* 将所有寄存器打包进共同体，符合MISRA-C规范时可废弃 */
typedef union {
  qmc5883p_status_t        status;
  qmc5883p_ctrl1_t         ctrl1;
  qmc5883p_ctrl2_t         ctrl2;
  uint8_t                  byte;
} qmc5883p_reg_t;

/* ========================================================================== */
/* 可读的高级枚举配置定义                                              */
/* ========================================================================== */
typedef enum {
  QMC5883P_SUSPEND_MODE    = 0x00,
  QMC5883P_NORMAL_MODE     = 0x01,
  QMC5883P_SINGLE_MODE     = 0x02,
  QMC5883P_CONT_MODE       = 0x03,
} qmc5883p_mode_t;

typedef enum {
  QMC5883P_ODR_10Hz        = 0x00,
  QMC5883P_ODR_50Hz        = 0x01,
  QMC5883P_ODR_100Hz       = 0x02,
  QMC5883P_ODR_200Hz       = 0x03,
} qmc5883p_odr_t;

typedef enum {
  QMC5883P_30_GAUSS        = 0x00,
  QMC5883P_12_GAUSS        = 0x01,
  QMC5883P_8_GAUSS         = 0x02,
  QMC5883P_2_GAUSS         = 0x03,
} qmc5883p_rng_t;

/* ========================================================================== */
/* 5. 驱动 API 函数声明                                                       */
/* ========================================================================== */

int32_t qmc5883p_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
int32_t qmc5883p_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, const uint8_t *data, uint16_t len);

/* 基础控制 */
/**
  * @brief  获取芯片 ID (Get Device ID)
  * @param  ctx   设备上下文句柄
  * @param  buff  返回的 ID 值 (应为 0x80)
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff);

/**
  * @brief  软复位 (Soft Reset)
  * @param  val   1:复位, 0:无效
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_reset_set(const stmdev_ctx_t *ctx, uint8_t val);

/**
  * @brief  执行标准初始化流程 (复位 + 设置魔数)
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_init_set(const stmdev_ctx_t *ctx);

/* 模式与参数设置 */
/**
  * @brief  设置工作模式 (Operating Mode)
  * @param  val   模式枚举 (Suspend/Normal/Single/Continuous)
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_operating_mode_set(const stmdev_ctx_t *ctx, qmc5883p_mode_t val);

/**
  * @brief  设置输出数据速率 (ODR)
  * @param  val   速率枚举 (10Hz/50Hz/100Hz/200Hz)
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_data_rate_set(const stmdev_ctx_t *ctx, qmc5883p_odr_t val);

/**
  * @brief  设置量程 (Full Scale Range)
  * @param  val   量程枚举 (2G/8G)
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_full_scale_set(const stmdev_ctx_t *ctx, qmc5883p_rng_t val);

/* 数据获取与状态 */
/**
  * @brief  获取数据就绪状态 (Data Ready)
  * @param  val   返回状态 (1:就绪, 0:未就绪)
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_data_ready_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
  * @brief  获取三轴原始磁场数据 (Raw Magnetic Data)
  * @param  val   返回的三轴数据数组 (X, Y, Z), 单位 LSB
  * @retval 0:成功, <0:失败
  */
int32_t qmc5883p_magnetic_raw_get(const stmdev_ctx_t *ctx, int16_t *val);

/* 数据敏感度计算 (将LSB转换为物理单位Gauss) */
/**
  * @brief  将原始 LSB 转换为高斯 (Gauss)
  * @param  lsb   原始数据
  * @retval 转换后的高斯值
  */
float qmc5883p_from_fs8_to_gauss(int16_t lsb);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883P_REGS_H */
