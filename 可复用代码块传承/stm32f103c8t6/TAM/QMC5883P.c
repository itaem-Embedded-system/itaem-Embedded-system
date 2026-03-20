/**
  ******************************************************************************
  * @file    qmc5883p_reg.c
  * @author  ST-Style Sensor Driver Architecture
  * @brief   QMC5883P driver file
  ******************************************************************************
  */

#include "QMC5883P.h"

#define QMC5883P_CTRL1_MODE_MASK 0x03U
#define QMC5883P_CTRL1_ODR_MASK  0x0CU
#define QMC5883P_CTRL2_RNG_MASK  0x0CU
#define QMC5883P_CTRL2_SRST_MASK 0x80U

static int32_t qmc5883p_ctx_is_valid(const stmdev_ctx_t *ctx) {
  if (ctx == NULL) return 0;
  if (ctx->read_reg == NULL) return 0;
  if (ctx->write_reg == NULL) return 0;
  return 1;
}

static int32_t qmc5883p_mode_is_valid(qmc5883p_mode_t val) {
  return ((uint8_t)val <= (uint8_t)QMC5883P_CONT_MODE) ? 1 : 0;
}

static int32_t qmc5883p_odr_is_valid(qmc5883p_odr_t val) {
  return ((uint8_t)val <= (uint8_t)QMC5883P_ODR_200Hz) ? 1 : 0;
}

static int32_t qmc5883p_rng_is_valid(qmc5883p_rng_t val) {
  return ((uint8_t)val <= (uint8_t)QMC5883P_2_GAUSS) ? 1 : 0;
}

/* 底层接口实现 */
int32_t qmc5883p_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
  if (!qmc5883p_ctx_is_valid(ctx) || (data == NULL) || (len == 0U)) return -1;
  return ctx->read_reg(ctx->handle, reg, data, len);
}

int32_t qmc5883p_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, const uint8_t *data, uint16_t len) {
  if (!qmc5883p_ctx_is_valid(ctx) || (data == NULL) || (len == 0U)) return -1;
  return ctx->write_reg(ctx->handle, reg, data, len);
}

/* ========================================================================= */
/*                                基础设定区                                  */
/* ========================================================================= */

int32_t qmc5883p_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff) {
  if (buff == NULL) return -1;
  return qmc5883p_read_reg(ctx, QMC5883P_CHIP_ID, buff, 1);
}

int32_t qmc5883p_reset_set(const stmdev_ctx_t *ctx, uint8_t val) {
  uint8_t ctrl2 = 0U;
  int32_t ret;

  if (val > 1U) return -1;
  ret = qmc5883p_read_reg(ctx, QMC5883P_CTRL2, &ctrl2, 1);
  if (ret == 0) {
    if (val == 1U) {
      ctrl2 |= QMC5883P_CTRL2_SRST_MASK;
    } else {
      ctrl2 &= (uint8_t)(~QMC5883P_CTRL2_SRST_MASK);
    }
    ret = qmc5883p_write_reg(ctx, QMC5883P_CTRL2, &ctrl2, 1);
  }
  return ret;
}

/**
 * @brief  初始化特定原厂环境配置 (依据手册 Application Examples)
 */
int32_t qmc5883p_init_set(const stmdev_ctx_t *ctx) {
  uint8_t set_reset_val = QMC5883P_SET_RESET_INIT_VAL;
  int32_t ret;

  // 软复位
  ret = qmc5883p_reset_set(ctx, 1);
  // 必须配置原厂魔法寄存器 0x29
  if (ret == 0) {
    ret = qmc5883p_write_reg(ctx, QMC5883P_SET_RESET, &set_reset_val, 1);
  }
  return ret;
}

/* ========================================================================= */
/*                          读-改-写 (RMW) 核心配置区                         */
/* ========================================================================= */

int32_t qmc5883p_operating_mode_set(const stmdev_ctx_t *ctx, qmc5883p_mode_t val) {
  uint8_t ctrl1 = 0U;
  int32_t ret;

  if (qmc5883p_mode_is_valid(val) == 0) return -1;
  ret = qmc5883p_read_reg(ctx, QMC5883P_CTRL1, &ctrl1, 1);
  if (ret == 0) {
    ctrl1 = (uint8_t)((ctrl1 & (uint8_t)(~QMC5883P_CTRL1_MODE_MASK)) | ((uint8_t)val & QMC5883P_CTRL1_MODE_MASK));
    ret = qmc5883p_write_reg(ctx, QMC5883P_CTRL1, &ctrl1, 1);
  }
  return ret;
}

int32_t qmc5883p_data_rate_set(const stmdev_ctx_t *ctx, qmc5883p_odr_t val) {
  uint8_t ctrl1 = 0U;
  int32_t ret;

  if (qmc5883p_odr_is_valid(val) == 0) return -1;
  ret = qmc5883p_read_reg(ctx, QMC5883P_CTRL1, &ctrl1, 1);
  if (ret == 0) {
    ctrl1 = (uint8_t)((ctrl1 & (uint8_t)(~QMC5883P_CTRL1_ODR_MASK)) | ((((uint8_t)val << 2) & QMC5883P_CTRL1_ODR_MASK)));
    ret = qmc5883p_write_reg(ctx, QMC5883P_CTRL1, &ctrl1, 1);
  }
  return ret;
}

int32_t qmc5883p_full_scale_set(const stmdev_ctx_t *ctx, qmc5883p_rng_t val) {
  uint8_t ctrl2 = 0U;
  int32_t ret;

  if (qmc5883p_rng_is_valid(val) == 0) return -1;
  ret = qmc5883p_read_reg(ctx, QMC5883P_CTRL2, &ctrl2, 1);
  if (ret == 0) {
    ctrl2 &= (uint8_t)(~QMC5883P_CTRL2_SRST_MASK);
    ctrl2 = (uint8_t)((ctrl2 & (uint8_t)(~QMC5883P_CTRL2_RNG_MASK)) | ((((uint8_t)val << 2) & QMC5883P_CTRL2_RNG_MASK)));
    ret = qmc5883p_write_reg(ctx, QMC5883P_CTRL2, &ctrl2, 1);
  }
  return ret;
}

/* ========================================================================= */
/*                              数据获取区                                    */
/* ========================================================================= */

int32_t qmc5883p_data_ready_get(const stmdev_ctx_t *ctx, uint8_t *val) {
  qmc5883p_status_t status;
  int32_t ret;

  if (val == NULL) return -1;
  ret = qmc5883p_read_reg(ctx, QMC5883P_STATUS, (uint8_t *)&status, 1);
  if (ret == 0) {
    *val = status.drdy;
  }
  return ret;
}

int32_t qmc5883p_magnetic_raw_get(const stmdev_ctx_t *ctx, int16_t *val) {
  uint8_t buff[6];
  int32_t ret;

  if (val == NULL) return -1;
  ret = qmc5883p_read_reg(ctx, QMC5883P_OUT_X_L, buff, 6);
  if (ret == 0) {
    // 强制转换为带符号的16位整型 (小端拼接)
    val[0] = (int16_t)((buff[1] << 8) | buff[0]);
    val[1] = (int16_t)((buff[3] << 8) | buff[2]);
    val[2] = (int16_t)((buff[5] << 8) | buff[4]);
  }
  return ret;
}

/* ========================================================================= */
/*                              敏感度转化区                                  */
/* ========================================================================= */

/**
 * @brief 将原始ADC转为物理单位 (以 ±8 Gauss 为例 = 3750 LSB/G)
 */
float qmc5883p_from_fs8_to_gauss(int16_t lsb) {
  return ((float)lsb) / 3750.0f;
}
