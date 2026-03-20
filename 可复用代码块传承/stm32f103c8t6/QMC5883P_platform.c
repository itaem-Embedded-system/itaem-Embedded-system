#include "QMC5883P_platfrom.h"
#include "i2c.h"

#define QMC5883P_I2C_RETRY_MAX 3U
#define QMC5883P_I2C_RETRY_DELAY_MS 2U

static int32_t qmc_platform_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
  I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)handle;
  uint8_t retry = 0U;

  if ((i2c == NULL) || (data == NULL) || (len == 0U)) return -1;
  for (retry = 0U; retry < QMC5883P_I2C_RETRY_MAX; retry++) {
    if (HAL_I2C_Mem_Write(i2c, (uint16_t)(QMC5883P_I2C_ADDRESS << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, QMC5883P_I2C_TIMEOUT_MS) == HAL_OK) {
      return 0;
    }
    HAL_Delay(QMC5883P_I2C_RETRY_DELAY_MS);
  }
  return -1;
}

static int32_t qmc_platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)handle;
  uint8_t retry = 0U;

  if ((i2c == NULL) || (data == NULL) || (len == 0U)) return -1;
  for (retry = 0U; retry < QMC5883P_I2C_RETRY_MAX; retry++) {
    if (HAL_I2C_Mem_Read(i2c, (uint16_t)(QMC5883P_I2C_ADDRESS << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, QMC5883P_I2C_TIMEOUT_MS) == HAL_OK) {
      return 0;
    }
    HAL_Delay(QMC5883P_I2C_RETRY_DELAY_MS);
  }
  return -1;
}

static void qmc_platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

void qmc5883p_platfrom_init_ctx(stmdev_ctx_t *ctx, I2C_HandleTypeDef *i2c_handle)
{
  if ((ctx == NULL) || (i2c_handle == NULL)) return;
  ctx->write_reg = qmc_platform_write;
  ctx->read_reg = qmc_platform_read;
  ctx->mdelay = qmc_platform_delay;
  ctx->handle = i2c_handle;
}

int32_t qmc5883p_platfrom_init_device(const stmdev_ctx_t *ctx)
{
  uint8_t chip_id = 0U;
  uint8_t ctrl1 = 0xFFU;
  uint8_t ctrl2 = 0xFFU;

  if (qmc5883p_device_id_get(ctx, &chip_id) != 0) return -1;
  if (chip_id != QMC5883P_CHIP_ID_VAL) return -1;
  if (qmc5883p_init_set(ctx) != 0) return -1;
  if (qmc5883p_full_scale_set(ctx, QMC5883P_8_GAUSS) != 0) return -1;
  if (qmc5883p_data_rate_set(ctx, QMC5883P_ODR_50Hz) != 0) return -1;
  if (qmc5883p_operating_mode_set(ctx, QMC5883P_CONT_MODE) != 0) return -1;

  // Read back key control registers to avoid silent misconfiguration.
  if (qmc5883p_read_reg(ctx, QMC5883P_CTRL1, &ctrl1, 1U) != 0) return -1;
  if (qmc5883p_read_reg(ctx, QMC5883P_CTRL2, &ctrl2, 1U) != 0) return -1;

  if ((ctrl1 & 0x03U) != (uint8_t)QMC5883P_CONT_MODE) return -1;
  if ((ctrl1 & 0x0CU) != ((uint8_t)QMC5883P_ODR_50Hz << 2)) return -1;
  if ((ctrl2 & 0x0CU) != ((uint8_t)QMC5883P_8_GAUSS << 2)) return -1;
  if ((ctrl2 & 0x80U) != 0U) return -1;

  return 0;
}

int32_t qmc5883p_platfrom_poll_raw(const stmdev_ctx_t *ctx, int16_t mag[3], uint8_t *has_new_data)
{
  uint8_t drdy = 0U;

  if ((mag == NULL) || (has_new_data == NULL)) return -1;
  *has_new_data = 0U;
  if (qmc5883p_data_ready_get(ctx, &drdy) != 0) return -1;

  if (drdy != 0U) {
    if (qmc5883p_magnetic_raw_get(ctx, mag) != 0) return -1;
    *has_new_data = 1U;
    return 0;
  }

  /* Some modules report DRDY unreliably; do one fallback raw read. */
  if (qmc5883p_magnetic_raw_get(ctx, mag) == 0) {
    *has_new_data = 1U;
  }
  return 0;
}

void qmc5883p_platfrom_delay_ms(const stmdev_ctx_t *ctx, uint32_t ms)
{
  if (ctx != NULL && ctx->mdelay != NULL) {
    ctx->mdelay(ms);
    return;
  }
  HAL_Delay(ms);
}
