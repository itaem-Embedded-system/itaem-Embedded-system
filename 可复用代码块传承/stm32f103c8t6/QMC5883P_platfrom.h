#ifndef __QMC5883P_PLATFROM_H__
#define __QMC5883P_PLATFROM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "QMC5883P.h"

/**
  * @brief  绑定 I2C 句柄到驱动上下文
  * @param  ctx         驱动上下文
  * @param  i2c_handle  STM32 HAL I2C 句柄 (e.g. &hi2c1)
  */
void qmc5883p_platfrom_init_ctx(stmdev_ctx_t *ctx, I2C_HandleTypeDef *i2c_handle);

/**
  * @brief  平台级初始化设备 (复位 + 校验 ID + 默认配置)
  * @retval 0:成功, -1:失败
  */
int32_t qmc5883p_platfrom_init_device(const stmdev_ctx_t *ctx);

/**
  * @brief  轮询获取数据 (Check DRDY -> Read Data)
  * @param  mag           返回的三轴数据 (int16_t[3])
  * @param  has_new_data  返回是否有新数据 (1:有, 0:无)
  * @retval 0:成功, -1:I2C错误
  */
int32_t qmc5883p_platfrom_poll_raw(const stmdev_ctx_t *ctx, int16_t mag[3], uint8_t *has_new_data);

/**
  * @brief  毫秒级延时 (非阻塞或阻塞取决于具体实现，当前为阻塞)
  */
void qmc5883p_platfrom_delay_ms(const stmdev_ctx_t *ctx, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
