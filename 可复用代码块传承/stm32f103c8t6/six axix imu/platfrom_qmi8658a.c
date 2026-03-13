#include "platfrom_qmi8658a.h"
#include "i2c.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

/*
 * @brief  STM32 I2C 写函数适配 (Platform Layer)
 * @param  handle: 设备地址句柄 (通常强转为 uint16_t 设备地址)
 * @param  reg:    寄存器地址
 * @param  buf:    写入数据缓冲区
 * @param  len:    写入长度
 * @retval 0: 成功, -1: 失败
 */
int32_t stm32_i2c_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
    uint16_t dev_addr = (uint16_t)(uint32_t)handle;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg,
                                                 I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, len, 100);
    return (status == HAL_OK) ? QMI_OK : QMI_ERROR;
}

/*
 * @brief  STM32 I2C 读函数适配 (Platform Layer)
 * @param  handle: 设备地址句柄
 * @param  reg:    寄存器地址
 * @param  buf:    读取数据缓冲区
 * @param  len:    读取长度
 * @retval 0: 成功, -1: 失败
 */
int32_t stm32_i2c_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
    uint16_t dev_addr = (uint16_t)(uint32_t)handle;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg,
                                                I2C_MEMADD_SIZE_8BIT, buf, len, 100);
    return (status == HAL_OK) ? QMI_OK : QMI_ERROR;
}

/*
 * @brief  平台级毫秒延时适配
 */
void stm32_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

/*
 * @brief  获取系统 Tick (用于时间戳和超时判断)
 */
uint32_t platform_get_tick(void) {
    return HAL_GetTick();
}

stmdev_ctx_t qmi8658_ctx = {
    .write_reg = stm32_i2c_write,
    .read_reg  = stm32_i2c_read,
    .mdelay    = stm32_delay_ms,
    .handle    = (void*)0x6B,
};

