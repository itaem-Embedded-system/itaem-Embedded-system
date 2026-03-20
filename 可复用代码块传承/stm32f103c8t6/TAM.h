#ifndef __TAM_H__
#define __TAM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  uint8_t sensor_ready;
  uint8_t i2c_online;
  uint8_t chip_id;
  uint8_t reg_status;
  uint8_t reg_ctrl1;
  uint8_t reg_ctrl2;
  int16_t mag[3];
  uint8_t mag_valid;
  uint8_t poll_error_count;
  uint8_t bad_sample_count;
  uint8_t reinit_active;
  uint8_t reinit_try_count;
} TAM_DiagSnapshot_t;

/**
  * @brief  应用层初始化 (硬件绑定 + 首次上电初始化)
  * @note   应在 main() 初始化阶段调用
  */
void TAM_Init(void);

/**
  * @brief  应用层业务循环 (数据采集 + 故障恢复)
  * @note   应在 while(1) 中调用，内置 100ms 非阻塞节拍
  */
void TAM_Loop(void);

/**
  * @brief  获取最新磁场数据（单次消费）
  * @param  mag           返回三轴数据 int16_t[3]
  * @param  has_new_data  1:有新样本，0:无
  */
void TAM_GetLatestMag(int16_t mag[3], uint8_t *has_new_data);

/**
  * @brief  获取传感器初始化状态
  * @retval 1:已初始化, 0:未初始化
  */
uint8_t TAM_IsSensorReady(void);

/**
  * @brief  获取当前传感器诊断快照（初始化状态/寄存器/原始数据）
  * @param  snapshot  输出快照结构体
  */
void TAM_GetDiagSnapshot(TAM_DiagSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
