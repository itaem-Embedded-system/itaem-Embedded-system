#include "TAM.h"
#include "QMC5883P_platfrom.h"
#include "i2c.h"

/* 采样节拍参数 */
#define TAM_POLL_PERIOD_MS 100U

/* 重启策略参数 */
#define TAM_REINIT_INTERVAL_MS 1000U
#define TAM_REINIT_RETRY_MAX 3U
#define TAM_REINIT_RETRY_DELAY_MS 10U

/* 异常判定阈值 */
#define TAM_POLL_ERROR_RESTART_THRESHOLD 10U
#define TAM_BAD_SAMPLE_RESTART_THRESHOLD 8U
#define TAM_SAMPLE_ABS_LIMIT 32000

static stmdev_ctx_t tam_qmc_ctx;

/* 运行状态量 */
static uint8_t tam_qmc_initialized = 0U;
static uint8_t tam_poll_armed = 0U;
static uint8_t tam_reinit_active = 0U;

/* 时间戳状态量 */
static uint32_t tam_last_poll_tick = 0U;
static uint32_t tam_last_reinit_tick = 0U;
static uint32_t tam_last_reinit_try_tick = 0U;

/* 计数量 */
static uint8_t tam_reinit_try_count = 0U;
static uint8_t tam_poll_error_count = 0U;
static uint8_t tam_bad_sample_count = 0U;

/* 对外数据快照 */
static int16_t tam_latest_mag[3] = {0};
static uint8_t tam_latest_mag_ready = 0U;

/**
 * @brief  开始一次非阻塞重试窗口
 */
static void tam_qmc_reinit_start(uint32_t now_tick)
{
  tam_reinit_active = 1U;
  tam_reinit_try_count = 0U;
  tam_last_reinit_try_tick = now_tick - TAM_REINIT_RETRY_DELAY_MS;
}

/**
 * @brief  执行一次非阻塞初始化尝试
 * @note   在重试窗口内每隔TAM_REINIT_RETRY_DELAY_MS尝试一次，最多TAM_REINIT_RETRY_MAX次
 * @return 成功返回1U，其他情况返回0U
 */
static uint8_t tam_qmc_reinit_step(uint32_t now_tick)
{
  if (tam_reinit_active == 0U) return 0U;
  if ((uint32_t)(now_tick - tam_last_reinit_try_tick) < TAM_REINIT_RETRY_DELAY_MS) return 0U;

  tam_last_reinit_try_tick = now_tick;
  if (qmc5883p_platfrom_init_device(&tam_qmc_ctx) == 0) {
    tam_reinit_active = 0U;
    tam_reinit_try_count = 0U;
    return 1U;
  }

  tam_reinit_try_count++;
  if (tam_reinit_try_count >= TAM_REINIT_RETRY_MAX) {
    tam_reinit_active = 0U;
    tam_reinit_try_count = 0U;
    tam_last_reinit_tick = now_tick;
  }

  return 0U;
}

static uint8_t tam_qmc_sample_is_bad(const int16_t mag[3])
{
  int32_t ax = (int32_t)mag[0];
  int32_t ay = (int32_t)mag[1];
  int32_t az = (int32_t)mag[2];
  if ((ax == 0) && (ay == 0) && (az == 0)) {
    return 1U;
  }
  if (((ax >= TAM_SAMPLE_ABS_LIMIT) || (ax <= -TAM_SAMPLE_ABS_LIMIT)) &&
      ((ay >= TAM_SAMPLE_ABS_LIMIT) || (ay <= -TAM_SAMPLE_ABS_LIMIT)) &&
      ((az >= TAM_SAMPLE_ABS_LIMIT) || (az <= -TAM_SAMPLE_ABS_LIMIT))) {
    return 1U;
  }
  return 0U;
}

/**
 * @brief  应用层初始化 (硬件绑定 + 首次上电初始化)
 * @note   应在 main() 初始化阶段调用
 */
void TAM_Init(void)
{
  uint32_t now_tick;
  now_tick = HAL_GetTick();
  
  qmc5883p_platfrom_init_ctx(&tam_qmc_ctx, &hi2c1);

  tam_poll_error_count = 0U;
  tam_bad_sample_count = 0U;
  tam_reinit_active = 0U;
  tam_reinit_try_count = 0U;
  tam_last_reinit_tick = now_tick;
  tam_last_reinit_try_tick = now_tick;
  tam_latest_mag_ready = 0U;

  if (qmc5883p_platfrom_init_device(&tam_qmc_ctx) == 0) {
    tam_qmc_initialized = 1U;
    tam_last_reinit_tick = now_tick;
  } else {
    tam_qmc_initialized = 0U;
    tam_qmc_reinit_start(now_tick);
  }
}

/**
 * @brief  应用层业务循环 (数据采集 + 故障恢复)
 * @note   应在 while(1) 中调用，内置 100ms 非阻塞节拍
 */
void TAM_Loop(void)
{
  uint32_t now_tick = HAL_GetTick();
  if (tam_poll_armed == 0U) {
    tam_last_poll_tick = now_tick - TAM_POLL_PERIOD_MS;
    tam_poll_armed = 1U;
  }
  if ((uint32_t)(now_tick - tam_last_poll_tick) < TAM_POLL_PERIOD_MS) return;
  tam_last_poll_tick = now_tick;

  if (tam_qmc_initialized == 0U) {
    if ((tam_reinit_active == 0U) && ((uint32_t)(now_tick - tam_last_reinit_tick) >= TAM_REINIT_INTERVAL_MS)) {
      tam_qmc_reinit_start(now_tick);
    }
    if (tam_qmc_reinit_step(now_tick) == 1U) {
      tam_qmc_initialized = 1U;
      tam_poll_error_count = 0U;
      tam_bad_sample_count = 0U;
      tam_last_reinit_tick = now_tick;
    }
    return;
  }

  int16_t mag[3] = {0};
  uint8_t has_new_data = 0U;
  if (qmc5883p_platfrom_poll_raw(&tam_qmc_ctx, mag, &has_new_data) != 0) {
    if (tam_poll_error_count < 255U) tam_poll_error_count++;
    if (tam_poll_error_count >= TAM_POLL_ERROR_RESTART_THRESHOLD) {
      tam_qmc_initialized = 0U;
      tam_poll_error_count = 0U;
      tam_bad_sample_count = 0U;
      tam_qmc_reinit_start(now_tick);
    }
    return;
  }

  tam_poll_error_count = 0U;
  if (has_new_data == 0U) return;

  if (tam_qmc_sample_is_bad(mag) == 1U) {
    if (tam_bad_sample_count < 255U) tam_bad_sample_count++;
    if (tam_bad_sample_count >= TAM_BAD_SAMPLE_RESTART_THRESHOLD) {
      tam_qmc_initialized = 0U;
      tam_bad_sample_count = 0U;
      tam_qmc_reinit_start(now_tick);
    }
  } else {
    tam_bad_sample_count = 0U;
    tam_latest_mag[0] = mag[0];
    tam_latest_mag[1] = mag[1];
    tam_latest_mag[2] = mag[2];
    tam_latest_mag_ready = 1U;
  }
}

void TAM_GetLatestMag(int16_t mag[3], uint8_t *has_new_data)
{
  if ((mag == NULL) || (has_new_data == NULL)) return;

  mag[0] = tam_latest_mag[0];
  mag[1] = tam_latest_mag[1];
  mag[2] = tam_latest_mag[2];
  *has_new_data = tam_latest_mag_ready;
  tam_latest_mag_ready = 0U;
}

uint8_t TAM_IsSensorReady(void)
{
  return tam_qmc_initialized;
}

void TAM_GetDiagSnapshot(TAM_DiagSnapshot_t *snapshot)
{
  uint8_t regs[3] = {0xFFU, 0xFFU, 0xFFU};
  int16_t mag[3] = {0};

  if (snapshot == NULL) return;

  snapshot->sensor_ready = tam_qmc_initialized;
  snapshot->i2c_online = 0U;
  snapshot->chip_id = 0xFFU;
  snapshot->reg_status = 0xFFU;
  snapshot->reg_ctrl1 = 0xFFU;
  snapshot->reg_ctrl2 = 0xFFU;
  snapshot->mag[0] = tam_latest_mag[0];
  snapshot->mag[1] = tam_latest_mag[1];
  snapshot->mag[2] = tam_latest_mag[2];
  snapshot->mag_valid = tam_latest_mag_ready;
  snapshot->poll_error_count = tam_poll_error_count;
  snapshot->bad_sample_count = tam_bad_sample_count;
  snapshot->reinit_active = tam_reinit_active;
  snapshot->reinit_try_count = tam_reinit_try_count;

  if (qmc5883p_device_id_get(&tam_qmc_ctx, &snapshot->chip_id) != 0) {
    return;
  }

  if (qmc5883p_read_reg(&tam_qmc_ctx, QMC5883P_STATUS, &regs[0], 1U) != 0) return;
  if (qmc5883p_read_reg(&tam_qmc_ctx, QMC5883P_CTRL1, &regs[1], 1U) != 0) return;
  if (qmc5883p_read_reg(&tam_qmc_ctx, QMC5883P_CTRL2, &regs[2], 1U) != 0) return;

  snapshot->reg_status = regs[0];
  snapshot->reg_ctrl1 = regs[1];
  snapshot->reg_ctrl2 = regs[2];
  snapshot->i2c_online = 1U;

  if (qmc5883p_magnetic_raw_get(&tam_qmc_ctx, mag) == 0) {
    snapshot->mag[0] = mag[0];
    snapshot->mag[1] = mag[1];
    snapshot->mag[2] = mag[2];
    snapshot->mag_valid = 1U;
  }
}
