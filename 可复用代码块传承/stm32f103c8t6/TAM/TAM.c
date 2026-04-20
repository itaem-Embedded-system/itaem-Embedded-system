#include "TAM.h"
#include "QMC5883P_platfrom.h"
#include "i2c.h"
#include <stddef.h>
#include <string.h>

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

/* 校准参数 */
#define TAM_CALIB_MAGIC 0x54414D43UL
#define TAM_CALIB_VERSION 1U
#define TAM_CALIB_FLASH_PAGE_ADDR 0x0800FC00UL
#define TAM_CALIB_MIN_SAMPLE_COUNT 60U
#define TAM_CALIB_MIN_AXIS_SPAN 600

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t length;
  uint32_t sample_count;
  float offset[3];
  float scale[3];
  uint32_t checksum;
} TAM_CalibrationStore_t;

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
static int16_t tam_latest_mag_raw[3] = {0};
static int16_t tam_latest_mag[3] = {0};
static uint8_t tam_latest_mag_ready = 0U;
static uint8_t tam_latest_mag_valid = 0U;

/* 校准状态量 */
static uint8_t tam_calib_valid = 0U;
static uint8_t tam_calib_active = 0U;
static uint8_t tam_calib_dirty = 0U;
static uint32_t tam_calib_sample_count = 0U;
static int16_t tam_calib_min[3] = {0};
static int16_t tam_calib_max[3] = {0};
static float tam_calib_offset[3] = {0.0f, 0.0f, 0.0f};
static float tam_calib_scale[3] = {1.0f, 1.0f, 1.0f};

static uint32_t tam_calib_checksum(const uint8_t *data, uint32_t len)
{
  uint32_t checksum = 2166136261UL;
  uint32_t i;

  for (i = 0U; i < len; i++) {
    checksum ^= data[i];
    checksum *= 16777619UL;
  }
  return checksum;
}

static void tam_calib_reset_runtime(void)
{
  tam_calib_valid = 0U;
  tam_calib_offset[0] = 0.0f;
  tam_calib_offset[1] = 0.0f;
  tam_calib_offset[2] = 0.0f;
  tam_calib_scale[0] = 1.0f;
  tam_calib_scale[1] = 1.0f;
  tam_calib_scale[2] = 1.0f;
}

static uint8_t tam_calib_store_is_sane(const TAM_CalibrationStore_t *store)
{
  uint32_t expected_checksum;
  uint32_t payload_len;
  uint8_t axis;

  if (store == NULL) return 0U;
  if (store->magic != TAM_CALIB_MAGIC) return 0U;
  if (store->version != TAM_CALIB_VERSION) return 0U;
  if (store->length != (uint16_t)sizeof(TAM_CalibrationStore_t)) return 0U;
  if (store->sample_count < TAM_CALIB_MIN_SAMPLE_COUNT) return 0U;

  payload_len = (uint32_t)offsetof(TAM_CalibrationStore_t, checksum);
  expected_checksum = tam_calib_checksum((const uint8_t *)store, payload_len);
  if (expected_checksum != store->checksum) return 0U;

  for (axis = 0U; axis < 3U; axis++) {
    if ((store->offset[axis] > 32768.0f) || (store->offset[axis] < -32768.0f)) return 0U;
    if ((store->scale[axis] < 0.1f) || (store->scale[axis] > 8.0f)) return 0U;
  }

  return 1U;
}

static void tam_calib_apply_store(const TAM_CalibrationStore_t *store)
{
  uint8_t axis;

  if (store == NULL) return;
  for (axis = 0U; axis < 3U; axis++) {
    tam_calib_offset[axis] = store->offset[axis];
    tam_calib_scale[axis] = store->scale[axis];
  }
  tam_calib_valid = 1U;
}

static void tam_calib_load_from_flash(void)
{
  const TAM_CalibrationStore_t *store = (const TAM_CalibrationStore_t *)TAM_CALIB_FLASH_PAGE_ADDR;

  tam_calib_reset_runtime();
  if (tam_calib_store_is_sane(store) == 0U) return;
  tam_calib_apply_store(store);
}

static HAL_StatusTypeDef tam_calib_flash_erase_page(void)
{
  FLASH_EraseInitTypeDef erase_init = {0};
  uint32_t page_error = 0U;
  HAL_StatusTypeDef status;

  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = TAM_CALIB_FLASH_PAGE_ADDR;
  erase_init.NbPages = 1U;

  status = HAL_FLASHEx_Erase(&erase_init, &page_error);
  if (status != HAL_OK) return status;
  if (page_error != 0xFFFFFFFFUL) return HAL_ERROR;
  return HAL_OK;
}

static HAL_StatusTypeDef tam_calib_flash_write_store(const TAM_CalibrationStore_t *store)
{
  const uint8_t *bytes = (const uint8_t *)store;
  uint32_t addr = TAM_CALIB_FLASH_PAGE_ADDR;
  uint32_t i;

  if (store == NULL) return HAL_ERROR;

  for (i = 0U; i < (uint32_t)sizeof(TAM_CalibrationStore_t); i += 2U) {
    uint16_t halfword = bytes[i];
    if ((i + 1U) < (uint32_t)sizeof(TAM_CalibrationStore_t)) {
      halfword |= (uint16_t)((uint16_t)bytes[i + 1U] << 8);
    } else {
      halfword |= 0xFF00U;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, halfword) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  if (memcmp((const void *)TAM_CALIB_FLASH_PAGE_ADDR,
             store,
             sizeof(TAM_CalibrationStore_t)) != 0) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef tam_calib_store_to_flash(const TAM_CalibrationStore_t *store)
{
  HAL_StatusTypeDef status;

  if (HAL_FLASH_Unlock() != HAL_OK) return HAL_ERROR;

  status = tam_calib_flash_erase_page();
  if (status == HAL_OK) {
    status = tam_calib_flash_write_store(store);
  }

  (void)HAL_FLASH_Lock();
  return status;
}

static void tam_calib_collect_reset(void)
{
  tam_calib_active = 1U;
  tam_calib_dirty = 0U;
  tam_calib_sample_count = 0U;
  tam_calib_min[0] = 32767;
  tam_calib_min[1] = 32767;
  tam_calib_min[2] = 32767;
  tam_calib_max[0] = -32768;
  tam_calib_max[1] = -32768;
  tam_calib_max[2] = -32768;
}

static void tam_calib_collect_sample(const int16_t raw_mag[3])
{
  uint8_t axis;

  if ((tam_calib_active == 0U) || (raw_mag == NULL)) return;

  for (axis = 0U; axis < 3U; axis++) {
    if (raw_mag[axis] < tam_calib_min[axis]) tam_calib_min[axis] = raw_mag[axis];
    if (raw_mag[axis] > tam_calib_max[axis]) tam_calib_max[axis] = raw_mag[axis];
  }
  tam_calib_sample_count++;
  tam_calib_dirty = 1U;
}

static uint8_t tam_calib_has_coverage(void)
{
  uint8_t axis;

  if (tam_calib_sample_count < TAM_CALIB_MIN_SAMPLE_COUNT) return 0U;
  for (axis = 0U; axis < 3U; axis++) {
    if (((int32_t)tam_calib_max[axis] - (int32_t)tam_calib_min[axis]) < TAM_CALIB_MIN_AXIS_SPAN) {
      return 0U;
    }
  }
  return 1U;
}

static void tam_calib_build_params(float offset[3], float scale[3])
{
  float radius[3];
  float radius_avg;
  uint8_t axis;

  for (axis = 0U; axis < 3U; axis++) {
    offset[axis] = ((float)tam_calib_max[axis] + (float)tam_calib_min[axis]) * 0.5f;
    radius[axis] = ((float)tam_calib_max[axis] - (float)tam_calib_min[axis]) * 0.5f;
  }

  radius_avg = (radius[0] + radius[1] + radius[2]) / 3.0f;
  for (axis = 0U; axis < 3U; axis++) {
    if (radius[axis] > 1.0f) {
      scale[axis] = radius_avg / radius[axis];
    } else {
      scale[axis] = 1.0f;
    }
  }
}

static int16_t tam_saturate_to_i16(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  if (value >= 0.0f) return (int16_t)(value + 0.5f);
  return (int16_t)(value - 0.5f);
}

static void tam_apply_calibration(const int16_t raw_mag[3], int16_t corrected_mag[3])
{
  uint8_t axis;

  if ((raw_mag == NULL) || (corrected_mag == NULL)) return;

  for (axis = 0U; axis < 3U; axis++) {
    float corrected = ((float)raw_mag[axis] - tam_calib_offset[axis]) * tam_calib_scale[axis];
    corrected_mag[axis] = tam_saturate_to_i16(corrected);
  }
}

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
  tam_calib_load_from_flash();

  tam_poll_error_count = 0U;
  tam_bad_sample_count = 0U;
  tam_reinit_active = 0U;
  tam_reinit_try_count = 0U;
  tam_last_reinit_tick = now_tick;
  tam_last_reinit_try_tick = now_tick;
  tam_latest_mag_ready = 0U;
  tam_latest_mag_valid = 0U;
  tam_calib_active = 0U;
  tam_calib_dirty = 0U;
  tam_calib_sample_count = 0U;

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

  {
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
      return;
    }

    tam_bad_sample_count = 0U;
    tam_latest_mag_raw[0] = mag[0];
    tam_latest_mag_raw[1] = mag[1];
    tam_latest_mag_raw[2] = mag[2];
    tam_apply_calibration(mag, tam_latest_mag);
    tam_latest_mag_ready = 1U;
    tam_latest_mag_valid = 1U;
    tam_calib_collect_sample(mag);
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
  uint8_t axis;

  if (snapshot == NULL) return;

  snapshot->sensor_ready = tam_qmc_initialized;
  snapshot->i2c_online = 0U;
  snapshot->chip_id = 0xFFU;
  snapshot->reg_status = 0xFFU;
  snapshot->reg_ctrl1 = 0xFFU;
  snapshot->reg_ctrl2 = 0xFFU;
  snapshot->raw_mag[0] = tam_latest_mag_raw[0];
  snapshot->raw_mag[1] = tam_latest_mag_raw[1];
  snapshot->raw_mag[2] = tam_latest_mag_raw[2];
  snapshot->mag[0] = tam_latest_mag[0];
  snapshot->mag[1] = tam_latest_mag[1];
  snapshot->mag[2] = tam_latest_mag[2];
  snapshot->mag_valid = tam_latest_mag_valid;
  snapshot->poll_error_count = tam_poll_error_count;
  snapshot->bad_sample_count = tam_bad_sample_count;
  snapshot->reinit_active = tam_reinit_active;
  snapshot->reinit_try_count = tam_reinit_try_count;
  snapshot->calib_valid = tam_calib_valid;
  snapshot->calib_active = tam_calib_active;
  snapshot->calib_dirty = tam_calib_dirty;
  snapshot->calib_reserved = 0U;
  snapshot->calib_sample_count = tam_calib_sample_count;
  for (axis = 0U; axis < 3U; axis++) {
    snapshot->calib_offset[axis] = tam_calib_offset[axis];
    snapshot->calib_scale[axis] = tam_calib_scale[axis];
  }

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
    snapshot->raw_mag[0] = mag[0];
    snapshot->raw_mag[1] = mag[1];
    snapshot->raw_mag[2] = mag[2];
    tam_apply_calibration(mag, snapshot->mag);
    snapshot->mag_valid = 1U;
  }
}

void TAM_CalibrationStart(void)
{
  tam_calib_collect_reset();
}

void TAM_CalibrationCancel(void)
{
  tam_calib_active = 0U;
  tam_calib_dirty = 0U;
  tam_calib_sample_count = 0U;
}

TAM_CalibResult_t TAM_CalibrationSave(void)
{
  TAM_CalibrationStore_t store;
  float offset[3] = {0.0f, 0.0f, 0.0f};
  float scale[3] = {1.0f, 1.0f, 1.0f};
  uint32_t payload_len;

  if (tam_calib_active == 0U) return TAM_CALIB_ERR_STATE;
  if (tam_calib_has_coverage() == 0U) return TAM_CALIB_ERR_COVERAGE;

  tam_calib_build_params(offset, scale);

  memset(&store, 0, sizeof(store));
  store.magic = TAM_CALIB_MAGIC;
  store.version = TAM_CALIB_VERSION;
  store.length = (uint16_t)sizeof(TAM_CalibrationStore_t);
  store.sample_count = tam_calib_sample_count;
  memcpy(store.offset, offset, sizeof(offset));
  memcpy(store.scale, scale, sizeof(scale));
  payload_len = (uint32_t)offsetof(TAM_CalibrationStore_t, checksum);
  store.checksum = tam_calib_checksum((const uint8_t *)&store, payload_len);

  if (tam_calib_store_to_flash(&store) != HAL_OK) {
    return TAM_CALIB_ERR_FLASH;
  }

  tam_calib_apply_store(&store);
  tam_calib_active = 0U;
  tam_calib_dirty = 0U;
  if (tam_latest_mag_valid != 0U) {
    tam_apply_calibration(tam_latest_mag_raw, tam_latest_mag);
  }
  return TAM_CALIB_OK;
}

TAM_CalibResult_t TAM_CalibrationClear(void)
{
  HAL_StatusTypeDef status;

  if (HAL_FLASH_Unlock() != HAL_OK) return TAM_CALIB_ERR_FLASH;
  status = tam_calib_flash_erase_page();
  (void)HAL_FLASH_Lock();
  if (status != HAL_OK) return TAM_CALIB_ERR_FLASH;

  tam_calib_active = 0U;
  tam_calib_dirty = 0U;
  tam_calib_sample_count = 0U;
  tam_calib_reset_runtime();
  if (tam_latest_mag_valid != 0U) {
    tam_apply_calibration(tam_latest_mag_raw, tam_latest_mag);
  }
  return TAM_CALIB_OK;
}
