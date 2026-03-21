#include "baro.h"
#include "spl06.h"
#include "stm32ofspl06.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// 物理计算常量
#define KT 524288.0f
#define KP 1572864.0f

// 错误计数阈值
#define MAX_ERROR_THRESHOLD  5
#define SENSOR_UPDATE_PERIOD_MS 100  // 100ms更新一次
#define SENSOR_INIT_RETRY_PERIOD_MS 1000  // 初始化失败后每1s重试
#define SENSOR_WARMUP_SAMPLES 8      // 初始化后先收集稳定样本，再对外发布
#define SENSOR_WARMUP_PRESS_JUMP_TH 1500.0f // 预热期若相邻压力跳变过大，重启预热
#define SENSOR_WARMUP_ALT_JUMP_TH   15.0f   // 预热期若相邻海拔跳变过大，重启预热
#define SENSOR_PUBLISH_STABLE_SAMPLES 6     // 预热后连续稳定样本数达到该值才对外发布
#define SENSOR_PUBLISH_PRESS_STABLE_TH 60.0f // 相邻滤波输出压力变化阈值 (Pa)
#define SENSOR_PUBLISH_ALT_STABLE_TH   0.8f  // 相邻滤波输出海拔变化阈值 (m)

// 动态低通滤波参数：
// 1. 静止时使用较小 alpha，优先抑制抖动。
// 2. 变化变大时自动放大 alpha，减少拖尾感。
// 3. 若现场仍觉得“太抖”，优先减小 LOW/MID；若觉得“太慢”，优先增大 HIGH。
#define BARO_LPF_ALPHA_TEMP_LOW   0.10f
#define BARO_LPF_ALPHA_TEMP_MID   0.20f
#define BARO_LPF_ALPHA_TEMP_HIGH  0.35f

#define BARO_LPF_ALPHA_PRESS_LOW  0.08f
#define BARO_LPF_ALPHA_PRESS_MID  0.16f
#define BARO_LPF_ALPHA_PRESS_HIGH 0.30f

#define BARO_LPF_ALPHA_ALT_LOW    0.08f
#define BARO_LPF_ALPHA_ALT_MID    0.18f
#define BARO_LPF_ALPHA_ALT_HIGH   0.35f

// 动态门限（按每次更新量判断）
#define BARO_TEMP_DELTA_LOW_TH    0.05f   // degC
#define BARO_TEMP_DELTA_HIGH_TH   0.20f   // degC
#define BARO_PRESS_DELTA_LOW_TH   8.0f    // Pa
#define BARO_PRESS_DELTA_HIGH_TH  40.0f   // Pa
#define BARO_ALT_DELTA_LOW_TH     0.05f   // m
#define BARO_ALT_DELTA_HIGH_TH    0.30f   // m

static float g_qnh = 101325.0f;

static void SPL06_SetQNH(float qnh)
{
    g_qnh = qnh;
}

static float SPL06_CalcTemperature(int32_t t_raw, const SPL06_Calib_t* calib)
{
    float t_scaled = (float)t_raw / KT;
    return calib->c0 * 0.5f + calib->c1 * t_scaled;
}

static float SPL06_CalcPressure(int32_t p_raw, int32_t t_raw, const SPL06_Calib_t* calib)
{
    float t_scaled = (float)t_raw / KT;
    float p_scaled = (float)p_raw / KP;

    float p_comp = (float)calib->c00
        + p_scaled * ((float)calib->c10 + p_scaled * ((float)calib->c20 + p_scaled * (float)calib->c30))
        + t_scaled * (float)calib->c01
        + t_scaled * p_scaled * ((float)calib->c11 + p_scaled * (float)calib->c21);

    return p_comp;
}

static float SPL06_CalcAltitude(float pressure)
{
    return 44330.0f * (1.0f - powf(pressure / g_qnh, 0.1903f));
}

static SPL06_Calib_t spl06Calib;
static SPL06_Ctx_t* spl06Ctx = NULL;
static Baro_Data_t latestData = {0};
static uint8_t hasValidData = 0;
static uint8_t filterInitialized = 0;
static float filteredTemp = 0.0f;
static float filteredPress = 0.0f;
static float filteredAlt = 0.0f;
static uint8_t warmupSampleCount = 0;
static float warmupTempSum = 0.0f;
static float warmupPressSum = 0.0f;
static float warmupAltSum = 0.0f;
static float warmupLastPress = 0.0f;
static float warmupLastAlt = 0.0f;
static uint8_t publishStableCount = 0;

// 错误统计
static uint32_t errorCount = 0;
static uint32_t lastUpdateTime = 0;
static uint32_t lastInitRetryTime = 0;
static uint8_t appReady = 0;

// 延时函数声明 (需要在main.c中实现)
extern void App_DelayMs(uint32_t ms);
extern uint32_t App_GetTickMs(void);

static float LowPassFilter(float prev, float input, float alpha)
{
    return prev + alpha * (input - prev);
}

static float SelectDynamicAlpha(float delta_abs,
                                float low_th,
                                float high_th,
                                float alpha_low,
                                float alpha_mid,
                                float alpha_high)
{
    if (delta_abs < low_th) {
        return alpha_low;
    }
    if (delta_abs < high_th) {
        return alpha_mid;
    }
    return alpha_high;
}

static void ResetWarmupState(void)
{
    hasValidData = 0;
    filterInitialized = 0;
    warmupSampleCount = 0;
    warmupTempSum = 0.0f;
    warmupPressSum = 0.0f;
    warmupAltSum = 0.0f;
    warmupLastPress = 0.0f;
    warmupLastAlt = 0.0f;
    publishStableCount = 0;
}

void Baro_Init(void)
{
    // 获取SPL06上下文
    spl06Ctx = STM32OF_GetSPL06Ctx();
    if (spl06Ctx == NULL) {
        appReady = 0;
        hasValidData = 0;
        return;
    }

    // 初始化传感器并读取校准系数
    if (SPL06_Init(spl06Ctx, &spl06Calib) != 0) {
        appReady = 0;
        hasValidData = 0;
        return;
    }

    // 设定默认海平面气压 (QNH)
    SPL06_SetQNH(101325.0f);

    // 初始化时间戳
    lastUpdateTime = App_GetTickMs();
    lastInitRetryTime = lastUpdateTime;
    errorCount = 0;
    appReady = 1;
    ResetWarmupState();
}

void Baro_Run(void)
{
    uint32_t currentTime = App_GetTickMs();

    if (!appReady) {
        if ((currentTime - lastInitRetryTime) < SENSOR_INIT_RETRY_PERIOD_MS) {
            return;
        }
        lastInitRetryTime = currentTime;

        spl06Ctx = STM32OF_GetSPL06Ctx();
        if (spl06Ctx != NULL && SPL06_Init(spl06Ctx, &spl06Calib) == 0) {
            SPL06_SetQNH(101325.0f);
            lastUpdateTime = currentTime;
            errorCount = 0;
            appReady = 1;
            ResetWarmupState();
        }
        return;
    }

    // 非阻塞延时检查
    if ((currentTime - lastUpdateTime) < SENSOR_UPDATE_PERIOD_MS) {
        return; // 未到更新时间
    }
    lastUpdateTime = currentTime;

    int32_t rawTemp, rawPress;
    int32_t ret_temp = SPL06_ReadRawTemp(spl06Ctx, &rawTemp);
    int32_t ret_press = SPL06_ReadRawPress(spl06Ctx, &rawPress);

    // 检查通信是否成功
    if (ret_temp != 0 || ret_press != 0) {
        errorCount++;
    } else {
        // 计算物理量
        float temp = SPL06_CalcTemperature(rawTemp, &spl06Calib);
        float press = SPL06_CalcPressure(rawPress, rawTemp, &spl06Calib);
        float alt = SPL06_CalcAltitude(press);

        // 数据有效性检查
        if (press > 0 && press < 200000 && temp > -50.0f && temp < 100.0f) {
            if (!filterInitialized) {
                if (warmupSampleCount > 0U
                    && (fabsf(press - warmupLastPress) > SENSOR_WARMUP_PRESS_JUMP_TH
                        || fabsf(alt - warmupLastAlt) > SENSOR_WARMUP_ALT_JUMP_TH)) {
                    warmupSampleCount = 0;
                    warmupTempSum = 0.0f;
                    warmupPressSum = 0.0f;
                    warmupAltSum = 0.0f;
                }

                warmupTempSum += temp;
                warmupPressSum += press;
                warmupAltSum += alt;
                warmupLastPress = press;
                warmupLastAlt = alt;
                warmupSampleCount++;
                errorCount = 0;

                if (warmupSampleCount < SENSOR_WARMUP_SAMPLES) {
                    return;
                }

                filteredTemp = warmupTempSum / (float)warmupSampleCount;
                filteredPress = warmupPressSum / (float)warmupSampleCount;
                filteredAlt = warmupAltSum / (float)warmupSampleCount;
                filterInitialized = 1;
                publishStableCount = 0;
                return;
            }

            {
                float prevFilteredPress = filteredPress;
                float prevFilteredAlt = filteredAlt;
                float temp_delta = fabsf(temp - filteredTemp);
                float press_delta = fabsf(press - filteredPress);
                float alt_delta = fabsf(alt - filteredAlt);

                float temp_alpha = SelectDynamicAlpha(temp_delta,
                                                      BARO_TEMP_DELTA_LOW_TH,
                                                      BARO_TEMP_DELTA_HIGH_TH,
                                                      BARO_LPF_ALPHA_TEMP_LOW,
                                                      BARO_LPF_ALPHA_TEMP_MID,
                                                      BARO_LPF_ALPHA_TEMP_HIGH);
                float press_alpha = SelectDynamicAlpha(press_delta,
                                                       BARO_PRESS_DELTA_LOW_TH,
                                                       BARO_PRESS_DELTA_HIGH_TH,
                                                       BARO_LPF_ALPHA_PRESS_LOW,
                                                       BARO_LPF_ALPHA_PRESS_MID,
                                                       BARO_LPF_ALPHA_PRESS_HIGH);
                float alt_alpha = SelectDynamicAlpha(alt_delta,
                                                     BARO_ALT_DELTA_LOW_TH,
                                                     BARO_ALT_DELTA_HIGH_TH,
                                                     BARO_LPF_ALPHA_ALT_LOW,
                                                     BARO_LPF_ALPHA_ALT_MID,
                                                     BARO_LPF_ALPHA_ALT_HIGH);

                filteredTemp = LowPassFilter(filteredTemp, temp, temp_alpha);
                filteredPress = LowPassFilter(filteredPress, press, press_alpha);
                filteredAlt = LowPassFilter(filteredAlt, alt, alt_alpha);

                if (!hasValidData) {
                    float stepPress = fabsf(filteredPress - prevFilteredPress);
                    float stepAlt = fabsf(filteredAlt - prevFilteredAlt);
                    if (stepPress < SENSOR_PUBLISH_PRESS_STABLE_TH
                        && stepAlt < SENSOR_PUBLISH_ALT_STABLE_TH) {
                        if (publishStableCount < 255U) {
                            publishStableCount++;
                        }
                    } else {
                        publishStableCount = 0;
                    }

                    if (publishStableCount < SENSOR_PUBLISH_STABLE_SAMPLES) {
                        errorCount = 0;
                        return;
                    }

                    hasValidData = 1;
                }
            }

            // 数据有效，刷新最新样本（输出滤波后的值）
            latestData.temperature = filteredTemp;
            latestData.pressure = filteredPress;
            latestData.altitude = filteredAlt;
            latestData.tick_ms = currentTime;
            hasValidData = 1;
            errorCount = 0;
        } else {
            // 数据异常
            errorCount++;
        }
    }

    // 容错恢复机制
    if (errorCount >= MAX_ERROR_THRESHOLD) {
        // 执行完整重初始化流程
        SPL06_Sleep(spl06Ctx);  // 先休眠传感器
        App_DelayMs(10);        // 短暂延时

        // 重新初始化
        if (SPL06_Init(spl06Ctx, &spl06Calib) == 0) {
            errorCount = 0;  // 重初始化成功
            ResetWarmupState();
        } else {
            appReady = 0;
            hasValidData = 0;
            lastInitRetryTime = currentTime;
        }
    }
}

bool Baro_GetLatestData(Baro_Data_t *out_data)
{
    if (out_data == NULL || hasValidData == 0) {
        return false;
    }

    *out_data = latestData;
    return true;
}

bool Baro_IsReady(void)
{
    return appReady != 0;
}