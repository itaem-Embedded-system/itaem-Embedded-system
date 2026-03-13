#include "imu.h"
#include "QMI8658A.h"
#include "platfrom_qmi8658a.h"
#include <math.h>

static bool imu_initialized = false;
static imu_data_t latest_data = {0};
static imu_status_t imu_status = {0};
static uint32_t static_window_start_ms = 0;
static uint8_t  static_window_active = 0;
static uint32_t static_debounce_count = 0;

static qmi8658_handle_t qmi_handle;

// 动态滤波参数定义
#define DYNA_FILT_ALPHA_MIN      0.05f   // 静止时强滤波，越小越平滑
#define DYNA_FILT_ALPHA_MAX      0.60f   // 运动时弱滤波，越大响应越快
#define DYNA_FILT_GYRO_THR_MIN   2.0f    // dps，低于此值认为是静止
#define DYNA_FILT_GYRO_THR_MAX   20.0f   // dps，高于此值认为是运动

/**
 * @brief  应用动态滤波算法 (Application Layer - Algorithm)
 * @note   根据陀螺仪模长动态调整一阶低通滤波系数 Alpha
 *         - 静止时 (Alpha小): 强滤波，消除零漂
 *         - 运动时 (Alpha大): 弱滤波，保证响应速度
 * @param  data: 指向 imu_data_t 的指针，将直接修改其中的数据
 */
static void IMU_ApplyDynamicFilter(imu_data_t *data) {
    static float acc_lpf[3] = {0};
    static float gyro_lpf[3] = {0};
    static bool first_run = true;
    
    // 1. 计算当前陀螺仪模长（作为运动强度的指标）
    float gyro_norm = sqrtf(data->gyro_x * data->gyro_x + 
                            data->gyro_y * data->gyro_y + 
                            data->gyro_z * data->gyro_z);
    
    // 2. 动态计算 Alpha
    float alpha;
    if (gyro_norm <= DYNA_FILT_GYRO_THR_MIN) {
        alpha = DYNA_FILT_ALPHA_MIN;
    } else if (gyro_norm >= DYNA_FILT_GYRO_THR_MAX) {
        alpha = DYNA_FILT_ALPHA_MAX;
    } else {
        // 线性插值
        float ratio = (gyro_norm - DYNA_FILT_GYRO_THR_MIN) / (DYNA_FILT_GYRO_THR_MAX - DYNA_FILT_GYRO_THR_MIN);
        alpha = DYNA_FILT_ALPHA_MIN + ratio * (DYNA_FILT_ALPHA_MAX - DYNA_FILT_ALPHA_MIN);
    }
    
    // 3. 初始化或应用滤波
    if (first_run) {
        acc_lpf[0] = data->acc_x; acc_lpf[1] = data->acc_y; acc_lpf[2] = data->acc_z;
        gyro_lpf[0] = data->gyro_x; gyro_lpf[1] = data->gyro_y; gyro_lpf[2] = data->gyro_z;
        first_run = false;
    } else {
        acc_lpf[0] = acc_lpf[0] * (1.0f - alpha) + data->acc_x * alpha;
        acc_lpf[1] = acc_lpf[1] * (1.0f - alpha) + data->acc_y * alpha;
        acc_lpf[2] = acc_lpf[2] * (1.0f - alpha) + data->acc_z * alpha;
        
        gyro_lpf[0] = gyro_lpf[0] * (1.0f - alpha) + data->gyro_x * alpha;
        gyro_lpf[1] = gyro_lpf[1] * (1.0f - alpha) + data->gyro_y * alpha;
        gyro_lpf[2] = gyro_lpf[2] * (1.0f - alpha) + data->gyro_z * alpha;
        
        // 写回数据
        data->acc_x = acc_lpf[0]; data->acc_y = acc_lpf[1]; data->acc_z = acc_lpf[2];
        data->gyro_x = gyro_lpf[0]; data->gyro_y = gyro_lpf[1]; data->gyro_z = gyro_lpf[2];
    }
}

/**
 * @brief  IMU 模块初始化 (Application Layer - Init)
 * @note   包含 I2C 地址探测、软复位、寄存器配置、自检流程
 * @retval IMU_OK: 初始化成功
 *         IMU_ERROR_NOT_INIT: 设备未找到
 *         IMU_ERROR: 配置失败或自检不通过
 */
imu_status_code_t IMU_Init(void)
{
    uint8_t id;
    bool success = false;
    
    // 尝试探测地址 0x6B
    qmi8658_ctx.handle = (void*)(uint32_t)0x6B;
    if (qmi8658_device_id_get(&qmi8658_ctx, &id) == QMI_OK && id == QMI_CHIP_ID) {
        success = true;
    } else {
        // 尝试探测地址 0x6A
        qmi8658_ctx.handle = (void*)(uint32_t)0x6A;
        if (qmi8658_device_id_get(&qmi8658_ctx, &id) == QMI_OK && id == QMI_CHIP_ID) {
            success = true;
        }
    }
    
    if (!success) {
        return IMU_ERROR_NOT_INIT;
    }
    
    qmi8658_reset_set(&qmi8658_ctx);
    qmi8658_ctx.mdelay(20);
    
    if (qmi8658_basic_init(&qmi8658_ctx) != QMI_OK) {
        return IMU_ERROR;
    }
    
    // 初始化 handle
    qmi8658_handle_init(&qmi_handle, &qmi8658_ctx);
    
    qmi8658_accel_config_set(&qmi8658_ctx, QMI_ACC_4G, QMI_ODR_500Hz);
    qmi8658_gyro_config_set(&qmi8658_ctx, QMI_GYRO_128DPS, QMI_ODR_500Hz);
    
    qmi8658_configure_no_motion(&qmi8658_ctx, 3, 3, 3, 50, 0x07, 1);
    qmi8658_no_motion_enable(&qmi8658_ctx, 1);
    
    // 禁用 FIFO：不复位和配置 FIFO，改为寄存器轮询读取
    // qmi8658_fifo_reset(&qmi8658_ctx);
    // qmi8658_fifo_config(&qmi8658_ctx, QMI_FIFO_SIZE_64, QMI_FIFO_MODE_FIFO, 32);
    
    qmi8658_sensor_enable_set(&qmi8658_ctx, 1, 1);
    qmi8658_ctx.mdelay(160);
    
    // 初始化自检：读取一次数据，确保非全零
    qmi_raw_data_t raw;
    qmi_physical_data_t phys;
    bool data_valid = false;
    for (int i = 0; i < 5; i++) {
        if (qmi8658_read_data_compensated(&qmi_handle, &raw, &phys) == QMI_OK) {
            // 简单判断：只要加速度模长不为0，或任意轴不为0
            if (fabsf(phys.acc[0]) > 0.001f || fabsf(phys.acc[1]) > 0.001f || fabsf(phys.acc[2]) > 0.0001f) {
                data_valid = true;
                break;
            }
        }
        qmi8658_ctx.mdelay(20);
    }
    
    if (!data_valid) {
        // 自检失败，尝试复位
        qmi8658_reset_set(&qmi8658_ctx);
        return IMU_ERROR;
    }
    
    // 开启软件低通滤波 (alpha=0.1, 值越小越平滑但延迟越高)
    // qmi8658_lpf_enable(&qmi_handle, 0.1f); // 禁用固定滤波，改用动态滤波
    qmi8658_lpf_disable(&qmi_handle);
    
    imu_initialized = true;
    return IMU_OK;
}

/**
 * @brief  获取 IMU 数据 (Application Layer - Data Access)
 * @note   包含自动重连机制、数据读取、动态滤波处理、静止检测更新
 * @param  data: 输出数据结构体指针
 * @retval IMU_OK: 获取成功
 *         IMU_ERROR_NO_DATA: 读取失败或数据无效
 */
imu_status_code_t IMU_GetData(imu_data_t *data)
{
    if (!imu_initialized) {
        // 尝试自动重试初始化
        static uint32_t last_retry = 0;
        uint32_t now = platform_get_tick();
        if ((now - last_retry) > 1000) {
            last_retry = now;
            if (IMU_Init() == IMU_OK) {
                // 初始化成功，继续执行
            } else {
                return IMU_ERROR_NOT_INIT;
            }
        } else {
            return IMU_ERROR_NOT_INIT;
        }
    }
    
    if (!data) return IMU_ERROR;

    qmi_raw_data_t raw;
    qmi_physical_data_t phys;
    
    // 连续错误计数器
    static uint8_t error_count = 0;
    
    if (qmi8658_read_data_compensated(&qmi_handle, &raw, &phys) != QMI_OK) {
        // 读取失败
        error_count++;
        if (error_count > 10) {
            imu_initialized = false; // 触发重置
            error_count = 0;
        }
        
        // 清除静止状态
        static_window_active = 0;
        static_window_start_ms = 0;
        imu_status.no_motion_detected = false;
        imu_status.no_motion_count = 0;
        return IMU_ERROR_NO_DATA;
    }
    
    // 数据全零检测（异常情况）
    if (fabsf(phys.acc[0]) < 0.000001f && fabsf(phys.acc[1]) < 0.000001f && fabsf(phys.acc[2]) < 0.000001f &&
        fabsf(phys.gyro[0]) < 0.000001f && fabsf(phys.gyro[1]) < 0.000001f && fabsf(phys.gyro[2]) < 0.000001f) {
        error_count++;
        if (error_count > 20) { // 连续20次全零则重启
            imu_initialized = false;
            error_count = 0;
        }
        return IMU_ERROR_NO_DATA;
    }
    
    error_count = 0; // 读取成功且数据有效，清零错误计数
    
    data->acc_x = phys.acc[0];
    data->acc_y = phys.acc[1];
    data->acc_z = phys.acc[2];
    data->gyro_x = phys.gyro[0];
    data->gyro_y = phys.gyro[1];
    data->gyro_z = phys.gyro[2];
    data->temperature = phys.temp;
    data->timestamp = platform_get_tick();
    
    // 应用动态滤波
    IMU_ApplyDynamicFilter(data);
    
    latest_data = *data;
    
    IMU_UpdateStaticFromSample(phys.acc[0], phys.acc[1], phys.acc[2],
                               phys.gyro[0], phys.gyro[1], phys.gyro[2]);
    
    return IMU_OK;
}

imu_status_code_t IMU_GetStatus(imu_status_t *status)
{
    if (!imu_initialized || !status) {
        return IMU_ERROR_NOT_INIT;
    }
    
    *status = imu_status;
    return IMU_OK;
}

imu_status_code_t IMU_GetAccel(float *x, float *y, float *z)
{
    if (!imu_initialized) {
        return IMU_ERROR_NOT_INIT;
    }
    
    if (x) *x = latest_data.acc_x;
    if (y) *y = latest_data.acc_y;
    if (z) *z = latest_data.acc_z;
    
    return IMU_OK;
}

imu_status_code_t IMU_GetGyro(float *x, float *y, float *z)
{
    if (!imu_initialized) {
        return IMU_ERROR_NOT_INIT;
    }
    
    if (x) *x = latest_data.gyro_x;
    if (y) *y = latest_data.gyro_y;
    if (z) *z = latest_data.gyro_z;
    
    return IMU_OK;
}

bool IMU_IsStatic(void)
{
    if (!imu_initialized) {
        return false;
    }
    qmi_raw_data_t raw;
    qmi_physical_data_t phys;
    bool sw_static = false;
    if (qmi8658_read_data_compensated(&qmi_handle, &raw, &phys) == IMU_OK) {
        sw_static = IMU_SoftwareStaticDetect(phys.acc[0], phys.acc[1], phys.acc[2],
                                             phys.gyro[0], phys.gyro[1], phys.gyro[2]);
    }
    uint8_t hw_flag = 0;
    bool hw_static = false;
    if (qmi8658_no_motion_get_status(&qmi8658_ctx, &hw_flag) == IMU_OK) {
        hw_static = (hw_flag != 0);
    }
    bool combined = (sw_static || hw_static);
    uint32_t now = platform_get_tick();
    if (combined) {
        if (!static_window_active) {
            static_window_active = 1;
            static_window_start_ms = now;
        }
        uint32_t elapsed = now - static_window_start_ms;
        imu_status.no_motion_detected = (elapsed >= IMU_STATIC_DEBOUNCE_MS);
        imu_status.no_motion_count = elapsed;
    } else {
        static_window_active = 0;
        static_window_start_ms = 0;
        imu_status.no_motion_detected = false;
        imu_status.no_motion_count = 0;
    }
    return imu_status.no_motion_detected;
}

void IMU_ResetNoMotionCount(void)
{
    static_debounce_count = 0;
    static_window_active = 0;
    static_window_start_ms = 0;
    imu_status.no_motion_count = 0;
}

bool IMU_SoftwareStaticDetect(float ax, float ay, float az, float gx, float gy, float gz)
{
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);
    return (fabsf(acc_norm - 1.0f) < IMU_STATIC_ACC_TOLERANCE_G) && (gyro_norm < IMU_STATIC_GYRO_THR_DPS);
}

void IMU_UpdateStaticFromSample(float ax, float ay, float az, float gx, float gy, float gz)
{
    bool sw_static = IMU_SoftwareStaticDetect(ax, ay, az, gx, gy, gz);
    uint8_t hw_flag = 0;
    bool hw_static = false;
    if (qmi8658_no_motion_get_status(&qmi8658_ctx, &hw_flag) == IMU_OK) {
        hw_static = (hw_flag != 0);
    }
    bool combined = (sw_static || hw_static);
    if (combined) {
        if (static_debounce_count < UINT32_MAX) static_debounce_count++;
    } else {
        static_debounce_count = 0;
    }
    imu_status.no_motion_detected = (static_debounce_count >= IMU_STATIC_DEBOUNCE_COUNT);
    imu_status.no_motion_count = static_debounce_count;
}
