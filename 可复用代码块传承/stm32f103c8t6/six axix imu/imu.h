#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;
    uint32_t timestamp;
} imu_data_t;

typedef struct {
    bool no_motion_detected;
    uint32_t no_motion_count;
} imu_status_t;

typedef enum {
    IMU_OK = 0,
    IMU_ERROR = -1,
    IMU_ERROR_NOT_INIT = -2,
    IMU_ERROR_NO_DATA = -3,
} imu_status_code_t;

#define IMU_STATIC_ACC_TOLERANCE_G 0.01f
#define IMU_STATIC_GYRO_THR_DPS    1.0f
#define IMU_STATIC_DEBOUNCE_COUNT  6u
#define IMU_STATIC_DEBOUNCE_MS     100u

imu_status_code_t IMU_Init(void);
imu_status_code_t IMU_GetData(imu_data_t *data);
imu_status_code_t IMU_GetStatus(imu_status_t *status);
imu_status_code_t IMU_GetAccel(float *x, float *y, float *z);
imu_status_code_t IMU_GetGyro(float *x, float *y, float *z);
bool IMU_IsStatic(void);
void IMU_ResetNoMotionCount(void);
bool IMU_SoftwareStaticDetect(float ax, float ay, float az, float gx, float gy, float gz);
void IMU_UpdateStaticFromSample(float ax, float ay, float az, float gx, float gy, float gz);

#endif /* __IMU_APP_H__ */
