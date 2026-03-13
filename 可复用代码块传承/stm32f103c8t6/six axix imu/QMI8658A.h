#ifndef __QMI8658A_H
#define __QMI8658A_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define QMI_OK            0
#define QMI_ERROR        -1

typedef int32_t (*stmdev_write_ptr)(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len);
typedef int32_t (*stmdev_read_ptr)(void *handle, uint8_t reg, uint8_t *buf, uint16_t len);
typedef void    (*stmdev_mdelay_ptr)(uint32_t ms);

typedef struct {
    stmdev_write_ptr  write_reg;
    stmdev_read_ptr   read_reg;
    stmdev_mdelay_ptr mdelay;
    void              *handle;
} stmdev_ctx_t;

#define QMI_WHO_AM_I      0x00
#define QMI_REVISION      0x01
#define QMI_CTRL1         0x02
#define QMI_CTRL2         0x03
#define QMI_CTRL3         0x04
#define QMI_CTRL5         0x06
#define QMI_CTRL7         0x08
#define QMI_CTRL8         0x09
#define QMI_CTRL9         0x0A
#define QMI_CAL1_L        0x0B
#define QMI_CAL1_H        0x0C
#define QMI_CAL2_L        0x0D
#define QMI_CAL2_H        0x0E
#define QMI_CAL3_L        0x0F
#define QMI_CAL3_H        0x10
#define QMI_CAL4_L        0x11
#define QMI_CAL4_H        0x12
#define QMI_FIFO_WTM_TH   0x13
#define QMI_FIFO_CTRL     0x14
#define QMI_FIFO_SMPL_CNT 0x15
#define QMI_FIFO_STATUS   0x16
#define QMI_FIFO_DATA     0x17
#define QMI_STATUSINT     0x2D
#define QMI_STATUS0       0x2E
#define QMI_STATUS1       0x2F
#define QMI_TEMP_L        0x33
#define QMI_COD_STATUS    0x46
#define QMI_RESET         0x60

#define QMI_CHIP_ID       0x05

typedef struct {
    uint8_t sensor_dis  : 1;
    uint8_t reserved    : 1;
    uint8_t fifo_int_en : 1;
    uint8_t int1_en     : 1;
    uint8_t int2_en     : 1;
    uint8_t big_endian  : 1;
    uint8_t addr_inc    : 1;
    uint8_t spi_mode    : 1;
} qmi_ctrl1_t;

typedef struct {
    uint8_t odr   : 4;
    uint8_t range : 3;
    uint8_t st    : 1;
} qmi_ctrl2_t;

typedef struct {
    uint8_t odr   : 4;
    uint8_t range : 3;
    uint8_t st    : 1;
} qmi_ctrl3_t;

typedef struct {
    uint8_t aLPF_EN   : 1;
    uint8_t aLPF_MODE : 2;
    uint8_t reserved1 : 1;
    uint8_t gLPF_EN   : 1;
    uint8_t gLPF_MODE : 2;
    uint8_t reserved2 : 1;
} qmi_ctrl5_t;

typedef struct {
    uint8_t acc_en      : 1;
    uint8_t gyro_en     : 1;
    uint8_t reserved1   : 2;
    uint8_t gyro_snooze : 1;
    uint8_t drdy_dis    : 1;
    uint8_t reserved2   : 1;
    uint8_t sync_smpl   : 1;
} qmi_ctrl7_t;

typedef struct {
    uint8_t tap_en           : 1;
    uint8_t any_motion_en    : 1;
    uint8_t no_motion_en     : 1;
    uint8_t sig_motion_en    : 1;
    uint8_t pedo_en          : 1;
    uint8_t reserved         : 1;
    uint8_t activity_int_sel : 1;
    uint8_t ctrl9_handshake  : 1;
} qmi_ctrl8_t;

typedef struct {
    uint8_t fifo_mode     : 2;
    uint8_t reserved      : 6;
} qmi_fifo_ctrl_t;

typedef struct {
    uint8_t fifo_empty    : 1;
    uint8_t fifo_full     : 1;
    uint8_t fifo_overflow : 1;
    uint8_t wtm_flag      : 1;
    uint8_t reserved      : 4;
} qmi_fifo_status_t;

typedef struct {
    uint8_t reserved1       : 1;
    uint8_t tap_flag        : 1;
    uint8_t wom_flag        : 1;
    uint8_t reserved2       : 1;
    uint8_t pedometer_flag  : 1;
    uint8_t any_motion_flag : 1;
    uint8_t no_motion_flag  : 1;
    uint8_t sig_motion_flag : 1;
} qmi_status1_t;

typedef enum {
    QMI_MOTION_MODE_OR  = 0,
    QMI_MOTION_MODE_AND = 1,
} qmi_motion_axis_logic_t;

typedef enum {
    QMI_FIFO_MODE_BYPASS = 0,
    QMI_FIFO_MODE_FIFO   = 1,
    QMI_FIFO_MODE_STREAM = 2,
    QMI_FIFO_MODE_RESERVED = 3
} qmi_fifo_mode_t;

typedef enum {
    QMI_FIFO_SIZE_16  = 0,
    QMI_FIFO_SIZE_32  = 1,
    QMI_FIFO_SIZE_64  = 2,
    QMI_FIFO_SIZE_128 = 3
} qmi_fifo_size_t;

typedef struct {
    uint8_t any_motion_en_x : 1;
    uint8_t any_motion_en_y : 1;
    uint8_t any_motion_en_z : 1;
    uint8_t any_motion_logic: 1;
    uint8_t no_motion_en_x  : 1;
    uint8_t no_motion_en_y  : 1;
    uint8_t no_motion_en_z  : 1;
    uint8_t no_motion_logic : 1;
} qmi_motion_mode_ctrl_t;

typedef enum {
    QMI_ODR_1000Hz = 0x03,
    QMI_ODR_500Hz  = 0x04,
    QMI_ODR_250Hz  = 0x05,
    QMI_ODR_125Hz  = 0x06,
} qmi_odr_t;

typedef enum {
    QMI_ACC_2G  = 0x00,
    QMI_ACC_4G  = 0x01,
    QMI_ACC_8G  = 0x02,
    QMI_ACC_16G = 0x03,
} qmi_acc_range_t;

typedef enum {
    QMI_GYRO_16DPS   = 0x00,
    QMI_GYRO_32DPS   = 0x01,
    QMI_GYRO_64DPS   = 0x02,
    QMI_GYRO_128DPS  = 0x03,
    QMI_GYRO_256DPS  = 0x04,
    QMI_GYRO_512DPS  = 0x05,
    QMI_GYRO_1024DPS = 0x06,
    QMI_GYRO_2048DPS = 0x07,
} qmi_gyro_range_t;

typedef struct {
    int16_t acc[3];
    int16_t gyro[3];
    int16_t temp;
} qmi_raw_data_t;

typedef struct {
    float acc[3];
    float gyro[3];
    float temp;
} qmi_physical_data_t;

typedef struct {
    stmdev_ctx_t dev_ctx;
    int16_t acc_offset[3];
    int16_t gyro_offset[3];
    qmi_acc_range_t acc_range;
    qmi_gyro_range_t gyro_range;
    qmi_odr_t odr;
    uint8_t calibrated;
    
    float lpf_alpha;
    float acc_lpf[3];
    float gyro_lpf[3];
    uint8_t lpf_initialized;
} qmi8658_handle_t;

int32_t qmi8658_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
int32_t qmi8658_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);

int32_t qmi8658_device_id_get(stmdev_ctx_t *ctx, uint8_t *val);
int32_t qmi8658_reset_set(stmdev_ctx_t *ctx);
int32_t qmi8658_basic_init(stmdev_ctx_t *ctx);

int32_t qmi8658_accel_config_set(stmdev_ctx_t *ctx, qmi_acc_range_t range, qmi_odr_t odr);
int32_t qmi8658_gyro_config_set(stmdev_ctx_t *ctx, qmi_gyro_range_t range, qmi_odr_t odr);
int32_t qmi8658_sensor_enable_set(stmdev_ctx_t *ctx, uint8_t acc_on, uint8_t gyro_on);
int32_t qmi8658_data_ready_get(stmdev_ctx_t *ctx, uint8_t *ready_flag);
int32_t qmi8658_read_raw_data(stmdev_ctx_t *ctx, int16_t *acc_raw, int16_t *gyro_raw, int16_t *temp_raw);

int32_t qmi8658_run_cod_calibration(stmdev_ctx_t *ctx);

int32_t qmi8658_static_offset_calib(stmdev_ctx_t *ctx, uint16_t samples,
                                    int16_t *acc_off, int16_t *gyro_off);

int32_t qmi8658_calibration_run(stmdev_ctx_t *ctx, int16_t *acc_offset, int16_t *gyro_offset);

int32_t qmi8658_handle_init(qmi8658_handle_t *handle, stmdev_ctx_t *dev_ctx);
int32_t qmi8658_calibration_auto(qmi8658_handle_t *handle, uint16_t samples);
int32_t qmi8658_read_data_compensated(qmi8658_handle_t *handle, qmi_raw_data_t *raw, qmi_physical_data_t *physical);

float qmi_acc_to_g(int16_t raw_lsb, qmi_acc_range_t range);
float qmi_gyro_to_dps(int16_t raw_lsb, qmi_gyro_range_t range);
float qmi_temp_to_celsius(int16_t raw_lsb);

int32_t qmi8658_lpf_enable(qmi8658_handle_t *handle, float alpha);
int32_t qmi8658_lpf_disable(qmi8658_handle_t *handle);

int32_t qmi8658_no_motion_enable(stmdev_ctx_t *ctx, uint8_t en);
int32_t qmi8658_no_motion_get_status(stmdev_ctx_t *ctx, uint8_t *detected);
int32_t qmi8658_configure_no_motion(stmdev_ctx_t *ctx, 
                                     uint8_t thr_x, uint8_t thr_y, uint8_t thr_z, 
                                     uint8_t window, uint8_t axis_en, uint8_t logic_and);

int32_t qmi8658_fifo_config(stmdev_ctx_t *ctx, qmi_fifo_size_t size, qmi_fifo_mode_t mode, uint8_t watermark);
int32_t qmi8658_fifo_get_status(stmdev_ctx_t *ctx, uint8_t *fifo_status, uint16_t *sample_count);
int32_t qmi8658_fifo_read_data(stmdev_ctx_t *ctx, uint8_t *data_buf, uint16_t max_len, uint16_t *bytes_read);
int32_t qmi8658_fifo_reset(stmdev_ctx_t *ctx);
int32_t qmi8658_fifo_interrupt_config(stmdev_ctx_t *ctx, uint8_t map_to_int1, uint8_t enable_int);

#ifdef __cplusplus
}
#endif

#endif

