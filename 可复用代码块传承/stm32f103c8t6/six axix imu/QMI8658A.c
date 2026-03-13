#include "qmi8658a.h"

/**
  * @brief  读寄存器通用接口 (Driver Layer)
  * @param  ctx:  设备上下文句柄
  * @param  reg:  寄存器地址
  * @param  data: 数据缓冲区
  * @param  len:  长度
  * @retval QMI_OK / QMI_ERROR
  */
int32_t qmi8658_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
    if (ctx == NULL || ctx->read_reg == NULL) return QMI_ERROR;
    return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  写寄存器通用接口 (Driver Layer)
  */
int32_t qmi8658_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
    if (ctx == NULL || ctx->write_reg == NULL) return QMI_ERROR;
    return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  读取设备 ID (WHO_AM_I)
  */
int32_t qmi8658_device_id_get(stmdev_ctx_t *ctx, uint8_t *val) {
    return qmi8658_read_reg(ctx, QMI_WHO_AM_I, val, 1);
}

/**
  * @brief  执行软件复位
  */
int32_t qmi8658_reset_set(stmdev_ctx_t *ctx) {
    uint8_t reset_cmd = 0xB0;
    return qmi8658_write_reg(ctx, QMI_RESET, &reset_cmd, 1);
}

/**
  * @brief  基础初始化配置 (地址自增、大端模式、开启内部低通滤波)
  */
int32_t qmi8658_basic_init(stmdev_ctx_t *ctx) {
    qmi_ctrl1_t ctrl1;
    qmi_ctrl5_t ctrl5;
    int32_t ret;

    ret = qmi8658_read_reg(ctx, QMI_CTRL1, (uint8_t*)&ctrl1, 1);
    if(ret == QMI_OK) {
        ctrl1.addr_inc   = 1;
        ctrl1.big_endian = 0;
        ret = qmi8658_write_reg(ctx, QMI_CTRL1, (uint8_t*)&ctrl1, 1);
    }

    if(ret == QMI_OK) {
        ret = qmi8658_read_reg(ctx, QMI_CTRL5, (uint8_t*)&ctrl5, 1);
        if(ret == QMI_OK) {
            ctrl5.aLPF_EN = 1;
            ctrl5.aLPF_MODE = 0;
            ctrl5.gLPF_EN = 1;
            ctrl5.gLPF_MODE = 0;
            ret = qmi8658_write_reg(ctx, QMI_CTRL5, (uint8_t*)&ctrl5, 1);
        }
    }
    return ret;
}

int32_t qmi8658_accel_config_set(stmdev_ctx_t *ctx, qmi_acc_range_t range, qmi_odr_t odr) {
    qmi_ctrl2_t ctrl2;
    int32_t ret = qmi8658_read_reg(ctx, QMI_CTRL2, (uint8_t*)&ctrl2, 1);
    if (ret == QMI_OK) {
        ctrl2.range = (uint8_t)range;
        ctrl2.odr   = (uint8_t)odr;
        ret = qmi8658_write_reg(ctx, QMI_CTRL2, (uint8_t*)&ctrl2, 1);
    }
    return ret;
}

int32_t qmi8658_gyro_config_set(stmdev_ctx_t *ctx, qmi_gyro_range_t range, qmi_odr_t odr) {
    qmi_ctrl3_t ctrl3;
    int32_t ret = qmi8658_read_reg(ctx, QMI_CTRL3, (uint8_t*)&ctrl3, 1);
    if (ret == QMI_OK) {
        ctrl3.range = (uint8_t)range;
        ctrl3.odr   = (uint8_t)odr;
        ret = qmi8658_write_reg(ctx, QMI_CTRL3, (uint8_t*)&ctrl3, 1);
    }
    return ret;
}

int32_t qmi8658_sensor_enable_set(stmdev_ctx_t *ctx, uint8_t acc_on, uint8_t gyro_on) {
    qmi_ctrl7_t ctrl7;
    int32_t ret = qmi8658_read_reg(ctx, QMI_CTRL7, (uint8_t*)&ctrl7, 1);
    if (ret == QMI_OK) {
        ctrl7.acc_en  = acc_on ? 1 : 0;
        ctrl7.gyro_en = gyro_on ? 1 : 0;
        ret = qmi8658_write_reg(ctx, QMI_CTRL7, (uint8_t*)&ctrl7, 1);
    }
    return ret;
}

int32_t qmi8658_data_ready_get(stmdev_ctx_t *ctx, uint8_t *ready_flag) {
    uint8_t status = 0;
    int32_t ret = qmi8658_read_reg(ctx, QMI_STATUS0, &status, 1);
    if (ret == QMI_OK) {
        *ready_flag = (status & 0x03);
    }
    return ret;
}

int32_t qmi8658_read_raw_data(stmdev_ctx_t *ctx, int16_t *acc_raw, int16_t *gyro_raw, int16_t *temp_raw) {
    uint8_t buf[14];
    int32_t ret = qmi8658_read_reg(ctx, QMI_TEMP_L, buf, 14);
    if(ret == QMI_OK) {
        if(temp_raw) *temp_raw = (int16_t)((buf[1] << 8) | buf[0]);
        if(acc_raw) {
            acc_raw[0] = (int16_t)((buf[3] << 8) | buf[2]);
            acc_raw[1] = (int16_t)((buf[5] << 8) | buf[4]);
            acc_raw[2] = (int16_t)((buf[7] << 8) | buf[6]);
        }
        if(gyro_raw) {
            gyro_raw[0] = (int16_t)((buf[9] << 8) | buf[8]);
            gyro_raw[1] = (int16_t)((buf[11] << 8) | buf[10]);
            gyro_raw[2] = (int16_t)((buf[13] << 8) | buf[12]);
        }
    }
    return ret;
}

int32_t qmi8658_run_cod_calibration(stmdev_ctx_t *ctx) {
    uint8_t ctrl7_backup, status, cmd;

    qmi8658_read_reg(ctx, QMI_CTRL7, &ctrl7_backup, 1);
    uint8_t disable_all = 0x00;
    qmi8658_write_reg(ctx, QMI_CTRL7, &disable_all, 1);

    cmd = 0xA2;
    qmi8658_write_reg(ctx, QMI_CTRL9, &cmd, 1);

    if (ctx && ctx->mdelay) ctx->mdelay(1600);

    qmi8658_read_reg(ctx, QMI_COD_STATUS, &status, 1);
    qmi8658_write_reg(ctx, QMI_CTRL7, &ctrl7_backup, 1);

    return (status == 0x00) ? QMI_OK : QMI_ERROR;
}

float qmi_acc_to_g(int16_t raw_lsb, qmi_acc_range_t range) {
    switch (range) {
        case QMI_ACC_2G:  return (float)raw_lsb / 16384.0f;
        case QMI_ACC_4G:  return (float)raw_lsb / 8192.0f;
        case QMI_ACC_8G:  return (float)raw_lsb / 4096.0f;
        case QMI_ACC_16G: return (float)raw_lsb / 2048.0f;
        default:          return 0.0f;
    }
}

float qmi_gyro_to_dps(int16_t raw_lsb, qmi_gyro_range_t range) {
    switch (range) {
        case QMI_GYRO_16DPS:   return (float)raw_lsb / 2048.0f;
        case QMI_GYRO_32DPS:   return (float)raw_lsb / 1024.0f;
        case QMI_GYRO_64DPS:   return (float)raw_lsb / 512.0f;
        case QMI_GYRO_128DPS:  return (float)raw_lsb / 256.0f;
        case QMI_GYRO_256DPS:  return (float)raw_lsb / 128.0f;
        case QMI_GYRO_512DPS:  return (float)raw_lsb / 64.0f;
        case QMI_GYRO_1024DPS: return (float)raw_lsb / 32.0f;
        case QMI_GYRO_2048DPS: return (float)raw_lsb / 16.0f;
        default:               return 0.0f;
    }
}

float qmi_temp_to_celsius(int16_t raw_lsb) {
    return (float)raw_lsb / 256.0f;
}

int32_t qmi8658_static_offset_calib(stmdev_ctx_t *ctx, uint16_t samples,
                                    int16_t *acc_off, int16_t *gyro_off)
{
    int32_t sum_acc[3] = {0}, sum_gyro[3] = {0};
    int16_t acc[3], gyro[3], temp;

    if (!ctx || samples == 0) return QMI_ERROR;

    for (uint16_t i = 0; i < samples; i++) {
        if (qmi8658_read_raw_data(ctx, acc, gyro, &temp) != QMI_OK) return QMI_ERROR;
        for (int j = 0; j < 3; j++) {
            sum_acc[j]  += acc[j];
            sum_gyro[j] += gyro[j];
        }
        if (ctx->mdelay) ctx->mdelay(2);
    }

    for (int j = 0; j < 3; j++) {
        acc_off[j]  = (int16_t)(sum_acc[j] / samples);
        gyro_off[j] = (int16_t)(sum_gyro[j] / samples);
    }
    return QMI_OK;
}

int32_t qmi8658_calibration_run(stmdev_ctx_t *ctx, int16_t *acc_offset, int16_t *gyro_offset)
{
    int32_t ret;
    
    ret = qmi8658_run_cod_calibration(ctx);
    if (ret != QMI_OK) return ret;
    
    ret = qmi8658_static_offset_calib(ctx, 500, acc_offset, gyro_offset);
    if (ret != QMI_OK) return ret;
    
    return QMI_OK;
}

int32_t qmi8658_handle_init(qmi8658_handle_t *handle, stmdev_ctx_t *dev_ctx)
{
    if (!handle || !dev_ctx) return QMI_ERROR;
    
    handle->dev_ctx = *dev_ctx;
    for (int i = 0; i < 3; i++) {
        handle->acc_offset[i] = 0;
        handle->gyro_offset[i] = 0;
    }
    handle->acc_range = QMI_ACC_4G;
    handle->gyro_range = QMI_GYRO_128DPS;
    handle->odr = QMI_ODR_500Hz;
    handle->calibrated = 0;
    
    return QMI_OK;
}

int32_t qmi8658_calibration_auto(qmi8658_handle_t *handle, uint16_t samples)
{
    int32_t ret;
    
    if (!handle) return QMI_ERROR;
    
    ret = qmi8658_run_cod_calibration(&handle->dev_ctx);
    if (ret != QMI_OK) return ret;
    
    ret = qmi8658_static_offset_calib(&handle->dev_ctx, samples, 
                                       handle->acc_offset, handle->gyro_offset);
    if (ret != QMI_OK) return ret;
    
    handle->calibrated = 1;
    return QMI_OK;
}

/**
  * @brief  读取补偿后的物理数据 (Application Interface)
  * @param  handle:   QMI8658 句柄
  * @param  raw:      返回原始整数数据
  * @param  physical: 返回转换后的物理浮点数据 (g, dps, °C)
  */
int32_t qmi8658_read_data_compensated(qmi8658_handle_t *handle, 
                                       qmi_raw_data_t *raw, 
                                       qmi_physical_data_t *physical)
{
    int32_t ret;
    uint8_t ready;
    
    if (!handle || !raw || !physical) return QMI_ERROR;
    
    ret = qmi8658_data_ready_get(&handle->dev_ctx, &ready);
    if (ret != QMI_OK || !ready) return QMI_ERROR;
    
    ret = qmi8658_read_raw_data(&handle->dev_ctx, raw->acc, raw->gyro, &raw->temp);
    if (ret != QMI_OK) return ret;
    
    if (handle->calibrated) {
        raw->acc[0] -= handle->acc_offset[0];
        raw->acc[1] -= handle->acc_offset[1];
        raw->acc[2] -= handle->acc_offset[2];
        raw->gyro[0] -= handle->gyro_offset[0];
        raw->gyro[1] -= handle->gyro_offset[1];
        raw->gyro[2] -= handle->gyro_offset[2];
    }
    
    physical->acc[0] = qmi_acc_to_g(raw->acc[0], handle->acc_range);
    physical->acc[1] = qmi_acc_to_g(raw->acc[1], handle->acc_range);
    physical->acc[2] = qmi_acc_to_g(raw->acc[2], handle->acc_range);
    
    physical->gyro[0] = qmi_gyro_to_dps(raw->gyro[0], handle->gyro_range);
    physical->gyro[1] = qmi_gyro_to_dps(raw->gyro[1], handle->gyro_range);
    physical->gyro[2] = qmi_gyro_to_dps(raw->gyro[2], handle->gyro_range);
    
    physical->temp = qmi_temp_to_celsius(raw->temp);
    
    if (handle->lpf_alpha > 0.0f && handle->lpf_alpha <= 1.0f) {
        if (!handle->lpf_initialized) {
            for (int i = 0; i < 3; i++) {
                handle->acc_lpf[i] = physical->acc[i];
                handle->gyro_lpf[i] = physical->gyro[i];
            }
            handle->lpf_initialized = 1;
        } else {
            float alpha = handle->lpf_alpha;
            float beta = 1.0f - alpha;
            
            for (int i = 0; i < 3; i++) {
                handle->acc_lpf[i] = alpha * physical->acc[i] + beta * handle->acc_lpf[i];
                handle->gyro_lpf[i] = alpha * physical->gyro[i] + beta * handle->gyro_lpf[i];
            }
            
            physical->acc[0] = handle->acc_lpf[0];
            physical->acc[1] = handle->acc_lpf[1];
            physical->acc[2] = handle->acc_lpf[2];
            
            physical->gyro[0] = handle->gyro_lpf[0];
            physical->gyro[1] = handle->gyro_lpf[1];
            physical->gyro[2] = handle->gyro_lpf[2];
        }
    }
    
    return QMI_OK;
}

int32_t qmi8658_lpf_enable(qmi8658_handle_t *handle, float alpha)
{
    if (!handle) return QMI_ERROR;
    if (alpha <= 0.0f || alpha > 1.0f) return QMI_ERROR;
    
    handle->lpf_alpha = alpha;
    handle->lpf_initialized = 0;
    
    return QMI_OK;
}

int32_t qmi8658_lpf_disable(qmi8658_handle_t *handle)
{
    if (!handle) return QMI_ERROR;
    
    handle->lpf_alpha = 0.0f;
    handle->lpf_initialized = 0;
    
    return QMI_OK;
}

static int32_t qmi8658_ctrl9_write_cmd(stmdev_ctx_t *ctx, uint8_t cmd, 
                                       const uint8_t *cal_data, uint8_t cal_len)
{
    int32_t ret;
    uint8_t status;
    uint32_t timeout = 1000;
    
    if (cal_data != NULL && cal_len > 0) {
        ret = qmi8658_write_reg(ctx, QMI_CAL1_L, (uint8_t*)cal_data, cal_len);
        if (ret != QMI_OK) return ret;
    }
    
    ret = qmi8658_write_reg(ctx, QMI_CTRL9, &cmd, 1);
    if (ret != QMI_OK) return ret;
    
    do {
        ret = qmi8658_read_reg(ctx, QMI_STATUSINT, &status, 1);
        if (ret != QMI_OK) return ret;
        if (status & 0x80) break;
        if (ctx->mdelay) ctx->mdelay(1);
    } while (--timeout);
    
    if (timeout == 0) return QMI_ERROR;
    
    uint8_t ack = 0x00;
    ret = qmi8658_write_reg(ctx, QMI_CTRL9, &ack, 1);
    return ret;
}

int32_t qmi8658_configure_no_motion(stmdev_ctx_t *ctx, 
                                     uint8_t thr_x, uint8_t thr_y, uint8_t thr_z, 
                                     uint8_t window, uint8_t axis_en, uint8_t logic_and)
{
    uint8_t cal_buf[8];
    int32_t ret;
    
    cal_buf[0] = 0;
    cal_buf[1] = 0;
    cal_buf[2] = 0;
    cal_buf[3] = thr_x;
    cal_buf[4] = thr_y;
    cal_buf[5] = thr_z;
    cal_buf[6] = (logic_and ? 0x80 : 0x00) | 
                 ((axis_en & 0x04) ? 0x40 : 0x00) |
                 ((axis_en & 0x02) ? 0x20 : 0x00) |
                 ((axis_en & 0x01) ? 0x10 : 0x00);
    cal_buf[7] = 0x01;
    
    ret = qmi8658_ctrl9_write_cmd(ctx, 0x0E, cal_buf, 8);
    if (ret != QMI_OK) return ret;
    
    cal_buf[0] = 0;
    cal_buf[1] = window;
    cal_buf[2] = 0;
    cal_buf[3] = 0;
    cal_buf[4] = 0;
    cal_buf[5] = 0;
    cal_buf[6] = 0;
    cal_buf[7] = 0x02;
    
    ret = qmi8658_ctrl9_write_cmd(ctx, 0x0E, cal_buf, 8);
    if (ret != QMI_OK) return ret;
    
    return QMI_OK;
}

int32_t qmi8658_no_motion_enable(stmdev_ctx_t *ctx, uint8_t en)
{
    qmi_ctrl8_t ctrl8;
    int32_t ret = qmi8658_read_reg(ctx, QMI_CTRL8, (uint8_t*)&ctrl8, 1);
    if (ret != QMI_OK) return ret;
    ctrl8.no_motion_en = en ? 1 : 0;
    return qmi8658_write_reg(ctx, QMI_CTRL8, (uint8_t*)&ctrl8, 1);
}

int32_t qmi8658_no_motion_get_status(stmdev_ctx_t *ctx, uint8_t *detected)
{
    uint8_t status1;
    int32_t ret = qmi8658_read_reg(ctx, QMI_STATUS1, &status1, 1);
    if (ret == QMI_OK) {
        *detected = (status1 >> 6) & 0x01;
    }
    return ret;
}

static int32_t qmi8658_ctrl9_read_cmd(stmdev_ctx_t *ctx, uint8_t cmd, uint8_t *cal_data, uint8_t cal_len)
{
    int32_t ret;
    uint8_t status;
    uint32_t timeout = 1000;
    
    ret = qmi8658_write_reg(ctx, QMI_CTRL9, &cmd, 1);
    if (ret != QMI_OK) return ret;
    
    do {
        ret = qmi8658_read_reg(ctx, QMI_STATUSINT, &status, 1);
        if (ret != QMI_OK) return ret;
        if (status & 0x80) break;
        if (ctx->mdelay) ctx->mdelay(1);
    } while (--timeout);
    
    if (timeout == 0) return QMI_ERROR;
    
    if (cal_data != NULL && cal_len > 0) {
        ret = qmi8658_read_reg(ctx, QMI_CAL1_L, cal_data, cal_len);
        if (ret != QMI_OK) return ret;
    }
    
    uint8_t ack = 0x00;
    ret = qmi8658_write_reg(ctx, QMI_CTRL9, &ack, 1);
    return ret;
}

int32_t qmi8658_fifo_config(stmdev_ctx_t *ctx, qmi_fifo_size_t size, qmi_fifo_mode_t mode, uint8_t watermark)
{
    uint8_t fifo_ctrl;
    int32_t ret;
    
    ret = qmi8658_write_reg(ctx, QMI_FIFO_WTM_TH, &watermark, 1);
    if (ret != QMI_OK) return ret;
    
    fifo_ctrl = (size << 2) | (mode & 0x03);
    ret = qmi8658_write_reg(ctx, QMI_FIFO_CTRL, &fifo_ctrl, 1);
    return ret;
}

int32_t qmi8658_fifo_get_status(stmdev_ctx_t *ctx, uint8_t *fifo_status, uint16_t *sample_count)
{
    uint8_t smpl_lsb, smpl_msb;
    int32_t ret;
    
    ret = qmi8658_read_reg(ctx, QMI_FIFO_STATUS, fifo_status, 1);
    if (ret != QMI_OK) return ret;
    
    ret = qmi8658_read_reg(ctx, QMI_FIFO_SMPL_CNT, &smpl_lsb, 1);
    if (ret != QMI_OK) return ret;
    
    smpl_msb = (*fifo_status) & 0x03;
    *sample_count = (smpl_msb << 8) | smpl_lsb;
    return QMI_OK;
}

int32_t qmi8658_fifo_read_data(stmdev_ctx_t *ctx, uint8_t *data_buf, uint16_t max_len, uint16_t *bytes_read)
{
    uint16_t sample_count;
    uint8_t fifo_status;
    uint16_t need_bytes;
    int32_t ret;
    
    ret = qmi8658_fifo_get_status(ctx, &fifo_status, &sample_count);
    if (ret != QMI_OK) return ret;
    
    uint8_t bytes_per_sample = 12;
    need_bytes = sample_count * bytes_per_sample;
    if (need_bytes > max_len) {
        need_bytes = max_len;
    }
    
    if (need_bytes == 0) {
        *bytes_read = 0;
        return QMI_OK;
    }
    
    ret = qmi8658_ctrl9_read_cmd(ctx, 0x05, NULL, 0);
    if (ret != QMI_OK) return ret;
    
    ret = qmi8658_read_reg(ctx, QMI_FIFO_DATA, data_buf, need_bytes);
    if (ret != QMI_OK) {
        goto exit;
    }
    *bytes_read = need_bytes;

exit:
    {
        uint8_t fifo_ctrl;
        qmi8658_read_reg(ctx, QMI_FIFO_CTRL, &fifo_ctrl, 1);
        fifo_ctrl &= 0x7F;
        qmi8658_write_reg(ctx, QMI_FIFO_CTRL, &fifo_ctrl, 1);
    }
    return ret;
}

int32_t qmi8658_fifo_reset(stmdev_ctx_t *ctx)
{
    return qmi8658_ctrl9_read_cmd(ctx, 0x04, NULL, 0);
}

int32_t qmi8658_fifo_interrupt_config(stmdev_ctx_t *ctx, uint8_t map_to_int1, uint8_t enable_int)
{
    qmi_ctrl1_t ctrl1;
    int32_t ret;
    
    ret = qmi8658_read_reg(ctx, QMI_CTRL1, (uint8_t*)&ctrl1, 1);
    if (ret != QMI_OK) return ret;
    
    ctrl1.fifo_int_en = map_to_int1 ? 1 : 0;
    
    if (map_to_int1) {
        ctrl1.int1_en = enable_int ? 1 : 0;
    } else {
        ctrl1.int2_en = enable_int ? 1 : 0;
    }
    
    ret = qmi8658_write_reg(ctx, QMI_CTRL1, (uint8_t*)&ctrl1, 1);
    return ret;
}
