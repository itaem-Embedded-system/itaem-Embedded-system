#include "spl06.h"
#include <stddef.h>

// 超时时间定义
#define SPL06_INIT_TIMEOUT_MS  50
#define SPL06_RESET_TIMEOUT_MS 20
#define SPL06_RESET_CMD        0x89U

static int32_t validate_ctx(const SPL06_Ctx_t *ctx)
{
    if (ctx == NULL || ctx->read_reg == NULL || ctx->write_reg == NULL || ctx->delay_ms == NULL) {
        return SPL06_ERR_PARAM;
    }

    return SPL06_ERR_NONE;
}

static int32_t read_byte(SPL06_Ctx_t *ctx, uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return SPL06_ERR_PARAM;
    }

    if (ctx->read_reg(ctx->handle, reg, value, 1) != 0) {
        return SPL06_ERR_COMM;
    }

    return SPL06_ERR_NONE;
}

static int32_t write_byte(SPL06_Ctx_t *ctx, uint8_t reg, uint8_t value)
{
    if (ctx->write_reg(ctx->handle, reg, &value, 1) != 0) {
        return SPL06_ERR_COMM;
    }

    return SPL06_ERR_NONE;
}

static int32_t software_reset(SPL06_Ctx_t *ctx)
{
    uint8_t chip_id = 0;
    uint32_t timeout = 0;

    if (write_byte(ctx, SPL_RESET_REG, SPL06_RESET_CMD) != SPL06_ERR_NONE) {
        return SPL06_ERR_COMM;
    }

    while (timeout < SPL06_RESET_TIMEOUT_MS) {
        ctx->delay_ms(1);
        if (read_byte(ctx, SPL_ID_REG, &chip_id) == SPL06_ERR_NONE && chip_id == SPL_CHIP_ID) {
            return SPL06_ERR_NONE;
        }
        timeout++;
    }

    return SPL06_ERR_TIMEOUT;
}

static int32_t sign_extend_24bit(int32_t value)
{
    if (value & (1 << 23)) {
        value |= 0xFF000000;
    }
    return value;
}

static int32_t Read_C0(SPL06_Ctx_t* ctx, int16_t *out_value)
{
    uint8_t buff[2];
    int16_t c0;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C0, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C0_C1, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c0 = buff[0];
    c0 = (c0 << 4) | (buff[1] >> 4);
    if (c0 & (1 << 11)) {
        c0 |= 0xF000;
    }

    *out_value = c0;
    return SPL06_ERR_NONE;
}

static int32_t Read_C1(SPL06_Ctx_t* ctx, int16_t *out_value)
{
    uint8_t buff[2];
    int16_t c1;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C0_C1, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C1, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c1 = buff[0] & 0x0F;
    c1 = (c1 << 8) | buff[1];
    if (c1 & (1 << 11)) {
        c1 |= 0xF000;
    }

    *out_value = c1;
    return SPL06_ERR_NONE;
}

static int32_t Read_C00(SPL06_Ctx_t* ctx, int32_t *out_value)
{
    uint8_t buff[3];
    int32_t c00;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C00_H, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C00_L, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C00_C10, &buff[2]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c00 = buff[0];
    c00 = (c00 << 8) | buff[1];
    c00 = (c00 << 4) | (buff[2] >> 4);
    if (c00 & (1 << 19)) {
        c00 |= 0xFFF00000;
    }

    *out_value = c00;
    return SPL06_ERR_NONE;
}

static int32_t Read_C10(SPL06_Ctx_t* ctx, int32_t *out_value)
{
    uint8_t buff[3];
    int32_t c10;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C00_C10, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C10_M, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C10_L, &buff[2]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c10 = buff[0] & 0x0F;
    c10 = (c10 << 8) | buff[1];
    c10 = (c10 << 8) | buff[2];
    if (c10 & (1 << 19)) {
        c10 |= 0xFFF00000;
    }

    *out_value = c10;
    return SPL06_ERR_NONE;
}

static int32_t Read_C01(SPL06_Ctx_t* ctx, int16_t *out_value)
{
    uint8_t buff[2];
    int16_t c01;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C01_H, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C01_L, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c01 = buff[0];
    c01 = (c01 << 8) | buff[1];

    *out_value = c01;
    return SPL06_ERR_NONE;
}

static int32_t Read_C11(SPL06_Ctx_t* ctx, int16_t *out_value)
{
    uint8_t buff[2];
    int16_t c11;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C11_H, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C11_L, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c11 = buff[0];
    c11 = (c11 << 8) | buff[1];

    *out_value = c11;
    return SPL06_ERR_NONE;
}

static int32_t Read_C20(SPL06_Ctx_t* ctx, int16_t *out_value)
{
    uint8_t buff[2];
    int16_t c20;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C20_H, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C20_L, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c20 = buff[0];
    c20 = (c20 << 8) | buff[1];

    *out_value = c20;
    return SPL06_ERR_NONE;
}

static int32_t Read_C21(SPL06_Ctx_t* ctx, int16_t *out_value)
{
    uint8_t buff[2];
    int16_t c21;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C21_H, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C21_L, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c21 = buff[0];
    c21 = (c21 << 8) | buff[1];

    *out_value = c21;
    return SPL06_ERR_NONE;
}

static int32_t Read_C30(SPL06_Ctx_t* ctx, int16_t *out_value)
{
    uint8_t buff[2];
    int16_t c30;

    if (out_value == NULL) return SPL06_ERR_PARAM;
    if (read_byte(ctx, COEF_C30_H, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, COEF_C30_L, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    c30 = buff[0];
    c30 = (c30 << 8) | buff[1];
    *out_value = c30;
    return SPL06_ERR_NONE;
}

int32_t SPL06_Init(SPL06_Ctx_t* ctx, SPL06_Calib_t* calib)
{
    int32_t ret;
    uint8_t data;
    uint8_t prs_cfg;
    uint8_t tmp_cfg;
    uint8_t meas_cfg;
    uint8_t cfg_reg;
    uint32_t timeout = 0;

    ret = validate_ctx(ctx);
    if (ret != SPL06_ERR_NONE || calib == NULL) return SPL06_ERR_PARAM;

    // 验证传感器 ID
    if (read_byte(ctx, SPL_ID_REG, &data) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (data != SPL_CHIP_ID) return SPL06_ERR_ID; // ID 不匹配

    ret = software_reset(ctx);
    if (ret != SPL06_ERR_NONE) return ret;

    // 配置传感器工作模式
    if (write_byte(ctx, SPL_PRS_CFG, 0x01U) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (write_byte(ctx, SPL_TMP_CFG, 0x80U) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (write_byte(ctx, SPL_MEAS_CFG, 0x07U) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (write_byte(ctx, SPL_CFG_REG, 0x00U) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    // 等待传感器准备好
    while (timeout < SPL06_INIT_TIMEOUT_MS) {
        if (read_byte(ctx, SPL_MEAS_CFG, &data) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

        if ((data & (1 << 7)) && (data & (1 << 6))) {
            break;
        }
        timeout++;
        ctx->delay_ms(1);
    }

    if (timeout >= SPL06_INIT_TIMEOUT_MS) {
        return SPL06_ERR_TIMEOUT; // 超时
    }

    ret = Read_C0(ctx, &calib->c0); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C1(ctx, &calib->c1); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C01(ctx, &calib->c01); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C11(ctx, &calib->c11); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C20(ctx, &calib->c20); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C21(ctx, &calib->c21); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C30(ctx, &calib->c30); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C00(ctx, &calib->c00); if (ret != SPL06_ERR_NONE) return ret;
    ret = Read_C10(ctx, &calib->c10); if (ret != SPL06_ERR_NONE) return ret;

    // 检查传感器关键配置是否真实生效
    if (read_byte(ctx, SPL_PRS_CFG, &prs_cfg) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, SPL_TMP_CFG, &tmp_cfg) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, SPL_MEAS_CFG, &meas_cfg) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, SPL_CFG_REG, &cfg_reg) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (prs_cfg != 0x01 || tmp_cfg != 0x80 || (meas_cfg & 0x07) != 0x07 || cfg_reg != 0x00) {
        return SPL06_ERR_CONFIG;
    }

    return SPL06_ERR_NONE;
}

int32_t SPL06_Sleep(SPL06_Ctx_t* ctx)
{
    if (validate_ctx(ctx) != SPL06_ERR_NONE) return SPL06_ERR_PARAM;
    return write_byte(ctx, SPL_MEAS_CFG, 0x00U);
}

int32_t SPL06_Wakeup(SPL06_Ctx_t* ctx)
{
    if (validate_ctx(ctx) != SPL06_ERR_NONE) return SPL06_ERR_PARAM;
    return write_byte(ctx, SPL_MEAS_CFG, 0x07U);
}

int32_t SPL06_GetID(SPL06_Ctx_t* ctx)
{
    if (validate_ctx(ctx) != SPL06_ERR_NONE) return SPL06_ERR_PARAM;
    uint8_t id;
    if (read_byte(ctx, SPL_ID_REG, &id) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    return id;
}

int32_t SPL06_ReadRawTemp(SPL06_Ctx_t* ctx, int32_t* temp_raw)
{
    if (validate_ctx(ctx) != SPL06_ERR_NONE || temp_raw == NULL) return SPL06_ERR_PARAM;

    uint8_t buff[3];
    if (read_byte(ctx, SPL_TMP_B0, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, SPL_TMP_B1, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, SPL_TMP_B2, &buff[2]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    int32_t raw = buff[2];
    raw = (raw << 8) | buff[1];
    raw = (raw << 8) | buff[0];
    *temp_raw = sign_extend_24bit(raw);
    return SPL06_ERR_NONE;
}

int32_t SPL06_ReadRawPress(SPL06_Ctx_t* ctx, int32_t* press_raw)
{
    if (validate_ctx(ctx) != SPL06_ERR_NONE || press_raw == NULL) return SPL06_ERR_PARAM;

    uint8_t buff[3];
    if (read_byte(ctx, SPL_PRS_B0, &buff[0]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, SPL_PRS_B1, &buff[1]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;
    if (read_byte(ctx, SPL_PRS_B2, &buff[2]) != SPL06_ERR_NONE) return SPL06_ERR_COMM;

    int32_t raw = buff[2];
    raw = (raw << 8) | buff[1];
    raw = (raw << 8) | buff[0];
    *press_raw = sign_extend_24bit(raw);
    return SPL06_ERR_NONE;
}
