#ifndef GNSS_MAXM10S_H
#define GNSS_MAXM10S_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* App 层对外发布的定位快照。 */
typedef struct {
    double lat_deg;
    double lon_deg;
    double alt_m;
    float speed_kn;
    float speed_kmh;
    float track_deg;
    uint8_t sats;
    float hdop;
    float pdop;
    uint8_t fix_type;
    bool valid;
    struct {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint16_t msec;
    } utc;
} GNSS_maxm10s_fix_t;

/* 供业务层直接使用的本地时间结构，默认按 UTC+8 转换。 */
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t msec;
} GNSS_maxm10s_time_t;

typedef enum {
    GNSS_MAXM10S_STATE_INIT = 0,
    GNSS_MAXM10S_STATE_SEARCHING,
    GNSS_MAXM10S_STATE_FIXED,
    GNSS_MAXM10S_STATE_DEGRADED,
    GNSS_MAXM10S_STATE_RECOVERING,
    GNSS_MAXM10S_STATE_FAULT,
} GNSS_maxm10s_state_t;

typedef enum {
    GNSS_MAXM10S_FAULT_NONE = 0,
    GNSS_MAXM10S_FAULT_NO_SENTENCE_TIMEOUT,
    GNSS_MAXM10S_FAULT_SEARCH_TIMEOUT,
    GNSS_MAXM10S_FAULT_RECOVERY_LIMIT,
} GNSS_maxm10s_fault_t;

typedef struct {
    GNSS_maxm10s_state_t state;
    GNSS_maxm10s_fault_t fault;
    uint32_t init_tick_ms;
    uint32_t last_sentence_tick_ms;
    uint32_t last_valid_fix_tick_ms;
    uint32_t last_recovery_tick_ms;
    uint16_t recovery_count;
    uint16_t no_sentence_timeout_count;
    uint16_t search_timeout_count;
    bool recovery_limit_reached;
} GNSS_maxm10s_status_t;

/* 绑定 GNSS 使用的串口，并重置 app 层发布态与状态机。 */
void GNSS_maxm10s_init(UART_HandleTypeDef *uart);

/* 在主循环中持续调用，驱动接收、解析、有效性判断和状态更新。 */
void GNSS_maxm10s_update(void);

/* 便捷接口：先更新内部状态，再尝试返回最近一次有效经纬度。 */
bool GNSS_maxm10s_update_and_get_location(double *lat_deg, double *lon_deg);

/* 取出一帧新的有效定位，属于消费式接口。 */
bool GNSS_maxm10s_get_fix(GNSS_maxm10s_fix_t *out);

/* 读取最近一次有效定位快照，可重复读取。 */
bool GNSS_maxm10s_get_latest_fix(GNSS_maxm10s_fix_t *out);

/* 一次读取同一快照中的经纬度。 */
bool GNSS_maxm10s_get_location(double *lat_deg, double *lon_deg);

/* 读取最近一次有效定位的海拔，单位米。 */
bool GNSS_maxm10s_get_altitude(double *alt_m);

/* 读取最近一次有效定位的速度，单位 km/h。 */
bool GNSS_maxm10s_get_speed_kmh(float *speed_kmh);
/* 读取最近一次有效定位的航向角，单位度。 */
bool GNSS_maxm10s_get_heading(float *heading_deg);
/* 读取最近一次有效定位的本地时间，默认按 UTC+8 计算。 */
bool GNSS_maxm10s_get_local_time(GNSS_maxm10s_time_t *out);
/* 查询当前是否已有可读的最近有效定位。 */
bool GNSS_maxm10s_is_valid(bool *out);
/* 读取 app 层状态机和故障统计。 */
bool GNSS_maxm10s_get_status(GNSS_maxm10s_status_t *out);

#endif
