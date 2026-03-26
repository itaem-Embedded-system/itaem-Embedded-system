#ifndef GNSS_MAXM10S_DEVICE_H
#define GNSS_MAXM10S_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

/* Device 层维护的 UTC 解析结果，保留日期/时间是否齐全的信息。 */
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t msec;
    bool have_date;
    bool have_time;
} GNSS_maxm10s_utc_t;

/* Device 层输出给 app 层的原始解析结果，不包含业务门限判断。 */
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
    bool status_ok;
    bool have_coords;
    bool have_speed;
    bool have_quality;
    GNSS_maxm10s_utc_t utc;
} GNSS_maxm10s_parsed_t;

/* 清空 parser 的累计解析状态。 */
void GNSS_maxm10s_device_reset(void);
/* 解析一行 NMEA 文本，识别成功后更新内部解析快照。 */
void GNSS_maxm10s_device_parse_line(char *line);
/* 取出最近一次解析更新结果，属于消费式接口。 */
bool GNSS_maxm10s_device_take_parsed(GNSS_maxm10s_parsed_t *out);

#endif
