#include "GNSS_maxm10s.h"

#include "GNSS_maxm10s_device.h"
#include "GNSS_maxm10s_port.h"

#include <math.h>
#include <string.h>

#define GNSS_MAXM10S_LINE_MAX_LEN 160u
#define GNSS_MAXM10S_STARTUP_GRACE_MS 60000u
#define GNSS_MAXM10S_NO_SENTENCE_TIMEOUT_MS 3000u
#define GNSS_MAXM10S_NO_FIX_TIMEOUT_MS 60000u
#define GNSS_MAXM10S_RECOVERY_COOLDOWN_MS 10000u
#define GNSS_MAXM10S_MAX_RECOVERY_COUNT 5u

static GNSS_maxm10s_fix_t s_fix;
static GNSS_maxm10s_fix_t s_last_valid_fix;
static bool s_fix_ready = false;
static bool s_last_valid_fix_ready = false;
static GNSS_maxm10s_status_t s_status;
static UART_HandleTypeDef *s_uart = NULL;
static bool s_seen_sentence = false;
static bool s_seen_valid_fix = false;

static const GNSS_maxm10s_fix_t *get_latest_valid_fix_ptr(void)
{
    if (!s_last_valid_fix_ready) {
        return NULL;
    }

    return &s_last_valid_fix;
}

static bool is_leap_year(uint16_t year)
{
    return ((year % 4u) == 0u) && ((((year % 100u) != 0u) || ((year % 400u) == 0u)));
}

static uint8_t days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t s_days[12] = {
        31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u,
    };

    if ((month == 0u) || (month > 12u)) {
        return 0u;
    }

    if ((month == 2u) && is_leap_year(year)) {
        return 29u;
    }

    return s_days[month - 1u];
}

static bool convert_utc_to_local_time(const GNSS_maxm10s_fix_t *fix, GNSS_maxm10s_time_t *out)
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t max_day;
    uint16_t total_hour;

    if ((fix == NULL) || (out == NULL)) {
        return false;
    }

    year = fix->utc.year;
    month = fix->utc.month;
    day = fix->utc.day;
    max_day = days_in_month(year, month);

    if ((year == 0u) || (max_day == 0u) || (day == 0u) || (day > max_day)) {
        return false;
    }

    total_hour = (uint16_t)fix->utc.hour + 8u;

    out->year = year;
    out->month = month;
    out->day = day;
    out->hour = (uint8_t)(total_hour % 24u);
    out->minute = fix->utc.minute;
    out->second = fix->utc.second;
    out->msec = fix->utc.msec;

    if (total_hour < 24u) {
        return true;
    }

    out->day++;
    max_day = days_in_month(out->year, out->month);
    if (out->day <= max_day) {
        return true;
    }

    out->day = 1u;
    out->month++;
    if (out->month <= 12u) {
        return true;
    }

    out->month = 1u;
    out->year++;
    return true;
}

static bool tick_elapsed(uint32_t start_tick, uint32_t period_ms, uint32_t now_tick)
{
    return (uint32_t)(now_tick - start_tick) >= period_ms;
}

static void set_state(GNSS_maxm10s_state_t state)
{
    s_status.state = state;
}

static void reset_publish_state(void)
{
    memset(&s_fix, 0, sizeof(s_fix));
    memset(&s_last_valid_fix, 0, sizeof(s_last_valid_fix));
    s_fix_ready = false;
    s_last_valid_fix_ready = false;
}

static void perform_recovery(GNSS_maxm10s_fault_t fault, uint32_t now_tick)
{
    if (s_status.recovery_count >= GNSS_MAXM10S_MAX_RECOVERY_COUNT) {
        s_status.fault = GNSS_MAXM10S_FAULT_RECOVERY_LIMIT;
        s_status.recovery_limit_reached = true;
        set_state(GNSS_MAXM10S_STATE_FAULT);
        return;
    }

    reset_publish_state();
    GNSS_maxm10s_device_reset();
    GNSS_maxm10s_port_init(s_uart);

    s_seen_sentence = false;
    s_seen_valid_fix = false;
    s_status.fault = fault;
    s_status.recovery_count++;
    s_status.last_recovery_tick_ms = now_tick;
    set_state(GNSS_MAXM10S_STATE_RECOVERING);
}

static bool update_fix_from_parsed(const GNSS_maxm10s_parsed_t *parsed)
{
    bool coords_ok;
    bool sats_ok;
    bool dop_ok;
    bool type_ok;

    if (parsed == NULL) {
        return false;
    }

    coords_ok = parsed->have_coords && isfinite(parsed->lat_deg) && isfinite(parsed->lon_deg);
    sats_ok = parsed->sats >= 4u;
    dop_ok = (parsed->hdop > 0.0f) && (parsed->hdop < 99.99f) &&
             (parsed->pdop > 0.0f) && (parsed->pdop < 99.99f);
    type_ok = parsed->fix_type >= 2u;

    s_fix.valid = parsed->status_ok && coords_ok && sats_ok && dop_ok && type_ok;
    if (!s_fix.valid) {
        return false;
    }

    s_fix.lat_deg = parsed->lat_deg;
    s_fix.lon_deg = parsed->lon_deg;
    s_fix.alt_m = parsed->alt_m;
    s_fix.speed_kn = parsed->speed_kn;
    s_fix.speed_kmh = parsed->speed_kmh;
    s_fix.track_deg = parsed->track_deg;
    s_fix.sats = parsed->sats;
    s_fix.hdop = parsed->hdop;
    s_fix.pdop = parsed->pdop;
    s_fix.fix_type = parsed->fix_type;

    if (parsed->utc.have_time) {
        s_fix.utc.hour = parsed->utc.hour;
        s_fix.utc.minute = parsed->utc.minute;
        s_fix.utc.second = parsed->utc.second;
        s_fix.utc.msec = parsed->utc.msec;
    }
    if (parsed->utc.have_date) {
        s_fix.utc.year = parsed->utc.year;
        s_fix.utc.month = parsed->utc.month;
        s_fix.utc.day = parsed->utc.day;
    }

    s_fix_ready = true;
    s_last_valid_fix = s_fix;
    s_last_valid_fix_ready = true;
    return true;
}

static void handle_validity_state(bool got_valid_fix)
{
    if (got_valid_fix) {
        s_seen_valid_fix = true;
        s_status.fault = GNSS_MAXM10S_FAULT_NONE;
        set_state(GNSS_MAXM10S_STATE_FIXED);
        return;
    }

    if (!s_seen_sentence) {
        return;
    }

    if (s_seen_valid_fix) {
        if (s_status.state != GNSS_MAXM10S_STATE_RECOVERING) {
            set_state(GNSS_MAXM10S_STATE_DEGRADED);
        }
    } else if ((s_status.state != GNSS_MAXM10S_STATE_RECOVERING) &&
               (s_status.state != GNSS_MAXM10S_STATE_FAULT)) {
        set_state(GNSS_MAXM10S_STATE_SEARCHING);
    }
}

static void update_runtime_state(uint32_t now_tick)
{
    if (s_status.state == GNSS_MAXM10S_STATE_FAULT) {
        return;
    }

    if (s_status.state == GNSS_MAXM10S_STATE_RECOVERING) {
        if (!tick_elapsed(s_status.last_recovery_tick_ms, GNSS_MAXM10S_RECOVERY_COOLDOWN_MS, now_tick)) {
            return;
        }

        if (s_seen_valid_fix) {
            set_state(GNSS_MAXM10S_STATE_FIXED);
        } else if (s_seen_sentence) {
            set_state(GNSS_MAXM10S_STATE_SEARCHING);
        } else {
            set_state(GNSS_MAXM10S_STATE_INIT);
        }
    }

    if (!s_seen_sentence) {
        if (tick_elapsed(s_status.init_tick_ms, GNSS_MAXM10S_STARTUP_GRACE_MS, now_tick)) {
            s_status.no_sentence_timeout_count++;
            perform_recovery(GNSS_MAXM10S_FAULT_NO_SENTENCE_TIMEOUT, now_tick);
        }
        return;
    }

    if (tick_elapsed(s_status.last_sentence_tick_ms, GNSS_MAXM10S_NO_SENTENCE_TIMEOUT_MS, now_tick)) {
        s_status.no_sentence_timeout_count++;
        perform_recovery(GNSS_MAXM10S_FAULT_NO_SENTENCE_TIMEOUT, now_tick);
        return;
    }

    if (!s_seen_valid_fix) {
        if (tick_elapsed(s_status.init_tick_ms, GNSS_MAXM10S_NO_FIX_TIMEOUT_MS, now_tick)) {
            s_status.search_timeout_count++;
            perform_recovery(GNSS_MAXM10S_FAULT_SEARCH_TIMEOUT, now_tick);
        } else if (s_status.state == GNSS_MAXM10S_STATE_INIT) {
            set_state(GNSS_MAXM10S_STATE_SEARCHING);
        }
        return;
    }

    if ((s_status.state == GNSS_MAXM10S_STATE_DEGRADED) &&
        tick_elapsed(s_status.last_valid_fix_tick_ms, GNSS_MAXM10S_NO_FIX_TIMEOUT_MS, now_tick)) {
        s_status.search_timeout_count++;
        perform_recovery(GNSS_MAXM10S_FAULT_SEARCH_TIMEOUT, now_tick);
    }
}

void GNSS_maxm10s_init(UART_HandleTypeDef *uart)
{
    uint32_t now_tick = HAL_GetTick();

    reset_publish_state();
    memset(&s_status, 0, sizeof(s_status));
    s_uart = uart;
    s_seen_sentence = false;
    s_seen_valid_fix = false;
    s_status.init_tick_ms = now_tick;
    s_status.fault = GNSS_MAXM10S_FAULT_NONE;
    set_state(GNSS_MAXM10S_STATE_INIT);

    GNSS_maxm10s_device_reset();
    GNSS_maxm10s_port_init(uart);
}

void GNSS_maxm10s_update(void)
{
    char line[GNSS_MAXM10S_LINE_MAX_LEN];
    GNSS_maxm10s_parsed_t parsed;
    uint32_t now_tick;
    bool got_valid_fix = false;

    while (GNSS_maxm10s_port_read_line(line, sizeof(line))) {
        now_tick = HAL_GetTick();
        s_seen_sentence = true;
        s_status.last_sentence_tick_ms = now_tick;

        GNSS_maxm10s_device_parse_line(line);
        if (GNSS_maxm10s_device_take_parsed(&parsed)) {
            if (update_fix_from_parsed(&parsed)) {
                got_valid_fix = true;
                s_status.last_valid_fix_tick_ms = now_tick;
            }
        }

        handle_validity_state(got_valid_fix);
    }

    now_tick = HAL_GetTick();
    update_runtime_state(now_tick);
}

bool GNSS_maxm10s_update_and_get_location(double *lat_deg, double *lon_deg)
{
    GNSS_maxm10s_update();
    return GNSS_maxm10s_get_location(lat_deg, lon_deg);
}

bool GNSS_maxm10s_get_fix(GNSS_maxm10s_fix_t *out)
{
    if ((out == NULL) || (!s_fix_ready)) {
        return false;
    }

    *out = s_fix;
    s_fix_ready = false;
    return true;
}

bool GNSS_maxm10s_get_latest_fix(GNSS_maxm10s_fix_t *out)
{
    if ((out == NULL) || (!s_last_valid_fix_ready)) {
        return false;
    }

    *out = s_last_valid_fix;
    return true;
}

bool GNSS_maxm10s_get_location(double *lat_deg, double *lon_deg)
{
    const GNSS_maxm10s_fix_t *fix = get_latest_valid_fix_ptr();

    if ((fix == NULL) || (lat_deg == NULL) || (lon_deg == NULL)) {
        return false;
    }

    *lat_deg = fix->lat_deg;
    *lon_deg = fix->lon_deg;
    return true;
}

bool GNSS_maxm10s_get_altitude(double *alt_m)
{
    const GNSS_maxm10s_fix_t *fix = get_latest_valid_fix_ptr();

    if ((fix == NULL) || (alt_m == NULL)) {
        return false;
    }

    *alt_m = fix->alt_m;
    return true;
}

bool GNSS_maxm10s_get_speed_kmh(float *speed_kmh)
{
    const GNSS_maxm10s_fix_t *fix = get_latest_valid_fix_ptr();

    if ((fix == NULL) || (speed_kmh == NULL)) {
        return false;
    }

    *speed_kmh = fix->speed_kmh;
    return true;
}

bool GNSS_maxm10s_get_heading(float *heading_deg)
{
    const GNSS_maxm10s_fix_t *fix = get_latest_valid_fix_ptr();

    if ((fix == NULL) || (heading_deg == NULL)) {
        return false;
    }

    *heading_deg = fix->track_deg;
    return true;
}

bool GNSS_maxm10s_get_local_time(GNSS_maxm10s_time_t *out)
{
    const GNSS_maxm10s_fix_t *fix = get_latest_valid_fix_ptr();

    if (fix == NULL) {
        return false;
    }

    return convert_utc_to_local_time(fix, out);
}

bool GNSS_maxm10s_is_valid(bool *out)
{
    const GNSS_maxm10s_fix_t *fix = get_latest_valid_fix_ptr();

    if ((fix == NULL) || (out == NULL)) {
        return false;
    }

    *out = fix->valid;
    return true;
}

bool GNSS_maxm10s_get_status(GNSS_maxm10s_status_t *out)
{
    if (out == NULL) {
        return false;
    }

    *out = s_status;
    return true;
}
