#include "GNSS_maxm10s_device.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static GNSS_maxm10s_parsed_t s_parsed;
static bool s_updated = false;

static float atof_safe(const char *s)
{
    return (s != NULL && *s != '\0') ? (float)atof(s) : NAN;
}

static double coord_to_deg(const char *val, char hemi, bool is_lat)
{
    double raw;
    int deg_digits;
    double factor;
    double deg;
    double minutes;
    double out;

    if ((val == NULL) || (*val == '\0')) {
        return NAN;
    }

    raw = atof(val);
    if (raw <= 0.0) {
        return NAN;
    }

    deg_digits = is_lat ? 2 : 3;
    factor = pow(10.0, floor(log10(raw)) - (deg_digits - 1));
    deg = floor(raw / factor);
    minutes = raw - deg * factor;
    out = deg + minutes / 60.0;

    if ((hemi == 'S') || (hemi == 'W')) {
        out = -out;
    }
    return out;
}

static void parse_time(const char *val)
{
    const char *dot;

    if ((val == NULL) || (strlen(val) < 6u)) {
        return;
    }

    s_parsed.utc.hour = (uint8_t)((val[0] - '0') * 10 + (val[1] - '0'));
    s_parsed.utc.minute = (uint8_t)((val[2] - '0') * 10 + (val[3] - '0'));
    s_parsed.utc.second = (uint8_t)((val[4] - '0') * 10 + (val[5] - '0'));

    dot = strchr(val, '.');
    s_parsed.utc.msec = (dot != NULL) ? (uint16_t)(atof(dot) * 1000.0) : 0u;
    s_parsed.utc.have_time = true;
}

static void parse_date(const char *val)
{
    uint16_t year;

    if ((val == NULL) || (strlen(val) < 6u)) {
        return;
    }

    s_parsed.utc.day = (uint8_t)((val[0] - '0') * 10 + (val[1] - '0'));
    s_parsed.utc.month = (uint8_t)((val[2] - '0') * 10 + (val[3] - '0'));
    year = (uint16_t)((val[4] - '0') * 10 + (val[5] - '0'));
    s_parsed.utc.year = (year >= 80u) ? (uint16_t)(1900u + year) : (uint16_t)(2000u + year);
    s_parsed.utc.have_date = true;
}

static uint8_t split_fields(char *line, char **out, uint8_t max_fields)
{
    uint8_t count = 0;
    char *p = line;

    while ((count < max_fields) && (p != NULL)) {
        char *comma;
        out[count++] = p;
        comma = strchr(p, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    return count;
}

static bool checksum_ok(const char *line)
{
    const char *star;
    uint8_t sum = 0;
    uint8_t msg;
    const char *p;

    star = strchr(line, '*');
    if (star == NULL) {
        return false;
    }

    for (p = line + 1; p < star; ++p) {
        sum ^= (uint8_t)(*p);
    }

    msg = (uint8_t)strtoul(star + 1, NULL, 16);
    return sum == msg;
}

static void handle_rmc(char **f, uint8_t n)
{
    if (n < 10u) {
        return;
    }

    parse_time(f[1]);
    s_parsed.status_ok = (f[2][0] == 'A');
    s_parsed.lat_deg = coord_to_deg(f[3], f[4][0], true);
    s_parsed.lon_deg = coord_to_deg(f[5], f[6][0], false);
    s_parsed.have_coords = true;
    s_parsed.speed_kn = atof_safe(f[7]);
    s_parsed.speed_kmh = s_parsed.speed_kn * 1.852f;
    s_parsed.track_deg = atof_safe(f[8]);
    parse_date(f[9]);
    s_updated = true;
}

static void handle_gga(char **f, uint8_t n)
{
    if (n < 10u) {
        return;
    }

    s_parsed.lat_deg = coord_to_deg(f[2], f[3][0], true);
    s_parsed.lon_deg = coord_to_deg(f[4], f[5][0], false);
    s_parsed.have_coords = true;
    s_parsed.fix_type = (uint8_t)atoi(f[6]);
    s_parsed.sats = (uint8_t)atoi(f[7]);
    s_parsed.hdop = atof_safe(f[8]);
    s_parsed.alt_m = atof_safe(f[9]);
    s_parsed.have_quality = true;
    s_updated = true;
}

static void handle_gsa(char **f, uint8_t n)
{
    if (n < 17u) {
        return;
    }

    s_parsed.fix_type = (uint8_t)atoi(f[2]);
    s_parsed.pdop = atof_safe(f[14]);
    s_parsed.hdop = atof_safe(f[15]);
    s_parsed.have_quality = true;
    s_updated = true;
}

static void handle_vtg(char **f, uint8_t n)
{
    if (n < 8u) {
        return;
    }

    s_parsed.track_deg = atof_safe(f[1]);
    s_parsed.speed_kn = atof_safe(f[5]);
    s_parsed.speed_kmh = atof_safe(f[7]);
    s_parsed.have_speed = true;
    s_updated = true;
}

static void handle_gll(char **f, uint8_t n)
{
    if (n < 7u) {
        return;
    }

    s_parsed.lat_deg = coord_to_deg(f[1], f[2][0], true);
    s_parsed.lon_deg = coord_to_deg(f[3], f[4][0], false);
    s_parsed.have_coords = true;
    parse_time(f[5]);
    s_parsed.status_ok = (f[6][0] == 'A');
    s_updated = true;
}

void GNSS_maxm10s_device_reset(void)
{
    memset(&s_parsed, 0, sizeof(s_parsed));
    s_updated = false;
}

void GNSS_maxm10s_device_parse_line(char *line)
{
    char *star;
    char *fields[32] = {0};
    uint8_t count;

    if ((line == NULL) || (line[0] != '$')) {
        return;
    }
    if (!checksum_ok(line)) {
        return;
    }

    star = strchr(line, '*');
    if (star != NULL) {
        *star = '\0';
    }

    count = split_fields(line, fields, 32);
    if (count == 0u) {
        return;
    }

    if ((strcmp(fields[0], "$GNRMC") == 0) || (strcmp(fields[0], "$GPRMC") == 0)) {
        handle_rmc(fields, count);
    } else if ((strcmp(fields[0], "$GNGGA") == 0) || (strcmp(fields[0], "$GPGGA") == 0)) {
        handle_gga(fields, count);
    } else if ((strcmp(fields[0], "$GNGSA") == 0) || (strcmp(fields[0], "$GPGSA") == 0)) {
        handle_gsa(fields, count);
    } else if ((strcmp(fields[0], "$GNVTG") == 0) || (strcmp(fields[0], "$GPVTG") == 0)) {
        handle_vtg(fields, count);
    } else if ((strcmp(fields[0], "$GNGLL") == 0) || (strcmp(fields[0], "$GPGLL") == 0)) {
        handle_gll(fields, count);
    }
}

bool GNSS_maxm10s_device_take_parsed(GNSS_maxm10s_parsed_t *out)
{
    if ((out == NULL) || (!s_updated)) {
        return false;
    }
    *out = s_parsed;
    s_updated = false;
    return true;
}
