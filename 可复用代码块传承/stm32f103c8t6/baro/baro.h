#ifndef BARO_H
#define BARO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float temperature;
	float pressure;
	float altitude;
	uint32_t tick_ms;
} Baro_Data_t;

void Baro_Init(void);
void Baro_Run(void);
bool Baro_GetLatestData(Baro_Data_t *out_data);
bool Baro_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif // BARO_H