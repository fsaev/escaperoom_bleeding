#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "Honeywell_ABP.h"

void init_pressure_sensor(Honeywell_ABP* abp);
void tick_pressure_sensor(void);

float get_latest_pressure_reading(void);

#endif // PRESSURE_SENSOR_H