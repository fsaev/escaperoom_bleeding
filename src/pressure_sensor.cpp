#include "pressure_sensor.h"

static Honeywell_ABP* linked_instance_abp;

void init_pressure_sensor(Honeywell_ABP* abp) {
    linked_instance_abp = abp;

    Wire.begin();
}

void tick_pressure_sensor(void) {
  linked_instance_abp->update();
}

float get_latest_pressure_reading(void) {
    return linked_instance_abp->pressure();
}