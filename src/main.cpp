#include <Arduino.h>
#include "defines.h"
#include "pump_control.h"
#include "pressure_sensor.h"

Honeywell_ABP abp(
    0x08,   // I2C address
    0,      // minimum pressure
    2068,      // maximum pressure
    "mbar"   // pressure unit
);

static uint32_t subroutine_timestamp = 0;

void setup() {
  
  Serial.begin(9600);
  pinMode(LEG_OBSTRUCTION_PIN, INPUT_PULLUP);
  pinMode(TASK_COMPLETE_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  init_pressure_sensor(&abp);
  init_pump_control();
  subroutine_timestamp = millis();
}

void loop() {
  if(millis() - subroutine_timestamp >= 5) {
    subroutine_timestamp = millis();
    tick_pressure_sensor();
    tick_pump_control();
  }
}