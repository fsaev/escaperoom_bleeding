#include <Arduino.h>
#include "pump_control.h"
#include "pressure_sensor.h"
#include "defines.h"

#define PRESSURE_THRESHOLD 80
#define COAGULATION_TIME 10000

static pump_control_data_t pump;
static uint32_t pump_control_timestamp = 0;

void init_pump_control() {
  //Changing the frequency to remove high pitch sound from the pump
  TCB1.CTRLA &= ~TCB_ENABLE_bm; // Turn off timer while we change parameters.
  TCB1.CTRLA &= ~TCB_CLKSEL_gm; // Clear all CLKSEL bits.
  TCB1.CTRLA |= TCB_CLKSEL_CLKDIV1_gc; // Set prescaler to 2.
  TCB1.CTRLA |= TCB_ENABLE_bm; // Re-enable timer. Pins 5 and 9 now run at 31.25 kHz

  pump.state = PUMP_INIT;
  pump.target_power = 0;
  pump.current_power = 0;
  pump.velocity = 100;

  analogWrite(PUMP_PIN, pump.current_power);
}

void tick_pump_control() {
  switch (pump.state){
    case PUMP_INIT:
      digitalWrite(TASK_COMPLETE_PIN, LOW);
      if(!digitalRead(LEG_OBSTRUCTION_PIN)) { //Obstacle open
        pump.state = PUMP_PRIME;
      }else{
        pump.state = PUMP_ARMED;
      }
      break;
    case PUMP_PRIME:
      digitalWrite(TASK_COMPLETE_PIN, LOW);
      pump.target_power = 22000; //Bånn gass
      if(digitalRead(LEG_OBSTRUCTION_PIN)) { //Closing obstacle
        pump.state = PUMP_ARMED;
      }
      break;
    case PUMP_ARMED:
      digitalWrite(TASK_COMPLETE_PIN, LOW);
      pump.target_power = 0; //Bånn gass
      if(!digitalRead(LEG_OBSTRUCTION_PIN)) { //Opening obstacle
        pump.state = PUMP_FLOW;
      }
      break;
    case PUMP_FLOW:
      digitalWrite(TASK_COMPLETE_PIN, LOW);
      pump.target_power = 11000; //Normal flow
      if(get_latest_pressure_reading() >= PRESSURE_THRESHOLD){ //If wound is packed
        pump.state = PUMP_HOLDING;
        pump_control_timestamp = millis();
      }
      break;
    case PUMP_HOLDING:
      digitalWrite(TASK_COMPLETE_PIN, LOW);
      pump.target_power = 11000; //Normal flow
      if(get_latest_pressure_reading() >= PRESSURE_THRESHOLD){ //If wound is packed
        if(millis() - pump_control_timestamp >= COAGULATION_TIME){ //If wound is packed for 10 seconds
          pump.state = PUMP_STOPPED;
        }
      }else{
        pump.state = PUMP_FLOW;
      }
      break;
    case PUMP_STOPPED:
      pump.target_power = 0; //Stop
      digitalWrite(TASK_COMPLETE_PIN, HIGH);
      if(digitalRead(LEG_OBSTRUCTION_PIN)) {
        pump.state = PUMP_ARMED;
      }
      break;
    default:
      break;
  }
  Serial.print(get_latest_pressure_reading());
  Serial.print(" "); 

  if(pump.target_power > pump.current_power){
    pump.current_power += pump.velocity;
  }else if(pump.target_power < pump.current_power){
    pump.current_power -= pump.velocity;
  }

  Serial.println(map(pump.current_power, 0, 65535, 0, 255));
  analogWrite(PUMP_PIN, map(pump.current_power, 0, 65535, 0, 255));
}