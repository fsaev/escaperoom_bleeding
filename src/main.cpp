#include <Arduino.h>
#include "Honeywell_ABP.h"

#define LED 4
#define Pump 3
unsigned long currentTime;
unsigned long refTime;
int low_time;
int last_time;
float Pressure_limit = 150;
float Pressure;
enum STATE {INIT, Low_flow, High_flow}; // Enumerator for system states
int coag_time = 15000;
STATE current_state{INIT};

Honeywell_ABP abp(
    0x08,   // I2C address
    0,      // minimum pressure
    2068,      // maximum pressure
    "mbar"   // pressure unit
);

float Pressure_print() {
  currentTime = millis();
  refTime = currentTime;
  if(currentTime - refTime >= 1000)
  {
    Pressure = abp.pressure();
    refTime = currentTime;
  }
  return Pressure;
}

void setup() {
   pinMode(LED, OUTPUT);
   Serial.begin(9600);
   currentTime = millis();
   STATE current_state{INIT};
   // open I2C communication
   Wire.begin();
   Serial.println("Ready");
   pinMode(Pump, OUTPUT);
   //TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;// Turn off timer while we change parameters.
   //TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_CLKSEL_gm;// Clear all CLKSEL bits.
   //TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV2_gc;// Set prescaler to 2.
   //TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;// Re-enable timer. Pins 5 and 9 now run at 31.25 kHz.
   TCB1.CTRLA &= ~TCB_ENABLE_bm;// Turn off timer while we change parameters.
   TCB1.CTRLA &= ~TCB_CLKSEL_gm;// Clear all CLKSEL bits.
   TCB1.CTRLA |= TCB_CLKSEL_CLKDIV1_gc;// Set prescaler to 2.
   TCB1.CTRLA |= TCB_ENABLE_bm;// Re-enable timer. Pins 5 and 9 now run at 31.25 kHz

   analogWrite(Pump, 43);
}


void loop() {
  // update sensor reading
  abp.update();
  //Serial.println(abp.pressure());
  //Serial.println(Pressure_print());
  switch(current_state)
  {
    case (INIT):
    Serial.println("INIT");
    if (abp.pressure() < Pressure_limit)
    {
      current_state = High_flow;
      last_time = millis(); 
      break;
    }
    break;
    
    case (Low_flow):
      //Serial.println("Low flow");
      Serial.println("--");
      if(millis() - low_time >= coag_time)
      {
        Serial.println(abp.pressure());
        digitalWrite(LED, HIGH);
        analogWrite(Pump, 0);
      }
      if (abp.pressure() < Pressure_limit)
      {
        last_time = millis();
        current_state = High_flow;
        break;
      }
      break;
      
    case (High_flow):
      //Serial.println("High flow");
      Serial.println("*");
      if (abp.pressure() > Pressure_limit)
      {
        last_time = millis();
        low_time = millis();
        current_state = Low_flow;
        break;
      }
      break;
  }
}