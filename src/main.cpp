#include <Arduino.h>
#include "Honeywell_ABP.h"

#define Pump 3
#define LED_G 4
#define LED_Y 5
#define LED_R 6
unsigned long currentTime;
unsigned long refTime;
unsigned long printTime;
unsigned long RedBlinkTime;
int low_time;
int last_time;
float Pressure_limit = 150;
float Pressure;
enum STATE {INIT, Low_flow, High_flow}; // Enumerator for system states
int coag_time = 10000;
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
  RedBlinkTime = currentTime;
  if(currentTime - refTime >= 1000)
  {
    Pressure = abp.pressure();
    refTime = currentTime;
  }
  return Pressure;
}

void LED_G_ON() {
  digitalWrite(LED_G, HIGH);
}

void LED_G_OFF() {
  digitalWrite(LED_G, LOW);
}

void LED_Y_ON() {
  digitalWrite(LED_Y, HIGH);
}

void LED_Y_OFF() {
  digitalWrite(LED_Y, LOW);
}

void LED_R_ON() {
  digitalWrite(LED_R, HIGH);
}

void LED_R_OFF() {
  digitalWrite(LED_R, LOW);
}

void LED_R_Blink() {
  LED_R_OFF();
  if(millis() - RedBlinkTime > 250)
  {
    LED_R_ON();
    if(millis() - RedBlinkTime > 500)
    {
      LED_R_OFF();
      RedBlinkTime = millis();
    }
  }
}

void PrintVal() {
  if (millis() - printTime > 250)
  {
    Serial.println(abp.pressure());
    printTime = millis();
  }

}

void setup() {
   pinMode(LED_G, OUTPUT);
   pinMode(LED_Y, OUTPUT);
   pinMode(LED_R, OUTPUT);
   Serial.begin(9600);
   currentTime = millis();
   printTime = millis();
   STATE current_state{INIT};
   // open I2C communication
   Wire.begin();
   Serial.println("Ready");
   pinMode(Pump, OUTPUT);
   TCB1.CTRLA &= ~TCB_ENABLE_bm;// Turn off timer while we change parameters.
   TCB1.CTRLA &= ~TCB_CLKSEL_gm;// Clear all CLKSEL bits.
   TCB1.CTRLA |= TCB_CLKSEL_CLKDIV1_gc;// Set prescaler to 2.
   TCB1.CTRLA |= TCB_ENABLE_bm;// Re-enable timer. Pins 5 and 9 now run at 31.25 kHz

   analogWrite(Pump, 45);
}


void loop() {
  // update sensor reading
  abp.update();
  PrintVal();
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
      LED_G_OFF();
      LED_Y_ON();
      LED_R_OFF();
      if(millis() - low_time >= coag_time)
      {
        Serial.println(abp.pressure());
        LED_G_ON();
        LED_Y_OFF();
        LED_R_OFF();
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
      LED_G_OFF();
      LED_Y_OFF();
      LED_R_Blink();
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