#include <Arduino.h>
#include "Honeywell_ABP.h"

#define Pump 3
// #define LED_G 4
// #define LED_Y 5
// #define LED_R 6
#define SwitchPreventingBleeding 7
// #define StartSignal 5
// #define OutputSignalStoppedBledding 4
#define OutputSignalTaskComplete 6 // grey wire

unsigned long currentTime;
unsigned long refTime;
unsigned long printTime;
unsigned long RedBlinkTime;
unsigned long low_time;
float Pressure_limit = 100;
float Pressure;
enum STATE {INIT, Low_flow, High_flow}; // Enumerator for system states
int coag_time = 10000; // need to stop the bleeding for 10 seconds for the pump to stop
STATE current_state{INIT};
bool newSession = true;

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

// Function for printing pressure value every 0.25 sec

void PrintVal() {
  if (millis() - printTime > 250)
  {
    // Serial.println(abp.pressure());
    printTime = millis();
  }

}

void setup() {
  Serial.begin(9600);
  // pinMode(LED_G, OUTPUT);
  // pinMode(LED_Y, OUTPUT);
  // pinMode(LED_R, OUTPUT);
  pinMode(SwitchPreventingBleeding, INPUT_PULLUP);
  // pinMode(StartSignal, INPUT);
  pinMode(OutputSignalTaskComplete, OUTPUT);
  // pinMode(OutputSignalStoppedBledding, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //timestamps
  currentTime = millis();
  printTime = millis();

  STATE current_state{INIT};

  // open I2C communication
  Wire.begin();
  Serial.println("Ready");
  pinMode(Pump, OUTPUT);

  //Changing the frequency to remove high pitch sound from the pump
  TCB1.CTRLA &= ~TCB_ENABLE_bm;// Turn off timer while we change parameters.
  TCB1.CTRLA &= ~TCB_CLKSEL_gm;// Clear all CLKSEL bits.
  TCB1.CTRLA |= TCB_CLKSEL_CLKDIV1_gc;// Set prescaler to 2.
  TCB1.CTRLA |= TCB_ENABLE_bm;// Re-enable timer. Pins 5 and 9 now run at 31.25 kHz
  
  // PWM signal to run the pump
  analogWrite(Pump, 43);

  digitalWrite(OutputSignalTaskComplete, 0); 
  // digitalWrite(OutputSignalStoppedBledding, 0); 
}



void loop() {
  // update sensor reading
  abp.update();
  analogWrite(Pump, 43);
  // PrintVal();
  
  //State switching
  switch(current_state)
  {
    case (INIT):
      analogWrite(Pump, 0);
      Serial.println("Start?");

      //Criteria for starting the bleeding

      if (digitalRead(SwitchPreventingBleeding) == true && newSession == false) {
        // the leg should have been placed back in place
        Serial.println("Session has been reset");
        newSession = true;
        delay(1000); // wait for 1 seconds 
        }

      if (digitalRead(SwitchPreventingBleeding) == false && newSession == true) 
      {
        newSession = false; // the session is consumed
        current_state = High_flow;
        Serial.println("Bleeding started!");
        digitalWrite(OutputSignalTaskComplete, 1); // send 1 to notify the bleeding is running
        // digitalWrite(OutputSignalStoppedBledding, 0); // erase last config in case it's the second time the bleeding is triggered
        digitalWrite(LED_BUILTIN, 1);
        break;
      }
    break;
    
    case (Low_flow):
      //Serial.println("Low flow");
      // LED_pressureOK();

      Serial.println(low_time);

      if(millis() - low_time >= coag_time)
      {
        // LED_bleeding_Stopped();
        analogWrite(Pump, 0);
        digitalWrite(OutputSignalTaskComplete, 0); // send 1 to notify the task is complete
        // digitalWrite(OutputSignalStoppedBledding, 1); // send 1 to notify the bleeding has been correctly stopped
        digitalWrite(LED_BUILTIN, 0);
        Serial.println("Bleeding stopped!");
        
        delay(5000); // wait 5 seconds when bleeding end detected
        current_state = INIT;
        break;
      }
      if (abp.pressure() < Pressure_limit)
      {
        current_state = High_flow;
        break;
      }
      break;
      
    case (High_flow):
      //Serial.println("High flow");
      // LED_bleeding();
      if (abp.pressure() > Pressure_limit)
      {
        Serial.println("Correct pressure applied, hold on!");
        low_time = millis();
        current_state = Low_flow;
        break;
      }
      break;
  }

  delay(100);
}