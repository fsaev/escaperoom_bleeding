#include <Arduino.h>
#include "Honeywell_ABP.h"

#define Pump 3
#define LED_G 4
#define LED_Y 5
#define LED_R 6
#define StartPushButton 7

unsigned long currentTime;
unsigned long refTime;
unsigned long printTime;
unsigned long RedBlinkTime;
int low_time;
int last_time;
float Pressure_limit = 100;
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

//LED signals

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

// LED signal functions

void LED_R_Blink() 
{
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

void LED_bleeding() 
{
  LED_G_OFF();
  LED_Y_OFF();
  LED_R_Blink();
}

void LED_pressureOK() 
{
  LED_G_OFF();
  LED_Y_ON();
  LED_R_OFF();
}

void LED_bleeding_Stopped() 
{
  LED_G_ON();
  LED_Y_OFF();
  LED_R_OFF();
}


// Function for printing pressure value every 0.25 sec

void PrintVal() {
  if (millis() - printTime > 250)
  {
    Serial.println(abp.pressure());
    printTime = millis();
  }

}

void setup() {
  Serial.begin(9600);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(StartPushButton, INPUT_PULLUP);

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
}



void loop() {
  // update sensor reading
  abp.update();
  analogWrite(Pump, 43);
  PrintVal();
  
  //State switching
  switch(current_state)
  {
    case (INIT):
    analogWrite(Pump, 0);
    Serial.println("Start?");
    //Criteria for starting the bleeding
    if (digitalRead(StartPushButton) == false)
    {
      current_state = High_flow;
      last_time = millis(); 
      break;
    }
    break;
    
    case (Low_flow):
      //Serial.println("Low flow");
      LED_pressureOK();

      if(millis() - low_time >= coag_time)
      {
        LED_bleeding_Stopped();
        analogWrite(Pump, 0);
        //Inf loop for stopping the code from running
        while(true);
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
      LED_bleeding();
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