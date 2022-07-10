#include "RunningMedian.h"

float VOLTAGE_DIVIDER_FACTOR = 4.1166610794;
float VOLTAGE_TO_ANALOG_FACTOR = 1.0/ 1240.909;
float CALIBRATION_FACTOR = 1.0024788723;

RunningMedian samples = RunningMedian(1000);

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 34;

// variable for storing the potentiometer value
float potValue = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
}

void loop() {
  // Reading potentiometer value
  potValue = analogRead(potPin) * VOLTAGE_DIVIDER_FACTOR * VOLTAGE_TO_ANALOG_FACTOR * CALIBRATION_FACTOR;
  samples.add(potValue);
  float m = samples.getAverage(100);
  Serial.println(m);
  delay(1);
}
