#include "RunningMedian.h"

RunningMedian samples = RunningMedian(1000);

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 34;

// variable for storing the potentiometer value
float potValue = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  potValue = analogRead(potPin) / 1.01337737732 * 4.12438 / 1240.909;
  samples.add(potValue);
  float m = samples.getMedian();
  Serial.println(m);
  delay(1);
}
