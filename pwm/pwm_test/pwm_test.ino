// the number of the LED pin
const int ledPin = 25;  // GPIO

// setting PWM properties
const int freq = 1000; 
const int ledChannel = 1; 
const int resolution = 10; // Select resolution 1 - 16 bits
void setup(){
  // configure LED PWM functionalitites
  // pinMode(ledPin, OUTPUT);
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 500);
}
 
void loop(){
  // // increase the LED brightness
  // for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
  //   // changing the LED brightness with PWM
  //   ledcWrite(ledChannel, dutyCycle);
  //   delay(4);
  // }

  // // decrease the LED brightness
  // for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
  //   // changing the LED brightness with PWM
  //   ledcWrite(ledChannel, dutyCycle);   
  //   delay(4);
  // }
}
