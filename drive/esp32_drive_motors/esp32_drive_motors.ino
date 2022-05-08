/*********
  Rui Santos - Serial communication between ESP32 and Raspberry Pi 4
  Complete project details at https://randomnerdtutorials.com
*********/

#include <ArduinoJson.h>

// Config pin parameters
// Motor 1
const uint8_t AIN1 = 27;
const uint8_t AIN2 = 26;
const uint8_t PWMA = 25;
const uint8_t ENC1_A = 22;
const uint8_t ENC1_B = 23;

// Motor 2
const uint8_t BIN1 = 19;
const uint8_t BIN2 = 18;
const uint8_t PWMB = 5;
const uint8_t ENC2_A = 17;
const uint8_t ENC2_B = 16;

const uint8_t STBY = 21;


// Config pwm parameters
const int FREQ_PWM_1 = 1000;
const int RESOLUTION_PWM_1 = 10; // bits (1 - 16 bits)
const uint8_t CHANNEL_PWMA = 1;

const int FREQ_PWM_2 = 1000;
const int RESOLUTION_PWM_2 = 10; // bits (1 - 16 bits)
const uint8_t CHANNEL_PWMB = 2;


// Config serial parameters
const int BAUD_RATE = 115200;
// Sending
StaticJsonDocument<200> JSON_DOC_SEND;
const unsigned int SENDING_FREQUENCY = 2000; // Hz
double PERIOD; // milliseconds
volatile unsigned long long timerPivot = 0; // milliseconds

// Receiving
volatile char serialChar;
String serialLine = "";
// Allocate the JSON document
// Inside the brackets, 200 is the capacity of the memory pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/v6/assistant to compute the capacity.
StaticJsonDocument<200> JSON_DOC_RECEIVE;
String KEY[15] = "motor_data";
volatile int wheel_direction[2] = {0, 0};
volatile int pwm_frequency[2] = {0, 0};
volatile int pwm_pulse[2] = {0, 0};


// Config encoders' parameters
volatile int POS_1 = 0;
volatile int POS_2 = 0;

bool deserializeJSON() {
  DeserializationError error = deserializeJson(JSON_DOC_RECEIVE, serialLine);

  // Test if parsing succeeds.
  if (error) {
    //    Serial.print(F("deserializeJson() failed: "));
    //    Serial.println(error.f_str());
    return false;
  }
  return true;
}

void decodeJSON() {
  wheel_direction[0] = JSON_DOC_RECEIVE["motor_data"][0];
  pwm_frequency[0] = JSON_DOC_RECEIVE["motor_data"][1];
  pwm_pulse[0] = JSON_DOC_RECEIVE["motor_data"][2];

  wheel_direction[1] = JSON_DOC_RECEIVE["motor_data"][3];
  pwm_frequency[1] = JSON_DOC_RECEIVE["motor_data"][4];
  pwm_pulse[1] = JSON_DOC_RECEIVE["motor_data"][5];
}

void driveLeftWheel() {
  if (wheel_direction[0] == 1) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (wheel_direction[0] == -1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else if (wheel_direction[0] == 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void driveRightWheel() {
  if (wheel_direction[1] == 1) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else if (wheel_direction[1] == -1) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (wheel_direction[1] == 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}

void drive_motors() {
  //  JSON form: {"motor_data": [DIRECTION_LEFT, PWM_FREQUENCY_LEFT, PWM_LEFT, DIRECTION_RIGHT, PWM_FREQUENCY_RIGHT, PWM_RIGHT]}
  //  DIRECTION_RIGHT / DIRECTION_LEFT: 1 is forward, 0 is stop, -1 is backward
  //  PWM_FREQUENCY_LEFT / PWM_FREQUENCY_RIGHT: 1000 - 100000 Hz
  //  PWM: 0 - 1023
  if (deserializeJSON()) {
    decodeJSON();
    digitalWrite(STBY, LOW);
    driveLeftWheel();
    driveRightWheel();
    digitalWrite(STBY, HIGH);

    ledcSetup(CHANNEL_PWMA, pwm_frequency[0], RESOLUTION_PWM_1);
    ledcSetup(CHANNEL_PWMB, pwm_frequency[1], RESOLUTION_PWM_2);
    ledcWrite(CHANNEL_PWMA, pwm_pulse[0]);
    ledcWrite(CHANNEL_PWMB, pwm_pulse[1]);
  }
}

void serial_receive() {
  serialChar = Serial.read();
  if (serialChar == '\n' || serialChar == '#') {
    drive_motors();
    serialLine = "";
  } else {
    serialLine += serialChar;
  }
}


void ICACHE_RAM_ATTR readEncoder_1()
{
  int enc1_b = digitalRead(ENC1_B);
  if (enc1_b > 0)
  {
    POS_1++;
  }
  else
  {
    POS_1--;
  }
}

void ICACHE_RAM_ATTR readEncoder_2()
{
  int enc2_b = digitalRead(ENC2_B);
  if (enc2_b > 0)
  {
    POS_2--;
  }
  else
  {
    POS_2++;
  }
}

void initializeMotor()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);
}

void readEncoderTicks() {
  noInterrupts();
  JSON_DOC_SEND["left_tick"] = POS_1;
  JSON_DOC_SEND["right_tick"] = POS_2;
  interrupts();
}

void calculateSendingPeriod() {
  if (SENDING_FREQUENCY > 0) {
    PERIOD = (1 / SENDING_FREQUENCY) * 1e6;
  } else {
    PERIOD = 1e6;
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);

  // Double check Sending Frequency
  calculateSendingPeriod();

  // Setup pinmode
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder_1, RISING);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), readEncoder_2, RISING);

  pinMode(STBY, OUTPUT);

  // Setup PWM
  ledcSetup(CHANNEL_PWMA, FREQ_PWM_1, RESOLUTION_PWM_1);
  ledcSetup(CHANNEL_PWMB, FREQ_PWM_2, RESOLUTION_PWM_2);

  ledcAttachPin(PWMA, CHANNEL_PWMA);
  ledcAttachPin(PWMB, CHANNEL_PWMB);

  initializeMotor();

  ledcWrite(CHANNEL_PWMA, 0);
  ledcWrite(CHANNEL_PWMB, 0);
}

void loop()
{
  while (Serial.available()) {
    serial_receive();
  }

  readEncoderTicks();
  if (micros() - timerPivot >= PERIOD) {
    serializeJson(JSON_DOC_SEND, Serial);
    Serial.println();
    //  serializeJsonPretty(JSON_DOC_SEND, Serial);
    timerPivot = micros();
  }
}
