/*********
  Rui Santos - Serial communication between ESP32 and Raspberry Pi 4
  Complete project details at https://randomnerdtutorials.com
*********/

#include <ESP32Encoder.h>
#include <ArduinoJson.h>
#include <MD5.h>

class RPMCalculator {
  // Declare private variable of class
private:
  float _sample_time = 0.005;
  float _encoder_tick_per_round = 480;

  float _RPM_Filter = 0;
  float _previous_RPM = 0;

  float _RPM_Filter_coefficient = 0.854;
  float _RPM_coefficient = 0.0728;
  float _previous_RPM_coefficient = 0.0728;

  int32_t _current_tick = 0;
  int32_t _previous_tick = 0;
  long _previous_T = 0;

public:
  void setupSampleTime(float sample_time) {
    this->_sample_time = sample_time;
  }

  void setupEncoderTickPerRound(unsigned int encoder_tick_per_round) {
    this->_encoder_tick_per_round = encoder_tick_per_round;
  }

  void setupLowPassFilter(float RPM_Filter_coefficient, float RPM_coefficient, float previous_RPM_coefficient) {
    this->_RPM_Filter_coefficient = RPM_Filter_coefficient;
    this->_RPM_coefficient = RPM_coefficient;
    this->_previous_RPM_coefficient = previous_RPM_coefficient;
  }

  void calculate(int32_t current_tick) {
    long curr_T = micros();
    if (((float)(curr_T - this->_previous_T)) / 1.0e6 >= this->_sample_time) {
      float delta_T = ((float)(curr_T - this->_previous_T)) / 1.0e6;
      this->_current_tick = current_tick;
      float encoder_tick_per_sec = abs(this->_current_tick - this->_previous_tick) / delta_T;
      float RPM = encoder_tick_per_sec / this->_encoder_tick_per_round * 60.0;

      // Low-pass filter (over 25Hz cut off)
      this->_RPM_Filter = this->_RPM_Filter_coefficient * this->_RPM_Filter + RPM * this->_RPM_coefficient + this->_previous_RPM * this->_previous_RPM_coefficient;

      this->_previous_RPM = this->_RPM_Filter;
      this->_previous_tick = current_tick;
      this->_previous_T = curr_T;
    }
  }

  float getRPM() {
    return this->_RPM_Filter;
  }

  float getTick() {
    return this->_current_tick;
  }
};

struct MotorDataSend {
  int32_t left_tick;
  int32_t right_tick;
  float left_RPM;
  float right_RPM;
  String checksum;
};

struct MotorDataReceive {
  int left_wheel_direction;
  int left_pwm_frequency;
  int left_pwm_pulse;

  int right_wheel_direction;
  int right_pwm_frequency;
  int right_pwm_pulse;
};


const int RUNNING_TIME = 10000;  // ms

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
const int RESOLUTION_PWM_1 = 10;  // bits (1 - 16 bits)
const uint8_t CHANNEL_PWMA = 1;

const int FREQ_PWM_2 = 1000;
const int RESOLUTION_PWM_2 = 10;  // bits (1 - 16 bits)
const uint8_t CHANNEL_PWMB = 2;


// Config serial parameters
const int BAUD_RATE = 921600;
// Sending
const unsigned int SENDING_FREQUENCY = 500;  // Hz
double PERIOD;                               // milliseconds
volatile unsigned long long timerPivot = 0;  // milliseconds

// Receiving
volatile char serialChar;
String serialLine = "";
// Allocate the JSON document
// Inside the brackets, 200 is the capacity of the memory pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/v6/assistant to compute the capacity.
bool is_received = false;
String KEY[15] = "motor_data";

// Encoder instances
ESP32Encoder encoder_1;
ESP32Encoder encoder_2;

// RPMCalculator instances
RPMCalculator rpm_calculator_1;
RPMCalculator rpm_calculator_2;

// Struct
MotorDataSend motor_data_send;
MotorDataReceive motor_data_receive;

// Timer
long read_timer = 0;


void deserializeJSON() {
  StaticJsonDocument<200> JSON_DOC_RECEIVE;
  DeserializationError error = deserializeJson(JSON_DOC_RECEIVE, serialLine);

  // Test if parsing succeeds.
  if (error) {
    //    Serial.print(F("deserializeJson() failed: "));
    //    Serial.println(error.f_str());
    is_received = false;
    return;
  }

  // Assign data of JSON_DOC_RECEIVE to MotorDataReceive struct
  motor_data_receive.left_wheel_direction = JSON_DOC_RECEIVE["motor_data"][0];
  motor_data_receive.left_pwm_frequency = JSON_DOC_RECEIVE["motor_data"][1];
  motor_data_receive.left_pwm_pulse = JSON_DOC_RECEIVE["motor_data"][2];

  motor_data_receive.right_wheel_direction = JSON_DOC_RECEIVE["motor_data"][3];
  motor_data_receive.right_pwm_frequency = JSON_DOC_RECEIVE["motor_data"][4];
  motor_data_receive.right_pwm_pulse = JSON_DOC_RECEIVE["motor_data"][5];

  is_received = true;
}

void driveLeftWheel() {
  if (motor_data_receive.left_wheel_direction == 1) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }

  else if (motor_data_receive.left_wheel_direction == -1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  else if (motor_data_receive.left_wheel_direction == 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void driveRightWheel() {
  if (motor_data_receive.right_wheel_direction == 1) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  else if (motor_data_receive.right_wheel_direction == -1) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  else if (motor_data_receive.right_wheel_direction == 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}

void drive_motors() {
  //  JSON form: {"motor_data": [DIRECTION_LEFT, PWM_FREQUENCY_LEFT, PWM_LEFT, DIRECTION_RIGHT, PWM_FREQUENCY_RIGHT, PWM_RIGHT]}
  //  DIRECTION_RIGHT / DIRECTION_LEFT: 1 is forward, 0 is stop, -1 is backward
  //  PWM_FREQUENCY_LEFT / PWM_FREQUENCY_RIGHT: 1000 - 100000 Hz
  //  PWM: 0 - 1023
  deserializeJSON();

  if (is_received) {
    driveLeftWheel();
    driveRightWheel();

    ledcSetup(CHANNEL_PWMA, motor_data_receive.left_pwm_frequency, RESOLUTION_PWM_1);
    ledcSetup(CHANNEL_PWMB, motor_data_receive.right_pwm_frequency, RESOLUTION_PWM_2);

    digitalWrite(STBY, LOW);
    ledcWrite(CHANNEL_PWMA, motor_data_receive.left_pwm_pulse);
    ledcWrite(CHANNEL_PWMB, motor_data_receive.right_pwm_pulse);
    digitalWrite(STBY, HIGH);
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

void initializeMotor() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(STBY, HIGH);
}

void readRPM() {
  long start = micros();

  char* md5str;
  unsigned char* hash;


  motor_data_send.left_RPM = rpm_calculator_1.getRPM();
  motor_data_send.left_tick = rpm_calculator_1.getTick();

  motor_data_send.right_RPM = rpm_calculator_2.getRPM();
  motor_data_send.right_tick = rpm_calculator_2.getTick();

  if (motor_data_send.left_RPM < 1) {
    motor_data_send.left_RPM = 0;
  }
  if (motor_data_send.right_RPM < 1) {
    motor_data_send.right_RPM = 0;
  }

  // calculate checksum
  char buf[200];
  StaticJsonDocument<200> JSON_DOC_CHECK;
  JSON_DOC_CHECK["lt"] = motor_data_send.left_tick;
  JSON_DOC_CHECK["rt"] = motor_data_send.right_tick;
  JSON_DOC_CHECK["lR"] = motor_data_send.left_RPM;
  JSON_DOC_CHECK["rR"] = motor_data_send.right_RPM;
  serializeJson(JSON_DOC_CHECK, buf, 200);
  // Serial.println(buf);
  hash = MD5::make_hash(buf);
  md5str = MD5::make_digest(hash, 16);
  motor_data_send.checksum = String(md5str);

  free(hash);
  free(md5str);

  long end = micros();
  read_timer = end - start;
}

void calculateSendingPeriod() {
  if (SENDING_FREQUENCY > 0) {
    PERIOD = (1 / SENDING_FREQUENCY) * 1e6;
  } else {
    PERIOD = 1e6;
  }
}

void sendJSON() {
  StaticJsonDocument<200> JSON_DOC_SEND;

  // JSON_DOC_SEND["lt"] = motor_data_send.left_tick;
  // JSON_DOC_SEND["rt"] = motor_data_send.right_tick;
  // JSON_DOC_SEND["lR"] = motor_data_send.left_RPM;
  // JSON_DOC_SEND["rR"] = motor_data_send.right_RPM;
  // JSON_DOC_SEND["ck"] = motor_data_send.checksum;

  JSON_DOC_SEND["lt"] = 1000000;
  JSON_DOC_SEND["rt"] = 1000000;
  JSON_DOC_SEND["lR"] = 199.123456;
  JSON_DOC_SEND["rR"] = 199.123456;
  JSON_DOC_SEND["ck"] = "3931756a764b44099197a21ec5d67a57";

  serializeJson(JSON_DOC_SEND, Serial);
  Serial.println();
}

void setup() {
  Serial.begin(BAUD_RATE);

  // Enable the weak pull down resistors
  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  // Double check Sending Frequency
  calculateSendingPeriod();

  // Setup pinmode
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  encoder_1.attachSingleEdge(ENC1_A, ENC1_B);
  encoder_1.setFilter(1023);
  encoder_1.clearCount();
  // void attachHalfQuad(int aPintNumber, int bPinNumber);
  // void attachFullQuad(int aPintNumber, int bPinNumber);
  // void attachSingleEdge(int aPintNumber, int bPinNumber);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
  encoder_2.attachSingleEdge(ENC2_B, ENC2_A);
  encoder_2.setFilter(1023);
  encoder_2.clearCount();

  pinMode(STBY, OUTPUT);

  // Setup PWM
  ledcSetup(CHANNEL_PWMA, FREQ_PWM_1, RESOLUTION_PWM_1);
  ledcSetup(CHANNEL_PWMB, FREQ_PWM_2, RESOLUTION_PWM_2);

  ledcAttachPin(PWMA, CHANNEL_PWMA);
  ledcAttachPin(PWMB, CHANNEL_PWMB);

  initializeMotor();

  ledcWrite(CHANNEL_PWMA, 0);
  ledcWrite(CHANNEL_PWMB, 0);

  // set starting count value after attaching
  // encoder.setCount(37);

  // clear the encoder's raw count and set the tracked count to zero
  // encoder.clearCount();
  // encoder2.clearCount();
  // Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
}

void loop() {

  while (Serial.available()) {
    serial_receive();
  }

  rpm_calculator_1.calculate((int32_t)encoder_1.getCount());
  rpm_calculator_2.calculate((int32_t)encoder_2.getCount());

  // if (millis() < RUNNING_TIME){
  //   readRPM();
  // }

  // if (millis() >= RUNNING_TIME) {
  //   ledcWrite(CHANNEL_PWMA, 0);
  //   ledcWrite(CHANNEL_PWMB, 0);
  // }

  readRPM();

  if (micros() - timerPivot >= PERIOD) {
    // Serial.println("Encoder count = " + String((int32_t)encoder_1.getCount()) + " " + String((int32_t)encoder_2.getCount()));
    sendJSON();

    // Serial.print("left_tick: ");
    // Serial.println((int32_t)encoder_1.getCount());
    // Serial.print("right_tick: ");
    // Serial.println((int32_t)encoder_2.getCount());
    timerPivot = micros();
  }
}
