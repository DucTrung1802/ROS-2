/*********
  Rui Santos - Serial communication between ESP32 and Raspberry Pi 4
  Complete project details at https://randomnerdtutorials.com
*********/

#include "esp_adc_cal.h"
#include <ESP32Encoder.h>
#include <ArduinoJson.h>
#include <MD5.h>

#include "RunningMedian.h"

RunningMedian samples = RunningMedian(1000);

float VREF = 1100;
float REFERENCE_VOLTAGE = 3.072; 
float VOLTAGE_DIVIDER_COEFFICIENT = 3.99;
float VOLTAGE_DROP = 0.25;
float CALIBRATION = 1.031271; // Adjust for ultimate accuracy when input is measured using an accurate DVM, if reading too high then use e.g. 0.99, too low use 1.01

const int ADC_PIN = 34;
float potValue = 0;

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

    void calculate(int64_t current_tick) {
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

struct DataSend {
  int32_t left_tick;
  int32_t right_tick;
  float left_RPM;
  float right_RPM;
  float battery_voltage;
  String checksum;
};

struct DataReceive {
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
const double SENDING_FREQUENCY = 500;  // Hz
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
DataSend data_send;
DataReceive data_receive;

// Timer
long read_timer = 0;

float readVoltage(byte ADC_Pin) {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  VREF = adc_chars.vref; // Obtain the device ADC reference voltage
  return (analogRead(ADC_PIN) / 4095.0) * 3.3 * (1100 / VREF) * CALIBRATION;  // ESP by design reference voltage in mV
}

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
  data_receive.left_wheel_direction = JSON_DOC_RECEIVE["motor_data"][0];
  data_receive.left_pwm_frequency = JSON_DOC_RECEIVE["motor_data"][1];
  data_receive.left_pwm_pulse = JSON_DOC_RECEIVE["motor_data"][2];

  data_receive.right_wheel_direction = JSON_DOC_RECEIVE["motor_data"][3];
  data_receive.right_pwm_frequency = JSON_DOC_RECEIVE["motor_data"][4];
  data_receive.right_pwm_pulse = JSON_DOC_RECEIVE["motor_data"][5];

  is_received = true;
}

void driveLeftWheel() {
  if (data_receive.left_wheel_direction == 1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  else if (data_receive.left_wheel_direction == -1) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }

  else if (data_receive.left_wheel_direction == 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void driveRightWheel() {
  if (data_receive.right_wheel_direction == 1) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  else if (data_receive.right_wheel_direction == -1) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  else if (data_receive.right_wheel_direction == 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}

// void driveMotors() {
//  JSON form: {"motor_data": [DIRECTION_LEFT, PWM_FREQUENCY_LEFT, PWM_LEFT, DIRECTION_RIGHT, PWM_FREQUENCY_RIGHT, PWM_RIGHT]}
//  DIRECTION_RIGHT / DIRECTION_LEFT: 1 is forward, 0 is stop, -1 is backward
//  PWM_FREQUENCY_LEFT / PWM_FREQUENCY_RIGHT: 1000 - 100000 Hz
//  PWM: 0 - 1023

void driveMotors() {

  if (is_received) {
    driveLeftWheel();
    driveRightWheel();

    ledcSetup(CHANNEL_PWMA, data_receive.left_pwm_frequency, RESOLUTION_PWM_1);
    ledcSetup(CHANNEL_PWMB, data_receive.right_pwm_frequency, RESOLUTION_PWM_2);

    digitalWrite(STBY, LOW);
    ledcWrite(CHANNEL_PWMA, data_receive.left_pwm_pulse);
    ledcWrite(CHANNEL_PWMB, data_receive.right_pwm_pulse);
    digitalWrite(STBY, HIGH);
  }
}

void serialReceive() {
  serialChar = Serial.read();
  if (serialChar == '\n' || serialChar == '#') {
    deserializeJSON();
    driveMotors();
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

void packageData() {
  long start = micros();

  char* md5str;
  unsigned char* hash;

  data_send.left_RPM = rpm_calculator_1.getRPM();
  data_send.left_tick = rpm_calculator_1.getTick();

  data_send.right_RPM = rpm_calculator_2.getRPM();
  data_send.right_tick = rpm_calculator_2.getTick();

  float read_voltage = readVoltage(ADC_PIN);
  float adjusted_voltage = REFERENCE_VOLTAGE - (REFERENCE_VOLTAGE - read_voltage) * 3.99 / 1.668 + VOLTAGE_DROP;
  samples.add(adjusted_voltage);
  data_send.battery_voltage = samples.getAverage(10);

  if (data_send.left_RPM < 1) {
    data_send.left_RPM = 0;
  }
  if (data_send.right_RPM < 1) {
    data_send.right_RPM = 0;
  }

  // calculate checksum
  char buf[200];
  StaticJsonDocument<200> JSON_DOC_CHECK;
  JSON_DOC_CHECK["lt"] = data_send.left_tick;
  JSON_DOC_CHECK["rt"] = data_send.right_tick;
  JSON_DOC_CHECK["lR"] = data_send.left_RPM;
  JSON_DOC_CHECK["rR"] = data_send.right_RPM;
  JSON_DOC_CHECK["bv"] = data_send.battery_voltage;
  serializeJson(JSON_DOC_CHECK, buf, 200);
  // Serial.println(buf);
  hash = MD5::make_hash(buf);
  md5str = MD5::make_digest(hash, 16);
  data_send.checksum = String(md5str);

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

  JSON_DOC_SEND["lt"] = data_send.left_tick;
  JSON_DOC_SEND["rt"] = data_send.right_tick;
  JSON_DOC_SEND["lR"] = data_send.left_RPM;
  JSON_DOC_SEND["rR"] = data_send.right_RPM;
  JSON_DOC_SEND["bv"] = data_send.battery_voltage;
  JSON_DOC_SEND["ck"] = data_send.checksum;

  // JSON_DOC_SEND["lt"] = 1000000;
  // JSON_DOC_SEND["rt"] = 1000000;
  // JSON_DOC_SEND["lR"] = 199.123456;
  // JSON_DOC_SEND["rR"] = 199.123456;
  // JSON_DOC_SEND["ck"] = "3931756a764b44099197a21ec5d67a57";

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

  // void attachHalfQuad(int aPintNumber, int bPinNumber);
  // void attachFullQuad(int aPintNumber, int bPinNumber);
  // void attachSingleEdge(int aPintNumber, int bPinNumber);

}

void loop() {

  while (Serial.available()) {
    serialReceive();
  }

  rpm_calculator_1.calculate((int64_t)encoder_1.getCount());
  rpm_calculator_2.calculate((int64_t)encoder_2.getCount());

  packageData();

  if (micros() - timerPivot >= PERIOD) {
    sendJSON();
    timerPivot = micros();
  }
}
