/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com
*********/

// Config pin parameters
// Motor 1
const uint8_t AIN1 = 25;
const uint8_t AIN2 = 26;
const uint8_t PWMA = 27;
const uint8_t ENC1_A = 22;
const uint8_t ENC1_B = 23;

// Motor 2
const uint8_t BIN1 = 19;
const uint8_t BIN2 = 18;
const uint8_t PWMB = 5;
const uint8_t ENC2_A = 17;
const uint8_t ENC2_B = 16;

const uint8_t STBY = 13;

// Config pwm parameters
const int FREQ_PWM_1 = 1000;
const int RESOLUTION_PWM_1 = 10; // bits (1 - 16 bits)
const uint8_t CHANNEL_PWMA = 1;

const int FREQ_PWM_2 = 1000;
const int RESOLUTION_PWM_2 = 10; // bits (1 - 16 bits)
const uint8_t CHANNEL_PWMB = 2;

int POS_1 = 0;
int POS_2 = 0;

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
    POS_2++;
  }
  else
  {
    POS_2--;
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

void setup()
{
  Serial.begin(115200);

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
  
  // initializeMotor();
  
  ledcWrite(CHANNEL_PWMA, 1023);
  ledcWrite(CHANNEL_PWMB, 1023);
}

void loop()
{
  Serial.println("\n-------------");
  Serial.print("POS_1: ");
  Serial.println(POS_1);
  Serial.println();
  Serial.print("POS_2: ");
  Serial.println(POS_2);
  Serial.println("-------------");
  delay(1000);
}
