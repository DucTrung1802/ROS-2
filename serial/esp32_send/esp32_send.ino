unsigned long number = 0;

void setup() {
  Serial.begin(115200);

}

void loop() {
  Serial.println(number);
  number++;
  delay(1);
}
