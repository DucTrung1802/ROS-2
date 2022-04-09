const int LED_PIN = 2;
char chr;
String command = "";

void control() {
  if (command == "on") {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED is ON");
  } else if (command == "off") {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED is OFF");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  while (Serial.available()) {
    chr = Serial.read();
    if (chr == '\n' || chr == '#') {
      control();
      command = "";
    } else {
      command += chr;
    }
  }
}
