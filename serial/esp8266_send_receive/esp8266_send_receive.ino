char chr;
String command = "";

void control() {
  if (command == "on") {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("LED is ON");
  } else if (command == "off") {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("LED is OFF");
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
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
