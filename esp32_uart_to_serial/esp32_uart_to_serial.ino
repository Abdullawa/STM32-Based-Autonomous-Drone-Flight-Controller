const int RX_PIN = 10;

void setup() {
  Serial.begin(460800);
  Serial1.begin(460800, SERIAL_8N1, RX_PIN, -1); // RX=GPIO10, no TX
}

void loop() {
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}