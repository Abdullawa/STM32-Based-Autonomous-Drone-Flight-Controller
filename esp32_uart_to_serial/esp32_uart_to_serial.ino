const int PWM_PIN = 11;
volatile uint32_t last_rising_time = 0;
volatile uint32_t latest_pulse_width = 0;
volatile bool pulse_updated = false;

void IRAM_ATTR pwm_isr() {
  uint32_t t = micros();
  if (digitalRead(PWM_PIN) == HIGH) {
    last_rising_time = t;
  } else {
    latest_pulse_width = t - last_rising_time;
    pulse_updated = true;
  }
}

uint32_t consume_pulse_width() {
  noInterrupts();
  uint32_t w = latest_pulse_width;
  pulse_updated = false;
  interrupts();
  return w;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 10, 17);
  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_isr, CHANGE);
}

void loop() {
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  static uint32_t last = 0;
  if (millis() - last > 100) {
    last = millis();
    Serial.print("GPIO11: ");
    Serial.println(digitalRead(11));
  }

  if (pulse_updated) {
    uint32_t pw = consume_pulse_width();
    Serial.print("PWM_us: ");
    Serial.println(pw);
  }

  delay(5);
}