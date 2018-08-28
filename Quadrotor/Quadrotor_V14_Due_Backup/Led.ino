void Led_init() {
  pinMode(ledPin, OUTPUT);
}

void Led_high() {
  digitalWrite(ledPin, HIGH);
}

void Led_low() {
  digitalWrite(ledPin, LOW);
}

void Led_blink(uint8_t num) {
  for (int i = 0; i < num; i++) {
    Led_high();
    delay(500);
    Led_low();
    delay(500);
  }
}

void Led_armstate() {
  digitalWrite(ledPin, ARM_flag);
}
