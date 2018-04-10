void LED_init() {
  pinMode(ledPin, OUTPUT);
}

void LED_high() {
  digitalWrite(ledPin, HIGH);
}

void LED_low() {
  digitalWrite(ledPin, LOW);
}

