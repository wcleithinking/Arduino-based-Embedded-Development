void LED() {
  pinMode(ledPin, OUTPUT);
  led_high(); // indicate start config
}

void led_high() {
  digitalWrite(ledPin, HIGH);
}

void led_low() {
  digitalWrite(ledPin, LOW);
}

