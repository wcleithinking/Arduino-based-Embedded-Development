volatile uint32_t pulse1, pulse2, pulse3, pulse4;

void Receiver_init() {
  attachInterrupt(channel1Pin, updatepulse1, CHANGE);
  attachInterrupt(channel2Pin, updatepulse2, CHANGE);
  attachInterrupt(channel3Pin, updatepulse3, CHANGE);
  attachInterrupt(channel4Pin, updatepulse4, CHANGE);
}

void Receiver_copy() {
  RC[IndexRoll]     = pulse1;
  RC[IndexPitch]    = pulse2;
  RC[IndexYaw]      = pulse4;
  RC[IndexAltitude] = pulse3;
}

// Roll
void updatepulse1() {
  static uint32_t rising1;
  if (digitalRead(channel1Pin)) rising1 = micros();
  else {
    pulse1 = (uint32_t)(micros() - rising1);
    pulse1 = constrain(pulse1, Pulse_MIN_Original, Pulse_MAX_Original);
    pulse1 = map(pulse1, Pulse_MIN_Original, Pulse_MAX_Original, Pulse_MIN, Pulse_MAX);
  }
}
// Pitch
void updatepulse2() {
  static uint32_t rising2;
  if (digitalRead(channel2Pin)) rising2 = micros();
  else {
    pulse2 = (uint32_t)(micros() - rising2);
    pulse2 = constrain(pulse2, Pulse_MIN_Original, Pulse_MAX_Original);
    pulse2 = map(pulse2, Pulse_MIN_Original, Pulse_MAX_Original, Pulse_MIN, Pulse_MAX);
  }
}
// Altitude
void updatepulse3() {
  static uint32_t rising3;
  if (digitalRead(channel3Pin)) rising3 = micros();
  else {
    pulse3 = (uint32_t)(micros() - rising3);
    pulse3 = constrain(pulse3, Pulse_MIN_Original, Pulse_MAX_Original);
    pulse3 = map(pulse3, Pulse_MIN_Original, Pulse_MAX_Original, Pulse_MIN, Pulse_MAX);
  }
}
// Yaw
void updatepulse4() {
  static uint32_t rising4;
  if (digitalRead(channel4Pin)) rising4 = micros();
  else {
    pulse4 = (uint32_t)(micros() - rising4);
    pulse4 = constrain(pulse4, Pulse_MIN_Original, Pulse_MAX_Original);
    pulse4 = map(pulse4, Pulse_MIN_Original, Pulse_MAX_Original, Pulse_MIN, Pulse_MAX);
  }
}
