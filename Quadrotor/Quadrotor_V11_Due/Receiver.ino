void Receiver() {
  // attach the interrupts used to read the pulses
  attachInterrupt(channel1Pin, updatepulse1, CHANGE);
  attachInterrupt(channel2Pin, updatepulse2, CHANGE);
  attachInterrupt(channel3Pin, updatepulse3, CHANGE);
  attachInterrupt(channel4Pin, updatepulse4, CHANGE);
}

void updatepulse1() {
  static uint32_t rising1;
  if (digitalRead(channel1Pin)) rising1 = micros();
  else {
    pulse1 = (uint32_t)(micros() - rising1);
    pulse1 = constrain(pulse1, Pulse_Min_O, Pulse_Max_O);
    pulse1 = map(pulse1, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}

void updatepulse2() {
  static uint32_t rising2;
  if (digitalRead(channel2Pin)) rising2 = micros();
  else {
    pulse2 = (uint32_t)(micros() - rising2);
    pulse2 = constrain(pulse2, Pulse_Min_O, Pulse_Max_O);
    pulse2 = map(pulse2, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}

void updatepulse3() {
  static uint32_t rising3;
  if (digitalRead(channel3Pin)) rising3 = micros();
  else {
    pulse3 = (uint32_t)(micros() - rising3);
    pulse3 = constrain(pulse3, Pulse_Min_O, Pulse_Max_O);
    pulse3 = map(pulse3, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}

void updatepulse4() {
  static uint32_t rising4;
  if (digitalRead(channel4Pin)) rising4 = micros();
  else {
    pulse4 = (uint32_t)(micros() - rising4);
    pulse4 = constrain(pulse4, Pulse_Min_O, Pulse_Max_O);
    pulse4 = map(pulse4, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}
