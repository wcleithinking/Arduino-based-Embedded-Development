void Receiver() {
  PCICR  |= (1 << PCIE2);     // pin change interrupt enable 0: PCINT[23:16], which are enabled by PCMSK2
  PCMSK2 |= (1 << PCINT20);   // pin 4
  PCMSK2 |= (1 << PCINT21);   // pin 5
  PCMSK2 |= (1 << PCINT22);   // pin 6
  PCMSK2 |= (1 << PCINT23);   // pin 7
}

ISR(PCINT2_vect) {
  current_time = micros();
  // channel 1: digital pin 4 : roll signal
  if (PIND & 0b00010000) {
    if (prestate1 == 0) {
      prestate1 = 1;
      rising1 = current_time;
    }
  }
  else if (prestate1 == 1) {
    prestate1 = 0;
    pulse1 = current_time - rising1;
    pulse1 = constrain(pulse1, Pulse_Min_O, Pulse_Max_O);
    pulse1 = map(pulse1, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
  // channel 2: digital pin 5 : pitch signal
  if (PIND & 0b00100000) {
    if (prestate2 == 0) {
      prestate2 = 1;
      rising2 = current_time;
    }
  }
  else if (prestate2 == 1) {
    prestate2 = 0;
    pulse2 = current_time - rising2;
    pulse2 = constrain(pulse2, Pulse_Min_O, Pulse_Max_O);
    pulse2 = map(pulse2, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
  // channel 3: digital pin 6 : thrust signal
  if (PIND & 0b01000000) {
    if (prestate3 == 0) {
      prestate3 = 1;
      rising3 = current_time;
    }
  }
  else if (prestate3 == 1) {
    prestate3 = 0;
    pulse3 = current_time - rising3;
    pulse3 = constrain(pulse3, Pulse_Min_O, Pulse_Max_O);
    pulse3 = map(pulse3, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
  // channel 4: digital pin 7 : yaw signal
  if (PIND & 0b10000000) {
    if (prestate4 == 0) {
      prestate4 = 1;
      rising4 = current_time;
    }
  }
  else if (prestate4 == 1) {
    prestate4 = 0;
    pulse4 = current_time - rising4;
    pulse4 = constrain(pulse4, Pulse_Min_O, Pulse_Max_O);
    pulse4 = map(pulse4, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}
