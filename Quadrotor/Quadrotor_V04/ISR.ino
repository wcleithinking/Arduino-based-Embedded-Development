ISR(PCINT0_vect) {
  current_time = micros();
  // channel 1: digital pin  8 : roll signal
  if (PINB & B00000001) {
    if (prestate1 == 0) {
      prestate1 = 1;
      rising1 = current_time;
    }
  }
  else if (prestate1 == 1) {
    prestate1 = 0;
    pulse1 = current_time - rising1 - 16;
    pulse1 = constrain(pulse1, pulse_min, pulse_max);
  }
  // channel 2: digital pin 9 : pitch signal
  if (PINB & B00000010) {
    if (prestate2 == 0) {
      prestate2 = 1;
      rising2 = current_time;
    }
  }
  else if (prestate2 == 1) {
    prestate2 = 0;
    pulse2 = current_time - rising2 - 16;
    pulse2 = constrain(pulse2, pulse_min, pulse_max);
  }
  // channel 3: digital pin 10 : thrust signal
  if (PINB & B00000100) {
    if (prestate3 == 0) {
      prestate3 = 1;
      rising3 = current_time;
    }
  }
  else if (prestate3 == 1) {
    prestate3 = 0;
    pulse3 = current_time - rising3 - 16;
    pulse3 = constrain(pulse3, pulse_min, pulse_max);
  }
  // channel 4: digital pin 11 : yaw signal
  if (PINB & B00001000) {
    if (prestate4 == 0) {
      prestate4 = 1;
      rising4 = current_time;
    }
  }
  else if (prestate4 == 1) {
    prestate4 = 0;
    pulse4 = current_time - rising4 - 16;
    pulse4 = constrain(pulse4, pulse_min, pulse_max);
  }
}
