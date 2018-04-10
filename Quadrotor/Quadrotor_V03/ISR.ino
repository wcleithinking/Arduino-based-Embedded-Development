ISR(PCINT0_vect) {
  current_time = micros();
  // channel 1: digital pin  8 : roll signal
  if (PINB & B00000001) {
    if (previous_state_1 == 0) {
      previous_state_1 = 1;
      rising_time_1 = current_time;
    }
  }
  else if (previous_state_1 == 1) {
    previous_state_1 = 0;
    pulse_value_1 = current_time - rising_time_1 - 16;
    pulse_value_1 = constrain(pulse_value_1, pulse_value_min, pulse_value_max);
  }
  // channel 2: digital pin 9 : pitch signal
  if (PINB & B00000010) {
    if (previous_state_2 == 0) {
      previous_state_2 = 1;
      rising_time_2 = current_time;
    }
  }
  else if (previous_state_2 == 1) {
    previous_state_2 = 0;
    pulse_value_2 = current_time - rising_time_2 - 16;
    pulse_value_2 = constrain(pulse_value_2, pulse_value_min, pulse_value_max);
  }
  // channel 3: digital pin 10 : thrust signal
  if (PINB & B00000100) {
    if (previous_state_3 == 0) {
      previous_state_3 = 1;
      rising_time_3 = current_time;
    }
  }
  else if (previous_state_3 == 1) {
    previous_state_3 = 0;
    pulse_value_3 = current_time - rising_time_3 - 16;
    pulse_value_3 = constrain(pulse_value_3, pulse_value_min, pulse_value_max);
  }
  // channel 4: digital pin 11 : yaw signal
  if (PINB & B00001000) {
    if (previous_state_4 == 0) {
      previous_state_4 = 1;
      rising_time_4 = current_time;
    }
  }
  else if (previous_state_4 == 1) {
    previous_state_4 = 0;
    pulse_value_4 = current_time - rising_time_4 - 16;
    pulse_value_4 = constrain(pulse_value_4, pulse_value_min, pulse_value_max);
  }
}
