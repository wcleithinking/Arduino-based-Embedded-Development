void esc_thrust_setting() {
  if (esc_state_switch == 1) {
    while (pulse_value_3 <= 1995);
    while (pulse_value_1 <= 1990 && pulse_value_2 <= 1990) {
      esc_value_1 = pulse_value_3;
      esc_value_2 = pulse_value_3;
      esc_value_3 = pulse_value_3;
      esc_value_4 = pulse_value_3;
      esc_thrust();
    }
  }
}

void esc_thrust() {
  start_time = micros();
  PORTD |= 0b11110000;
  end_time_1 = start_time + esc_value_1;
  end_time_2 = start_time + esc_value_2;
  end_time_3 = start_time + esc_value_3;
  end_time_4 = start_time + esc_value_4;
  while ((int)PORTD >= 16) {
    if (micros() >= end_time_1) PORTD &= 0b11101111;
    if (micros() >= end_time_2) PORTD &= 0b11011111;
    if (micros() >= end_time_3) PORTD &= 0b10111111;
    if (micros() >= end_time_4) PORTD &= 0b01111111;
  }
}
