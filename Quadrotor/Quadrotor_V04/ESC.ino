void esc_thrust_setting() {
  if (esc_switch == 1) {
    while (pulse3 <= 1995);
    while (pulse1 <= 1990 && pulse2 <= 1990) {
      PWM1 = pulse3;
      PWM2 = pulse3;
      PWM3 = pulse3;
      PWM4 = pulse3;
      esc_thrust();
    }
  }
}

void esc_thrust() {
  start_time = micros();
  PORTD |= 0b11110000;
  end_time_1 = start_time + PWM1;
  end_time_2 = start_time + PWM2;
  end_time_3 = start_time + PWM3;
  end_time_4 = start_time + PWM4;
  while ((int)PORTD >= 16) {
    if (micros() >= end_time_1) PORTD &= 0b11101111;
    if (micros() >= end_time_2) PORTD &= 0b11011111;
    if (micros() >= end_time_3) PORTD &= 0b10111111;
    if (micros() >= end_time_4) PORTD &= 0b01111111;
  }
}
