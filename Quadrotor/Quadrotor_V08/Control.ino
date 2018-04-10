void Control() {
  if (loop_index < 350) {
    // Update Control Signals
    for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
  }
  else {
    if (pulse1 <= (Throttle_MIN + 15) && pulse2 >= (Throttle_MAX - 15)) {
      led_high();     // indicate lock
      esc_switch = 0; // lock motor
      I_reset();      // reset integration
    }
    if (pulse1 >= (Throttle_MAX - 10) && pulse2 >= (Throttle_MAX - 10)) {
      led_low();      // indicate unlock
      esc_switch = 1; // unlock motor
    }
    // Update Control Signals
    if (esc_switch == 0 ) {
      for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
    }
    else if (esc_switch == 1) {
      PID_controller();
    }
  }
}

void PID_controller() {
  // feedback
#if defined(PID_v1)
  roll_sum += (roll - roll_d) * dt;
  pitch_sum += (pitch - pitch_d) * dt;
  yaw_sum += (yaw - yaw_d) * dt;
  roll_feedback = ( - kp * roll - ki * roll_sum - kd * (gyro_x - gyro_x_bias) ) * 180 / PI;
  roll_feedback = constrain(roll_feedback, -umax, umax);
  pitch_feedback = ( - kp * pitch - ki * pitch_sum - kd * (gyro_y - gyro_y_bias) ) * 180 / PI;
  pitch_feedback = constrain(pitch_feedback, -umax, umax);
#if defined(SONAR)
  if (loop_index < 650) altitude_feedback = 0;
  else {
    if (sonar_error_flag = 0) altitude_sum += (altitude - altitude_d) * dt;
    else altitude_sum = 0;
    altitude_feedback = (kp_z * (altitude - altitude_d) + ki_z * altitude_sum) * 100;
  }
  altitude_feedback = constrain(altitude_feedback, -umax / 5, umax / 5);
#elif defined(BARO)
  if (loop_index < 650) altitude_feedback = 0;
  else {
    altitude_sum += (altitude - altitude_d) * dt;
    altitude_feedback = (kp_z * (altitude - altitude_d) + ki_z * altitude_sum) * 100;
  }
  altitude_feedback = constrain(altitude_feedback, -umax / 5, umax / 5);
#else
  altitude_feedback = 0;
#endif
  yaw_feedback = (-kp * yaw - ki * yaw_sum - kd * (gyro_z - gyro_z_bias) ) * 180 / PI;
  yaw_feedback = constrain( yaw_feedback, -umax, umax);
#elif defined(PID_v2)
  double y[3] = {roll, pitch, yaw};
  for (int i = 0; i < 3; i++) {
    e[i] = (yd[i] - y[i]) * RadToDeg;
    P[i] = e[i];
    D[i] = exp(-dt * N[i] / Td[i]) * Dold[i] + N[i] * (e[i] - eold[i]);
#ifdef PID_v2_MODIFIED
    if (abs(e[i]) <= 15) {
      PID[i] = K[i] * (P[i] + I[i] + D[i]);
      I[i] = I[i] + (dt / Ti[i]) * e[i];
    }
    else {
      PID[i] = K[i] * (P[i] + D[i]);
      I[i] = 0;
    }
#else
    PID[i] = K[i] * (P[i] + I[i] + D[i]);
    I[i] = I[i] + (dt / Ti[i]) * e[i];
#endif
    eold[i] = e[i];
    Dold[i] = D[i];
  }
  roll_feedback = constrain(PID[0], -umax, umax);
  pitch_feedback = constrain(PID[1], -umax, umax);
  altitude_feedback = 0;
  yaw_feedback = constrain( PID[2], -umax, umax);
#endif
  // feedforward
  roll_feedforward  = 0;
  pitch_feedforward = 0;
  altitude_feedforward = map(pulse3, Throttle_MIN, Throttle_MIN, pulse_min, pulse_max);
  altitude_feedforward  = constrain(pulse3, pulse_min, pulse_max);
  yaw_feedforward = 0;
  // all
  roll_controller = roll_feedforward + roll_feedback;
  pitch_controller = pitch_feedforward + pitch_feedback;
  altitude_controller = altitude_feedforward + altitude_feedback;
  yaw_controller = yaw_feedforward + yaw_feedback;
#if defined(QuadP)
  u[0] = pitch_controller  - yaw_controller + altitude_controller;
  u[1] = -roll_controller  + yaw_controller + altitude_controller;
  u[2] = -pitch_controller - yaw_controller + altitude_controller;
  u[3] = roll_controller   + yaw_controller + altitude_controller;
#elif defined(QuadX)
  u[0] =  roll_controller + pitch_controller + yaw_controller + altitude_controller;
  u[1] = -roll_controller + pitch_controller - yaw_controller + altitude_controller;
  u[2] = -roll_controller - pitch_controller + yaw_controller + altitude_controller;
  u[3] =  roll_controller - pitch_controller - yaw_controller + altitude_controller;
#endif
  MOTOR[0] = constrain(u[0], pulse_min, pulse_max);
  MOTOR[1] = constrain(u[1], pulse_min, pulse_max);
  MOTOR[2] = constrain(u[2], pulse_min, pulse_max);
  MOTOR[3] = constrain(u[3], pulse_min, pulse_max);
#ifdef DEBUG
  for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
#endif
}

void I_reset() {
#if defined(PID_v1)
  // reset sum
  roll_sum = 0;
  pitch_sum = 0;
  yaw_sum = 0;
#if defined(SONAR) || defined(BARO)
  altitude_sum = 0;
#endif
#elif defined(PID_v2)
  // reset I
  for (int i = 0; i < 3; i++) I[i] = 0;
#endif
}
