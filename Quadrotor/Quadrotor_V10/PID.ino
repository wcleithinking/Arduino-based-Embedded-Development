void motor_control() {
  if (loop_index < 350) {
    for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
  }
  else {
    // Judge the Lock of Motor
    if (pulse1 <= (PWM_MIN + 15) && pulse2 >= (PWM_MAX - 15)) {
      LED_high();     // indicate lock
      esc_switch = 0; // lock motor
    }
    if (pulse1 >= (PWM_MAX - 10) && pulse2 >= (PWM_MAX - 10)) {
      LED_low();      // indicate unlock
      esc_switch = 1; // unlock motor
    }
    // Update Control Signals
    if (esc_switch == 0 ) {
      reset_I();    // reset integration
      for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
    }
    else if (esc_switch == 1) {
      controller();
    }
  }
}

void controller() {
  // feedback
  PID_controller_roll();
  PID_controller_pitch();
  PID_controller_yaw();
  // feedforward
  roll_feedforward  = 0;
  pitch_feedforward = 0;
  altitude_feedforward = map(pulse3, Pulse_Min, Pulse_Max, PWM_MIN, PWM_MAX);
  altitude_feedforward = constrain(altitude_feedforward, PWM_MIN, PWM_MAX);
  yaw_feedforward = 0;
  // all
  roll_controller = roll_feedforward + roll_feedback;
  pitch_controller = pitch_feedforward + pitch_feedback;
  altitude_controller = altitude_feedforward + altitude_feedback;
  yaw_controller = yaw_feedforward + yaw_feedback;
#if defined(QuadP)
  PWM_ref[0] =  pitch_controller - yaw_controller + altitude_controller;
  PWM_ref[1] = -roll_controller  + yaw_controller + altitude_controller;
  PWM_ref[2] = -pitch_controller - yaw_controller + altitude_controller;
  PWM_ref[3] =  roll_controller  + yaw_controller + altitude_controller;
#elif defined(QuadX)
  PWM_ref[0] =  roll_controller + pitch_controller + yaw_controller + altitude_controller;
  PWM_ref[1] = -roll_controller + pitch_controller - yaw_controller + altitude_controller;
  PWM_ref[2] = -roll_controller - pitch_controller + yaw_controller + altitude_controller;
  PWM_ref[3] =  roll_controller - pitch_controller - yaw_controller + altitude_controller;
#endif
  PWM_out[0] = constrain(PWM_ref[0], PWM_MIN, PWM_MAX);
  PWM_out[1] = constrain(PWM_ref[1], PWM_MIN, PWM_MAX);
  PWM_out[2] = constrain(PWM_ref[2], PWM_MIN, PWM_MAX);
  PWM_out[3] = constrain(PWM_ref[3], PWM_MIN, PWM_MAX);
  if (SerialDebug) for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
}

void PID_controller_roll() {
#if defined(PID_v1)
  roll_sum += (roll - roll_d) * dt;
  roll_feedback  = - kp * roll - ki * roll_sum - kd * droll;
#elif defined(PID_v2)
  e[0] = (roll_d - roll) * RadToDeg;
  P[0] = K[0] * e[0];
  D[0] = K[0] * (exp(-dt * N[0] / Td[0]) * Dold[0] + N[0] * (e[0] - eold[0]));
  PID[0] = P[0] + I[0] + D[0];
  I[0] = K[0] * (I[0] + (dt / Ti[0]) * e[0]);
  eold[0] = e[0];
  Dold[0] = D[0];
  roll_feedback  = PID[0];
#endif
  roll_feedback  = constrain(roll_feedback, -umax, umax);
}

void PID_controller_pitch() {
#if defined(PID_v1)
  pitch_sum += (pitch - pitch_d) * dt;
  pitch_feedback = - kp * pitch - ki * pitch_sum - kd * dpitch;
#elif defined(PID_v2)
  e[1] = (pitch_d - pitch) * RadToDeg;
  P[1] = K[1] * e[1];
  D[1] = K[1] * (exp(-dt * N[1] / Td[1]) * Dold[1] + N[1] * (e[1] - eold[1]));
  PID[1] = P[1] + I[1] + D[1];
  I[1] = K[1] * (I[1] + (dt / Ti[1]) * e[1]);
  eold[1] = e[1];
  Dold[1] = D[1];
  pitch_feedback  = PID[1];
#endif
  pitch_feedback = constrain(pitch_feedback, -umax, umax);
}

void PID_controller_yaw() {
#ifdef PID_v1
  yaw_sum += (yaw - yaw_d) * dt;
  yaw_feedback = - kp * yaw - ki * yaw_sum - kd * dyaw;
#elif defined(PID_v2)
  e[2] = (yaw_d - yaw) * RadToDeg;
  P[2] = K[2] * e[2];
  D[2] = K[2] * (exp(-dt * N[2] / Td[2]) * Dold[2] + N[2] * (e[2] - eold[2]));
  PID[2] = P[2] + I[2] + D[2];
  I[2] = K[2] * (I[2] + (dt / Ti[2]) * e[2]);
  eold[2] = e[2];
  Dold[2] = D[2];
  yaw_feedback  = PID[2];
#endif
  yaw_feedback = constrain( yaw_feedback, -umax, umax);
}

void reset_I() {
#if defined(PID_v1)
  roll_sum = 0;
  pitch_sum = 0;
  yaw_sum = 0;
#elif defined(PID_v2)
  for (int i = 0; i < 3; i++) I[i] = 0;
#endif
}
