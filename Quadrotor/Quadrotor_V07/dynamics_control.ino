void dynamics_control() {
  if (esc_switch == 0 ) {
    for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
  }
  else if (esc_switch == 1) {
    PID_controller();
  }
}

void PID_controller() {
  double y[3] = {roll, pitch, yaw};
  for (int i = 0; i < 3; i++) {
    e[i] = yd[i] - y[i];
    P[i] = e[i];
    D[i] = exp(-dt * N / Td) * Dold[i] + N * (e[i] - eold[i]);
#ifdef MODIFIED_PID
    if (abs(e[i]) <= 15) {
      PID[i] = K * (P[i] + I[i] + D[i]);
      I[i] = I[i] + (dt / Ti) * e[i];
    }
    else {
      PID[i] = P[i] + D[i];
      I[i] = 0;
    }
#else
    PID[i] = P[i] + I[i] + D[i];
    I[i] = I[i] + (K * dt / Ti) * e[i];
    eold[i] = e[i];
    Dold[i] = D[i];
#endif
  }
  // feedback
  roll_feedback = constrain(PID[0], -umax, umax);
  pitch_feedback = constrain(PID[1], -umax, umax);
  altitude_feedback = 0;
  yaw_feedback = constrain( PID[2], -umax, umax);
  // feedforward
  roll_feedforward  = 0;
  pitch_feedforward = 0;
  altitude_feedforward  = constrain(pulse3, pulse_min, pulse_max);
  yaw_feedforward = 0;
  // all
  roll_controller = roll_feedforward + roll_feedback;
  pitch_controller = pitch_feedforward + pitch_feedback;
  altitude_controller = altitude_feedforward + altitude_feedback;
  yaw_controller = yaw_feedforward + yaw_feedback;

#if defined(QuadP)
  /*
    For Quad "+" Configuration
  */
  u[0] = pitch_controller  - yaw_controller + altitude_controller;
  u[1] = -roll_controller  + yaw_controller + altitude_controller;
  u[2] = -pitch_controller - yaw_controller + altitude_controller;
  u[3] = roll_controller   + yaw_controller + altitude_controller;
#elif defined(QuadX)
  /*
     For Quad "X" Configuration
  */
  u[0] =  roll_controller + pitch_controller + yaw_controller + altitude_controller;
  u[1] = -roll_controller + pitch_controller - yaw_controller + altitude_controller;
  u[2] = -roll_controller - pitch_controller + yaw_controller + altitude_controller;
  u[3] =  roll_controller - pitch_controller - yaw_controller + altitude_controller;
#endif

  MOTOR[0] = constrain(u[0], pulse_min, pulse_max);
  MOTOR[1] = constrain(u[1], pulse_min, pulse_max);
  MOTOR[2] = constrain(u[2], pulse_min, pulse_max);
  MOTOR[3] = constrain(u[3], pulse_min, pulse_max);
}
