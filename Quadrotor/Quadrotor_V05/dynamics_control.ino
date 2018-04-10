void dynamics_control() {
  if (esc_switch == 0 ) {
    for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
  }
  else if (esc_switch == 1) {
    PID_controller();
  }
}

void PID_controller() {
  /*
    feedforward components
  */
  float roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
  roll_feedforward  = 0;//map(pulse1, pulse_min, pulse_max, -umax, umax);
  pitch_feedforward = 0;//map(pulse2, pulse_min, pulse_max, -umax, umax);
  altitude_feedforward  = constrain(pulse3, pulse_min, pulse_max - 2 * umax);
  yaw_feedforward = 0;//map(pulse4, pulse_min, pulse_max, -umax, umax);
  /*
    feedback components
  */
  float roll_feedback = 0, pitch_feedback = 0, altitude_feedback = 0, yaw_feedback = 0;
  roll_feedback = ( - kp * roll - ki * roll_sum - kd * (gyro_x - gyro_x_bias) ) * 180 / PI;
  roll_feedback = constrain(roll_feedback, -umax, umax);
  pitch_feedback = ( - kp * pitch - ki * pitch_sum - kd * (gyro_y - gyro_y_bias) ) * 180 / PI;
  pitch_feedback = constrain(pitch_feedback, -umax, umax);
  altitude_feedback = 0;
  altitude_feedback = constrain(altitude_feedback, -20, 20);
  yaw_feedback = (-kp * yaw - ki * yaw_sum - kd * (gyro_z - gyro_z_bias) ) * 180 / PI;
  yaw_feedback = constrain( yaw_feedback, -umax, umax);
  /*
     controller = feedforward + feedback
  */
  float roll_controller, pitch_controller, altitude_controller, yaw_controller;
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
