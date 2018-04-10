void pid_controller() {
  /*
    feedforward components
  */
  double roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
  roll_feedforward     = 0;
  pitch_feedforward    = 0;
  altitude_feedforward = map(pulse3, pulse_min, pulse_max, pulse_min, pulse_max - 100);
  yaw_feedforward      = 0;
  /*
    feedback components
  */
  double roll_feedback = 0, pitch_feedback = 0, altitude_feedback = 0, yaw_feedback = 0;
  roll_feedback     = constrain(-(kp * roll  + ki * roll_sum  + kd * (gyro_x - gyro_x_bias)) * 180 / PI, -50, 50);
  pitch_feedback    = constrain(-(kp * pitch + ki * pitch_sum + kd * (gyro_y - gyro_y_bias)) * 180 / PI, -50, 50);
  altitude_feedback = 0;
  yaw_feedback      = constrain(-kp * yaw * 180 / PI, -50, 50);
  /*
     controller = feedforward + feedback
  */
  double roll_controller, pitch_controller, altitude_controller, yaw_controller;
  roll_controller     = roll_feedforward + roll_feedback;
  pitch_controller    = pitch_feedforward + pitch_feedback;
  altitude_controller = altitude_feedforward + altitude_feedback;
  yaw_controller      = yaw_feedforward + yaw_feedback;
  /*
     generate PWM signals
  */
  u1 = pitch_controller  + yaw_controller + altitude_controller;
  u2 = -roll_controller  - yaw_controller + altitude_controller;
  u3 = -pitch_controller + yaw_controller + altitude_controller;
  u4 = roll_controller   - yaw_controller + altitude_controller;
  PWM1 = constrain(u1, pulse_min, pulse_max);
  PWM2 = constrain(u2, pulse_min, pulse_max);
  PWM3 = constrain(u3, pulse_min, pulse_max);
  PWM4 = constrain(u4, pulse_min, pulse_max);
}
