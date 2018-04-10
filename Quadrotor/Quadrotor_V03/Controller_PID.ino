void pid_controller(double kp, double ki, double kd) {
  /*
    feedforward components
  */
  double roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
  roll_feedforward      = 0; //map(pulse_value_1 - 1500, pulse_value_min - 1500, pulse_value_max - 1500, -50, 50);
  pitch_feedforward     = 0; //map(pulse_value_2 - 1500, pulse_value_min - 1500, pulse_value_max - 1500, -50, 50);
  altitude_feedforward  = map(pulse_value_3, pulse_value_min, pulse_value_max, pulse_value_min, pulse_value_max - 100);
  yaw_feedforward       = 0; //map(pulse_value_4 - 1500, pulse_value_min - 1500, pulse_value_max - 1500, -50, 50);
  /*
    feedback components
  */
  double roll_feedback = 0, pitch_feedback = 0, altitude_feedback = 0, yaw_feedback = 0;
  roll_feedback         = constrain(-kp * roll * 180 / PI  - ki * roll_sum * 180 / PI  - kd * (gyro_x - gyro_x_bias) * 180 / PI, -100, 100);
  pitch_feedback        = constrain(-kp * pitch * 180 / PI - ki * pitch_sum * 180 / PI - kd * (gyro_y - gyro_y_bias) * 180 / PI, -100, 100);
  altitude_feedback     = constrain(0, -100, 100);
  yaw_feedback          = constrain(-kp * yaw * 180 / PI, -100, 100);
  /*
     controller = feedforward + feedback
  */
  double roll_controller, pitch_controller, altitude_controller, yaw_controller;
  roll_controller       = roll_feedforward + roll_feedback;
  pitch_controller      = pitch_feedforward + pitch_feedback;
  altitude_controller   = altitude_feedforward + altitude_feedback;
  yaw_controller        = yaw_feedforward + yaw_feedback;
  /*
     generate PWM signals
  */
  esc_value_1 = pitch_controller  + yaw_controller + altitude_controller;
  esc_value_2 = -roll_controller  - yaw_controller + altitude_controller;
  esc_value_3 = -pitch_controller + yaw_controller + altitude_controller;
  esc_value_4 = roll_controller   - yaw_controller + altitude_controller;
  esc_value_1 = constrain(esc_value_1, pulse_value_min, pulse_value_max);
  esc_value_2 = constrain(esc_value_2, pulse_value_min, pulse_value_max);
  esc_value_3 = constrain(esc_value_3, pulse_value_min, pulse_value_max);
  esc_value_4 = constrain(esc_value_4, pulse_value_min, pulse_value_max);
}
