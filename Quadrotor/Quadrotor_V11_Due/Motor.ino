void Motor() {
  //  analogWrite(motor1Pin, MOTOR[0]);
  //  analogWrite(motor2Pin, MOTOR[1]);
  //  analogWrite(motor3Pin, MOTOR[2]);
  //  analogWrite(motor4Pin, MOTOR[3]);
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  motor1.writeMicroseconds(PWM_out[0]);
  motor2.writeMicroseconds(PWM_out[1]);
  motor3.writeMicroseconds(PWM_out[2]);
  motor4.writeMicroseconds(PWM_out[3]);
}

void motor_update() {
  //  for (int i = 0; i < 4; i++) {
  //    // need modification for more accurate PWM input
  //    MOTOR[i] = map(PWM_out[i], PWM_MIN, PWM_MAX, 127, (int)(127 + 0.128 * (PWM_MAX - PWM_MIN)));
  //  }
  //  analogWrite(motor1Pin, MOTOR[0]);
  //  analogWrite(motor2Pin, MOTOR[1]);
  //  analogWrite(motor3Pin, MOTOR[2]);
  //  analogWrite(motor4Pin, MOTOR[3]);
  motor1.writeMicroseconds(PWM_out[0]);
  motor2.writeMicroseconds(PWM_out[1]);
  motor3.writeMicroseconds(PWM_out[2]);
  motor4.writeMicroseconds(PWM_out[3]);
}

void motor_control() {
  if (loop_index < 350) {
    for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
  }
  else {
    // check the lock of motor
    if (pulse1 <= (PWM_MIN + 15) && pulse2 >= (PWM_MAX - 15)) {
      led_high();                     // indicate lock
      esc_switch = 0;                 // lock motor
    }
    if (pulse1 >= (PWM_MAX - 10) && pulse2 >= (PWM_MAX - 10)) {
      led_low();                      // indicate unlock
      esc_switch = 1;                 // unlock motor
    }
    // update control signals
    if (esc_switch == 0 ) {
      reset_I();                      // reset integration
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
#ifdef STR_YAW
  time_yaw_new = millis();
  if (time_yaw_new >= time_yaw_old + 96) {
#ifdef DEBUG
    time_yaw_diff = time_yaw_new - time_yaw_old;
#endif
    time_yaw_old = time_yaw_new;
#ifdef DEBUG
    time_1 = millis();
#endif
    STR_controller_yaw();
#ifdef DEBUG
    time_2 = millis();
#endif
  }
#else
  PID_controller_yaw();
#endif
  // feedforward
  roll_feedforward  = 0;
  pitch_feedforward = 0;
  yaw_feedforward   = 0;
  altitude_feedforward = map(pulse3, Pulse_Min, Pulse_Max, PWM_MIN, PWM_MAX);
  altitude_feedforward = constrain(altitude_feedforward, PWM_MIN, PWM_MAX);
  // all
  roll_controller  = roll_feedforward  + roll_feedback;
  pitch_controller = pitch_feedforward + pitch_feedback;
  yaw_controller   = yaw_feedforward   + yaw_feedback;
  altitude_controller = altitude_feedforward + altitude_feedback;
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
#ifdef DEBUG
  for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
#endif
}

void reset_I() {
#if defined(PID_v1)
  roll_sum = 0;
  pitch_sum = 0;
  yaw_sum = 0;
#elif defined(PID_v2)
  for (int i = 0; i < 3; i++) I[i] = 0;
#elif defined(PID_v3)
  roll_sum = 0;
  pitch_sum = 0;
  yaw_sum = 0;
  droll_sum = 0;
  dpitch_sum = 0;
  dyaw_sum = 0;
#endif
}
