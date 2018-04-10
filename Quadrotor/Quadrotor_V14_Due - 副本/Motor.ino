void Motor_init() {
  motor[0].attach(motor0Pin);
  motor[1].attach(motor1Pin);
  motor[2].attach(motor2Pin);
  motor[3].attach(motor3Pin);
}

void Motor_update() {
  for (int i = 0; i < 4; i++) motor[i].writeMicroseconds(PWM_out[i]);
}

void Motor_control() {
  if (loop_index < 350) {
    for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
  }
  else {
    if (RC[IndexRoll] <= (PWM_MIN + 15) && RC[IndexPitch] >= (PWM_MAX - 15)) {
      ARM_flag = 0;
      Led_armstate();
    }
    if (RC[IndexRoll] >= (PWM_MAX - 10) && RC[IndexPitch] >= (PWM_MAX - 10)) {
      ARM_flag = 1;
      Led_armstate();
    }
    if (ARM_flag == 0 ) {
      Controller_cleardata();
      for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
    }
    else if (ARM_flag == 1) {
      Receiver_copy();
      Controller_run();
    }
  }
}
