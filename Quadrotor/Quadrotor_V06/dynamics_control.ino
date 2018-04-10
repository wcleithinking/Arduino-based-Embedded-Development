double t0[4];
double t1[4];
double t2[4];
double r1[4];
double r2[4];
double s0[4];
double s1[4];
double s2[4];

void dynamics_control() {
  if (esc_switch == 1) {
    PID_controller();
    self_tuning_regulator();
  }
  else if (esc_switch == 0 ) {
    for (int i = 0; i < 4; i++) {
      PWM[i] = pulse_min;
      u[i] = 0;
    }
  }
}

void PID_controller() {
  /*
    feedforward components
  */
  double roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
  roll_feedforward  = 0;//map(pulse1 - 1500, pulse_min - 1500, pulse_max - 1500, -50, 50);
  pitch_feedforward = 0;//map(pulse2 - 1500, pulse_min - 1500, pulse_max - 1500, -50, 50);
  altitude_feedforward  = map(pulse3, pulse_min, pulse_max, pulse_min, pulse_max - 100);
  yaw_feedforward = 0;//map(pulse4 - 1500, pulse_min - 1500, pulse_max - 1500, -50, 50);
  /*
    feedback components
  */
  double roll_feedback = 0, pitch_feedback = 0, altitude_feedback = 0, yaw_feedback = 0;
  roll_feedback = ( -kp_accel * accel_y - kp * roll - ki * roll_sum - kd * (gyro_x - gyro_x_bias) ) * 180 / PI;
  roll_feedback = constrain(roll_feedback, -100, 100);
  pitch_feedback = ( kp_accel * accel_x - kp * pitch - ki * pitch_sum - kd * (gyro_y - gyro_y_bias) ) * 180 / PI;
  pitch_feedback = constrain(pitch_feedback, -100, 100);
  altitude_feedback = 0;
  altitude_feedback = constrain(altitude_feedback, -100, 100);
  yaw_feedback = (-kp * yaw - ki * yaw_sum - kd * (gyro_z - gyro_z_bias) ) * 180 / PI;
  yaw_feedback = constrain( yaw_feedback, -100, 100);
  /*
     controller = feedforward + feedback
  */
  double roll_controller, pitch_controller, altitude_controller, yaw_controller;
  roll_controller = roll_feedforward + roll_feedback;
  pitch_controller = pitch_feedforward + pitch_feedback;
  altitude_controller = altitude_feedforward + altitude_feedback;
  yaw_controller = yaw_feedforward + yaw_feedback;
  PWM[0] = constrain(pitch_controller  + yaw_controller + altitude_controller, pulse_min, pulse_max);
  PWM[1] = constrain(-roll_controller  - yaw_controller + altitude_controller, pulse_min, pulse_max);
  PWM[2] = constrain(-pitch_controller + yaw_controller + altitude_controller, pulse_min, pulse_max);
  PWM[3] = constrain(roll_controller   - yaw_controller + altitude_controller, pulse_min, pulse_max);
  for (int i = 0; i < 4; i++) PWM_percent[i] = (PWM[i] - 1000) * 0.001;
  u[0] = PWM_percent[3] - PWM_percent[1];
  u[1] = PWM_percent[0] - PWM_percent[2];
  u[2] = PWM_percent[0] - PWM_percent[1] + PWM_percent[2] - PWM_percent[3];
  u[3] = -PWM_percent[0] - PWM_percent[1] - PWM_percent[2] - PWM_percent[3];
}

void self_tuning_regulator() {
  double beta;
  double u_ref[4];
  double AA[5][5];
  double AA_inv[5][5];
  double bb[5];
  double xx[5];
  for (int i; i < 4; i++) {
    // solve diophantine equation: AR + BS = A_c = A_m*A_o
    beta = (1 + am1[i] + am2[i] + am3[i]) / (hatb0[i] + hatb1[i] + hatb2[i]);
    t0[i] = beta;
    t1[i] = beta * ao1[i];
    t2[i] = beta * ao2[i];
    AA[0][0] = 1;
    AA[0][1] = 0;
    AA[0][2] = hatb0[i];
    AA[0][3] = 0;
    AA[0][4] = 0;
    AA[1][0] = hata1[i];
    AA[1][1] = 1;
    AA[1][2] = hatb1[i];
    AA[1][3] = hatb0[i];
    AA[1][4] = 0;
    AA[2][0] = hata2[i];
    AA[2][1] = hata1[i];
    AA[2][2] = hatb2[i];
    AA[2][3] = hatb1[i];
    AA[2][4] = hatb0[i];
    AA[3][0] = hata3[i];
    AA[3][1] = hata2[i];
    AA[3][2] = 0;
    AA[3][3] = hatb2[i];
    AA[3][4] = hatb1[i];
    AA[4][0] = 0;
    AA[4][1] = hata3[i];
    AA[4][2] = 0;
    AA[4][3] = 0;
    AA[4][4] = hatb2[i];
    bb[0] = am1[i] + ao1[i] - hata1[i];
    bb[1] = am2[i] + ao1[i] * am1[i] + ao2[i] - hata2[i];
    bb[2] = am3[i] + ao1[i] * am2[i] + ao2[i] * am1[i] - hata3[i];
    bb[3] = ao1[i] * am3[i] + ao2[i] * am2[i];
    bb[4] = ao2[i] * am3[i];
    getInverse(AA, 5, AA_inv);
    // xx = AA\bb = AA_inv*bb
    for (int i = 0; i < 5; i++) {
      xx[i] = 0;
      for (int j = 0; j < 5; j++) xx[i] += AA_inv[i][j] * bb[j];
    }
    r1[i] = xx[0];
    r2[i] = xx[1];
    s0[i] = xx[2];
    s1[i] = xx[3];
    s2[i] = xx[4];
  }
  // design referenced controllers according to Ru = Tu_c - Sy
  u_ref[0] = -r1[0] * u_o[0] - r2[0] * u_oo[0] + t0[0] * uc[0] + t1[0] * uc_o[0] + t2[0] * uc_oo[0] - s0[0] * roll - s1[0] * roll_o - s2[0] * roll_oo;
  u_ref[1] = -r1[1] * u_o[1] - r2[1] * u_oo[1] + t0[1] * uc[1] + t1[1] * uc_o[1] + t2[1] * uc_oo[1] - s0[1] * pitch - s1[1] * pitch_o - s2[1] * pitch_oo;
  u_ref[2] = -r1[2] * u_o[2] - r2[2] * u_oo[2] + t0[2] * uc[2] + t1[2] * uc_o[2] + t2[2] * uc_oo[2] - s0[2] * yaw - s1[2] * yaw_o - s2[2] * yaw_oo;
  u_ref[3] = -r1[3] * u_o[3] - r2[3] * u_oo[3] + t0[3] * uc[3] + t1[3] * uc_o[3] + t2[3] * uc_oo[3] - s0[3] * altitude - s1[3] * altitude_o - s2[3] * altitude_oo;
  u_ref[0] = constrain(u_ref[0], -1, 1);
  u_ref[1] = constrain(u_ref[1], -1, 1);
  u_ref[2] = constrain(u_ref[2], -2, 2);
  u_ref[3] = constrain(u_ref[3], -4, 0);
  PWM_percent[0] = constrain(0.5 * u_ref[1] + 0.25 * u_ref[2] - 0.25 * u_ref[3], 0, 1);
  PWM_percent[1] = constrain(-0.5 * u_ref[0] - 0.25 * u_ref[2] - 0.25 * u_ref[3], 0, 1);
  PWM_percent[2] = constrain(-0.5 * u_ref[1] + 0.25 * u_ref[2] - 0.25 * u_ref[3], 0, 1);
  PWM_percent[3] = constrain(0.5 * u_ref[0] - 0.25 * u_ref[2] - 0.25 * u_ref[3], 0, 1);
  PWM[0] = 1000 + 1000 * PWM_percent[0];
  PWM[1] = 1000 + 1000 * PWM_percent[1];
  PWM[2] = 1000 + 1000 * PWM_percent[2];
  PWM[3] = 1000 + 1000 * PWM_percent[3];
  u[0] = PWM_percent[3] - PWM_percent[1];
  u[1] = PWM_percent[0] - PWM_percent[2];
  u[2] = PWM_percent[0] - PWM_percent[1] + PWM_percent[2] - PWM_percent[3];
  u[3] = -PWM_percent[0] - PWM_percent[1] - PWM_percent[2] - PWM_percent[3];
}
