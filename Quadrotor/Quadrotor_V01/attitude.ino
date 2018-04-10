// for kalman filter
double F[2][2] = {1, -dt, 0, 1};
double Q[2][2] = {dt * dt, 0, 0, 0}, R = 0.01;
double roll_state[2] = {hatroll, gyro_x_bias};
double pitch_state[2] = {hatpitch, gyro_y_bias};
double roll_P[2][2] = {100, 0, 0, 100};
double pitch_P[2][2] = {100, 0, 0, 100};

void attitude() {
  accel_Euler();
  kalman_filter(roll_accel, gyro_x, 1);
  kalman_filter(pitch_accel, gyro_y, 2);
  hatroll = roll_state[0];
  hatpitch = pitch_state[0];
  gyro_x_bias = roll_state[1];
  gyro_y_bias = pitch_state[1];
}

void accel_Euler() {
  double tagroll, tagpitch;
  tagroll = accel_y / accel_z;
  tagpitch = accel_x / sqrt(accel_y * accel_y + accel_z * accel_z);
  if (tagroll > 100) {
    roll_accel = PI / 2 - 0.01;
  }
  else if (tagroll < -100) {
    roll_accel = -PI / 2 + 0.01;
  }
  else {
    roll_accel = atan(tagroll);
  }
  if (tagpitch > 100) {
    pitch_accel = PI / 2 - 0.01;
  }
  else if (tagpitch < -100) {
    pitch_accel = -PI / 2 + 0.01;
  }
  else {
    pitch_accel = atan(tagpitch);
  }
}

void kalman_filter(double angle_accel, double rate_gyro, int angle_flag) {
  double state[2], P[2][2];
  if (angle_flag == 1) {
    state[0] = roll_state[0];
    state[1] = roll_state[1];
    P[0][0] = roll_P[0][0];
    P[0][1] = roll_P[0][1];
    P[1][0] = roll_P[1][0];
    P[1][1] = roll_P[1][1];
  }
  else if (angle_flag == 2) {
    state[0] = pitch_state[0];
    state[1] = pitch_state[1];
    P[0][0] = pitch_P[0][0];
    P[0][1] = pitch_P[0][1];
    P[1][0] = pitch_P[1][0];
    P[1][1] = pitch_P[1][1];
  }
  double prestate[2], preP[2][2];
  double e, S, K[2];
  // predict
  prestate[0] = F[0][0] * state[0] + F[0][1] * state[1] + dt * rate_gyro;
  prestate[1] = F[1][0] * state[0] + F[1][1] * state[1];
  preP[0][0] = F[0][0] * F[0][0] * P[0][0] + 2 * F[0][0] * F[0][1] * P[0][1] + F[0][1] * F[0][1] * P[1][1] + Q[0][0];
  preP[0][1] = F[0][0] * F[1][1] * P[0][1] + F[0][1] * F[1][1] * P[1][1] + Q[0][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = F[1][1] * F[1][1] * P[1][1] + Q[1][1];
  // update
  e = angle_accel - prestate[0];
  S = preP[0][0] + R;
  K[0] = preP[0][0] / S;
  K[1] = preP[0][1] / S;
  state[0] = prestate[0] + K[0] * e;
  state[1] = prestate[1] + K[1] * e;
  P[0][0] = (1 - K[0]) * preP[0][0];
  P[0][1] = (1 - K[0]) * preP[0][1];
  P[1][0] = P[0][1];
  P[1][1] = -K[1] * preP[0][1] + preP[1][1];
  if (angle_flag == 1) {
    roll_state[0] = state[0];
    roll_state[1] = state[1];
    roll_P[0][0] = P[0][0];
    roll_P[0][1] = P[0][1];
    roll_P[1][0] = P[1][0];
    roll_P[1][1] = P[1][1];
  }
  else if (angle_flag == 2) {
    pitch_state[0] = state[0];
    pitch_state[1] = state[1];
    pitch_P[0][0] = P[0][0];
    pitch_P[0][1] = P[0][1];
    pitch_P[1][0] = P[1][0];
    pitch_P[1][1] = P[1][1];
  }
}
