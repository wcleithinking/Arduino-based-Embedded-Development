double roll_accel, pitch_accel;
double roll_state[2] = {hatroll, gyro_x_bias};
double pitch_state[2] = {hatpitch, gyro_y_bias};
double yaw_state[5] = {hatyaw, gyro_z_bias, M_north, mag_x_bias, mag_y_bias};
double roll_P[2][2] = {100, 0, 0, 100};
double pitch_P[2][2] = {100, 0, 0, 100};
double yaw_P[5][5] = {100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100};

void attitude_filter() {
  accel_rollpitch();
  kalman_filter_roll();
  kalman_filter_pitch();
  kalman_filter_yaw();
  hatroll = roll_state[0];
  hatpitch = pitch_state[0];
  hatyaw = yaw_state[0];
  gyro_x_bias = roll_state[1];
  gyro_y_bias = pitch_state[1];
  gyro_z_bias = yaw_state[1];
  M_north = yaw_state[2];
  mag_x_bias = yaw_state[3];
  mag_y_bias = yaw_state[4];
}

void complementary_filter_rollpitch() {
    accel_rollpitch();
    hatroll = 0.98 * (hatroll  + gyro_x * dt) + 0.02 * roll_accel;
    hatpitch  = 0.98 * (hatpitch  + gyro_y * dt)  + 0.02 * pitch_accel;
}

void accel_rollpitch() {
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

void kalman_filter_roll() {
  double prestate[2], preP[2][2];
  double e, S, K[2];
  // predict
  prestate[0] = roll_state[0] - dt * roll_state[1] + dt * gyro_x;
  prestate[1] = roll_state[1];
  preP[0][0] = roll_P[0][0] - 2 * dt * roll_P[0][1] + dt * dt * roll_P[1][1] + dt * dt;
  preP[0][1] = roll_P[0][1] - dt * roll_P[1][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = roll_P[1][1];
  // update
  e = roll_accel - prestate[0];
  S = preP[0][0] + 0.01;
  K[0] = preP[0][0] / S;
  K[1] = preP[0][1] / S;
  roll_state[0] = prestate[0] + K[0] * e;
  roll_state[1] = prestate[1] + K[1] * e;
  roll_P[0][0] = (1 - K[0]) * preP[0][0];
  roll_P[0][1] = (1 - K[0]) * preP[0][1];
  roll_P[1][0] = roll_P[0][1];
  roll_P[1][1] = -K[1] * preP[0][1] + preP[1][1];
}

void kalman_filter_pitch() {
  double prestate[2], preP[2][2];
  double e, S, K[2];
  // predict
  prestate[0] = pitch_state[0] - dt * pitch_state[1] + dt * gyro_y;
  prestate[1] = pitch_state[1];
  preP[0][0] = pitch_P[0][0] - 2 * dt * pitch_P[0][1] + dt * dt * pitch_P[1][1] + dt * dt;
  preP[0][1] = pitch_P[0][1] - dt * pitch_P[1][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = pitch_P[1][1];
  // update
  e = pitch_accel - prestate[0];
  S = preP[0][0] + 0.01;
  K[0] = preP[0][0] / S;
  K[1] = preP[0][1] / S;
  pitch_state[0] = prestate[0] + K[0] * e;
  pitch_state[1] = prestate[1] + K[1] * e;
  pitch_P[0][0] = (1 - K[0]) * preP[0][0];
  pitch_P[0][1] = (1 - K[0]) * preP[0][1];
  pitch_P[1][0] = roll_P[0][1];
  pitch_P[1][1] = -K[1] * preP[0][1] + preP[1][1];
}

void kalman_filter_yaw() {
  double det, det_inv, prex[5], e[2];
  double preP[5][5], h[2][2], S[2][2], S_inv[2][2], temp[5][5], K[5][2];
  // predict
  prex[0] = yaw_state[0] + dt * (gyro_z - yaw_state[1]);
  for (int i = 1; i < 5; i++) prex[i] = yaw_state[i];
  preP[0][0] = yaw_P[0][0] - 2 * dt * yaw_P[0][1] + dt * dt * yaw_P[1][1] + 1 * dt * dt;
  preP[0][1] = yaw_P[0][1] - dt * yaw_P[1][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = yaw_P[1][1];
  for (int i = 2; i < 5; i++) {
    preP[0][i] = yaw_P[0][i] - dt * yaw_P[1][i];
    preP[1][i] = yaw_P[1][i];
    preP[i][0] = preP[0][i];
    preP[i][1] = preP[1][i];
    for (int j = 2; j < 5; j++) preP[i][j] = yaw_P[i][j];
  }
  // update
  h[0][0] = -prex[2] * sin(prex[0]);
  h[0][1] = cos(prex[0]);
  h[1][0] = -prex[2] * cos(prex[0]);
  h[1][1] = -sin(prex[0]);
  e[0] = mag_x - (prex[2] * cos(prex[0]) + prex[3]);
  e[1] = mag_y - (-prex[2] * sin(prex[0]) + prex[4]);
  S[0][0] = (h[0][0] * preP[0][0] + h[0][1] * preP[2][0]) * h[0][0] + (h[0][0] * preP[0][2] + h[0][1] * preP[2][2]) * h[0][1] + 2 * (h[0][0] * preP[0][3] + h[0][1] * preP[2][3]) + preP[3][3] + 1.5;
  S[0][1] = (h[0][0] * preP[0][0] + h[0][1] * preP[2][0]) * h[1][0] + (h[0][0] * preP[0][2] + h[0][1] * preP[2][2]) * h[1][1];
  S[0][1] += h[0][0] * preP[0][4] + h[0][1] * preP[2][4] + h[1][0] * preP[0][3] + h[1][1] * preP[2][3] + preP[3][4];
  S[1][0] = S[0][1];
  S[1][1] = (h[1][0] * preP[0][0] + h[1][1] * preP[2][0]) * h[1][0] + (h[1][0] * preP[0][2] + h[1][1] * preP[2][2]) * h[1][1] + 2 * (h[1][0] * preP[0][4] + h[1][1] * preP[2][4]) + preP[4][4] + 1.5;
  det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
  det_inv = 1 / det;
  S_inv[0][0] = S[1][1] * det_inv;
  S_inv[0][1] = -S[0][1] * det_inv;
  S_inv[1][0] = -S[1][0] * det_inv;
  S_inv[1][1] = S[0][0] * det_inv;
  for (int i = 0; i < 5; i++) {
    temp[i][0] = preP[i][0] * h[0][0] + preP[i][2] * h[0][1] + preP[i][3];
    temp[i][1] = preP[i][0] * h[1][0] + preP[i][2] * h[1][1] + preP[i][4];
  }
  for (int i = 0; i < 5; i++) {
    K[i][0] = temp[i][0] * S_inv[0][0] + temp[i][1] * S_inv[1][0];
    K[i][1] = temp[i][0] * S_inv[0][1] + temp[i][1] * S_inv[1][1];
  }
  for (int i = 0; i < 5; i++) {
    yaw_state[i] = prex[i];
    for (int j = 0; j < 2; j++) yaw_state[i] += K[i][j] * e[j];
  }
  for (int i = 0; i < 5; i++) {
    temp[i][0] = -(K[i][0] * h[0][0] + K[i][1] * h[1][0]);
    temp[i][1] = 0;
    temp[i][2] = -(K[i][0] * h[0][1] + K[i][1] * h[1][1]);
    temp[i][3] = -K[i][0];
    temp[i][4] = -K[i][1];
  }
  for (int i = 0; i < 5; i++) temp[i][i] = 1 + temp[i][i];
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      yaw_P[i][j] = 0;
      for (int k = 0; k < 5; k++) yaw_P[i][j] += temp[i][k] * preP[k][j];
    }
  }
}
