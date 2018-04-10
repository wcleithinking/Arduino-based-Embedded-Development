double state1[7] = {roll, pitch, gyro_x_bias, gyro_y_bias, accel_x_bias, accel_y_bias, accel_z_bias};
double state2[5] = {yaw, gyro_z_bias, M_north, mag_x_bias, mag_y_bias};
double P1[7][7] = {100, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 100};
double P2[5][5] = {100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100};
double F[7][7] = {1, 0, -dt, 0, 0, 0, 0, 0, 1, 0, -dt, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1};
void attitude_filter() {
  kalman_filter_rollpitch();
  roll = state1[0];
  pitch = state1[1];
  gyro_x_bias = state1[2];
  gyro_y_bias = state1[3];
  accel_x_bias = state1[4];
  accel_y_bias = state1[5];
  accel_z_bias = state1[6];
  kalman_filter_yaw();
  yaw = state2[0];
  gyro_z_bias = state2[1];
  M_north = state2[2];
  mag_x_bias = state2[3];
  mag_y_bias = state2[4];
}

/*************************************************************************************************************
  Roll angle and Pitch angle
 *************************************************************************************************************/
void kalman_filter_rollpitch() {
  double det, det_inv, prex[7], e[3];
  double preP[7][7], H[3][7], temp[7][7], S[3][3], S_inv[3][3], K[7][3];
  // predict
  prex[0] = state1[0] + dt * ( (gyro_x - state1[2]) + sin(state1[0]) * tan(state1[1]) * (gyro_y - state1[3]) + cos(state1[0]) * tan(state1[1]) * (gyro_z - state2[1]) );
  prex[1] = state1[1] + dt * ( (gyro_y - state1[3]) * cos(state1[0]) - (gyro_z - state2[1]) * sin(state1[1]) );
  for (int i = 2; i < 7; i++) prex[i] = state1[i];
  for (int i = 0; i < 7; i++) {
    for (int k = 0; k < 7; k++) {
      temp[i][k] = 0;
      for (int j = 0; j < 7; j++) temp[i][k] += F[i][j] * P1[j][k];
    }
  }
  for (int i = 0; i < 7; i++) {
    for (int k = 0; k < 7; k++) {
      preP[i][k] = 0;
      for (int j = 0; j < 7; j++) preP[i][k] += temp[i][j] * F[k][j];
    }
  }
  preP[0][0] += dt * dt;
  preP[1][1] += dt * dt;
  // update
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 7; j++) H[i][j] = 0;
  }
  H[0][1] = cos(prex[1]);
  H[0][4] = 1;
  H[1][0] = -cos(prex[0]) * cos(prex[1]);
  H[1][1] = sin(prex[0]) * sin(prex[1]);
  H[1][5] = 1;
  H[2][0] = sin(prex[0]) * cos(prex[1]);
  H[2][1] = cos(prex[0]) * sin(prex[1]);
  H[2][6] = 1;
  e[0] = accel_x - (sin(prex[1]) + prex[4]);
  e[1] = accel_y - (-sin(prex[0] * cos(prex[1])) + prex[5]);
  e[2] = accel_z - (-cos(prex[0] * cos(prex[1])) + prex[6]);
  for (int i = 0; i < 3; i++) {
    for (int k = 0; k < 7; k++) {
      temp[i][k] = 0;
      for (int j = 0; j < 7; j++) temp[i][k] += H[i][j] * preP[j][k];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int k = 0; k < 3; k++) {
      S[i][k] = 0;
      for (int j = 0; j < 7; j++) S[i][k] += temp[i][j] * H[k][j];
    }
  }
  for (int i = 0; i < 3; i++) S[i][i] += 0.01;
  det = S[0][0] * S[1][1] * S[2][2] + S[0][1] * S[1][2] * S[2][0] + S[0][2] * S[1][0] * S[2][1] - S[0][2] * S[1][1] * S[2][0] - S[0][1] * S[1][0] * S[2][2] - S[0][0] * S[1][2] * S[2][1];
  det_inv = 1 / det;
  S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * det_inv;
  S_inv[0][1] = -(S[0][1] * S[2][2] - S[0][2] * S[2][1]) * det_inv;
  S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * det_inv;
  S_inv[1][0] = -(S[1][0] * S[2][2] - S[1][2] * S[2][0]) * det_inv;
  S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * det_inv;
  S_inv[1][2] = -(S[0][0] * S[1][2] - S[0][2] * S[1][0]) * det_inv;
  S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * det_inv;
  S_inv[2][1] = -(S[0][0] * S[2][1] - S[0][1] * S[2][0]) * det_inv;
  S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * det_inv;
  for (int i = 0; i < 7; i++) {
    for (int k = 0; k < 3; k++) {
      temp[i][k] = 0;
      for (int j = 0; j < 7; j++) temp[i][k] += preP[i][j] * H[k][j];
    }
  }
  for (int i = 0; i < 7; i++) {
    for (int k = 0; k < 3; k++) {
      K[i][k] = 0;
      for (int j = 0; j < 3; j++) K[i][k] += temp[i][j] * S_inv[j][k];
    }
  }
  for (int i = 0; i < 7; i++) {
    state1[i] = prex[i];
    for (int j = 0; j < 3; j++) state1[i] += K[i][j] * e[j];
  }
  for (int i = 0; i < 7; i++) {
    for (int k = 0; k < 7; k++) {
      temp[i][k] = 0;
      for (int j = 0; j < 3; j++) temp[i][k] += K[i][j] * H[j][k];
    }
  }
  for (int i = 0; i < 7; i++) {
    for (int k = 0; k < 7; k++) {
      P1[i][k] = 0;
      for (int j = 0; j < 7; j++) P1[i][k] += temp[i][j] * preP[j][k];
      P1[i][k] = preP[i][k] - P1[i][k];
    }
  }
}
/*****************************************************************************************************
   Yaw angle
*****************************************************************************************************/
void kalman_filter_yaw() {
  double det, det_inv, prex[5], e[2];
  double preP[5][5], h[2][2], S[2][2], S_inv[2][2], temp[5][5], K[5][2];
  // predict
  prex[0] = state2[0] + dt * ( sin(state1[0]) / cos(state1[1]) * (gyro_y - state1[3]) + cos(state1[0]) / cos(state1[1]) * (gyro_z - state2[1]) );
  for (int i = 1; i < 5; i++) prex[i] = state2[i];
  preP[0][0] = P2[0][0] - 2 * dt * P2[0][1] + dt * dt * P2[1][1] + 1 * dt * dt;
  preP[0][1] = P2[0][1] - dt * P2[1][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = P2[1][1];
  for (int i = 2; i < 5; i++) {
    preP[0][i] = P2[0][i] - dt * P2[1][i];
    preP[1][i] = P2[1][i];
    preP[i][0] = preP[0][i];
    preP[i][1] = preP[1][i];
    for (int j = 2; j < 5; j++) preP[i][j] = P2[i][j];
  }
  // update
  h[0][0] = -prex[2] * sin(prex[0]);
  h[0][1] = cos(prex[0]);
  h[1][0] = -prex[2] * cos(prex[0]);
  h[1][1] = -sin(prex[0]);
  e[0] = mag_x - (prex[2] * cos(prex[0]) + prex[3]);
  e[1] = mag_y - (-prex[2] * sin(prex[0]) + prex[4]);
  S[0][0] = (h[0][0] * preP[0][0] + h[0][1] * preP[2][0]) * h[0][0] + (h[0][0] * preP[0][2] + h[0][1] * preP[2][2]) * h[0][1] + 2 * (h[0][0] * preP[0][3] + h[0][1] * preP[2][3]) + preP[3][3] + 0.01;
  S[0][1] = (h[0][0] * preP[0][0] + h[0][1] * preP[2][0]) * h[1][0] + (h[0][0] * preP[0][2] + h[0][1] * preP[2][2]) * h[1][1];
  S[0][1] += h[0][0] * preP[0][4] + h[0][1] * preP[2][4] + h[1][0] * preP[0][3] + h[1][1] * preP[2][3] + preP[3][4];
  S[1][0] = S[0][1];
  S[1][1] = (h[1][0] * preP[0][0] + h[1][1] * preP[2][0]) * h[1][0] + (h[1][0] * preP[0][2] + h[1][1] * preP[2][2]) * h[1][1] + 2 * (h[1][0] * preP[0][4] + h[1][1] * preP[2][4]) + preP[4][4] + 0.01;
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
    state2[i] = prex[i];
    for (int j = 0; j < 2; j++) state2[i] += K[i][j] * e[j];
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
      P2[i][j] = 0;
      for (int k = 0; k < 5; k++) P2[i][j] += temp[i][k] * preP[k][j];
    }
  }
}
