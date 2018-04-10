// for kalman filter
double F[2][2] = {1, -dt, 0, 1};
double Q[2][2] = {dt*dt, 0, 0, 0}, R = 0.01;
double roll_P[2][2] = {100, 0, 0, 100};
double pitch_P[2][2] = {100, 0, 0, 100};

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

void kalman_filter_yaw() {
  double det, det_inv, prex[5], e[2];
  double preP[5][5], h[2][2], S[2][2], S_inv[2][2], temp[5][5], K[5][2];
  // predict
  prex[0] = state2[0] + dt * (gyro_z - state2[1]);
  for (int i = 1; i < 5; i++) prex[i] = state2[i];
  preP[0][0] = P2[0][0] - 2 * dt * P2[0][1] + dt * dt * P2[1][1] + dt * dt;
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
  S[0][0] = (h[0][0] * preP[0][0] + h[0][1] * preP[2][0]) * h[0][0] + (h[0][0] * preP[0][2] + h[0][1] * preP[2][2]) * h[0][1] + 2 * (h[0][0] * preP[0][3] + h[0][1] * preP[2][3]) +preP[3][3]+ 1;
  S[0][1] = (h[0][0] * preP[0][0] + h[0][1] * preP[2][0]) * h[1][0] + (h[0][0] * preP[0][2] + h[0][1] * preP[2][2]) * h[1][1];
  S[0][1] += h[0][0] * preP[0][4] + h[0][1] * preP[2][4] + h[1][0] * preP[0][3] + h[1][1] * preP[2][3]+preP[3][4];
  S[1][0] = S[0][1];
  S[1][1] = (h[1][0] * preP[0][0] + h[1][1] * preP[2][0]) * h[1][0] + (h[1][0] * preP[0][2] + h[1][1] * preP[2][2]) * h[1][1] + 2 * (h[1][0] * preP[0][4] + h[1][1] * preP[2][4]) +preP[4][4]+ 1;
  det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
  det_inv = 1/det;
  S_inv[0][0] = S[1][1] *det_inv;
  S_inv[0][1] = -S[0][1] *det_inv;
  S_inv[1][0] = -S[1][0] *det_inv;
  S_inv[1][1] = S[0][0] *det_inv;
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
