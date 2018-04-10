#if defined(SONAR)
void altitude_filter() {
  if (loop_index < 100) hataltitude = sonar_hatz;
  else if (loop_index >= 100) {
    if (sonar_error_flag == 1) hataltitude = sonar_hatz;
    else {
      if (abs(hataltitude - altitude_d) <= 0.005 ) hataltitude = sonar_hatz;
      else hataltitude = (1 - SONAR_CPF) * hataltitude + SONAR_CPF * sonar_hatz;
    }
  }
  hataltitude = constrain(hataltitude, -SONAR_MAX * 0.01, 0);
}

#elif defined(BARO)
double Q_VAR[4] = {0, 0.008, 0, 0.00001};
double R_VAR = 0.18;
double z_state[4] = {hataltitude, hatv_z, accel_z_bias, hataltitude_bias};
double z_P[4][4] = {50, 0, 0, 0, 0, 50, 0, 0, 0, 0, 50, 0, 0, 0, 0, 50};

void altitude_filter() {
  if (loop_index < 100 ) {
  }
  else if ((loop_index >= 100) && (loop_index < 300)) {
    calculate_altitude();
    baro_altitude_bias += baro_hatz;
  }
  else if ((loop_index >= 300) && (loop_index < 350)) {
    if (loop_index == 300) baro_altitude_bias *= 0.005;
    calculate_altitude();
    baro_altitude = baro_hatz - baro_altitude_bias;
  }
  else {
    calculate_altitude();
    baro_altitude = (1 - BARO_CPF) * baro_altitude + BARO_CPF * (baro_hatz - baro_altitude_bias);
  }
  kalman_filter_altitude_v2();
  hataltitude = z_state[0] ;
  hatv_z = z_state[1];
  // accel_z_bias = z_state[2];
}

void kalman_filter_altitude_v1() {
  double prex[2], preP[2][2];
  double e, S, S_inv, K[2];
  double z_accel = G_ACCEL * (accel_z - accel_z_start);
  prex[0] = z_state[0] + dt * z_state[1];
  prex[1] = z_state[1] + dt * z_accel;
  preP[0][0] = z_P[0][0] + dt * z_P[0][1] + dt * z_P[1][0] + dt * dt * z_P[1][1] + Q_VAR[0] * dt * dt;
  preP[0][1] = z_P[0][1] + dt * z_P[1][1];
  preP[1][0] = z_P[1][0] + dt * z_P[1][1];
  preP[1][1] = z_P[1][1] + Q_VAR[1] * G_ACCEL * G_ACCEL * dt * dt;
  e = baro_altitude - prex[0];
  S = preP[0][0] + R_VAR;
  S_inv = 1.0 / S;
  K[0] = preP[0][0] * S_inv;
  K[1] = preP[1][0] * S_inv;
  z_state[0] = prex[0] + K[0] * e;
  z_state[1] = prex[1] + K[1] * e;
  z_P[0][0] = (1 - K[0]) * preP[0][0];
  z_P[0][1] = (1 - K[0]) * preP[0][1];
  z_P[1][0] = -K[1] * preP[0][0] + preP[1][0];
  z_P[1][1] = -K[1] * preP[0][1] + preP[1][1];
}

void kalman_filter_altitude_v2() {
  double prex[3], preP[3][3];
  double e, S, S_inv, K[3];
  prex[0] = z_state[0] + dt * z_state[1];
  prex[1] = z_state[1] - dt * G_ACCEL * z_state[2] + dt * G_ACCEL * (accel_z + 1);
  prex[2] = z_state[2];
  preP[0][0] = z_P[0][0] + dt * z_P[0][1] + dt * z_P[1][0] + dt * dt * z_P[1][1] + Q_VAR[0] * dt * dt;
  preP[0][1] = z_P[0][1] + dt * z_P[1][1] - dt * G_ACCEL * (z_P[0][2] + dt * z_P[1][2]);
  preP[0][2] = z_P[0][2] + dt * z_P[1][2];
  preP[1][0] = preP[0][1];
  preP[1][1] = z_P[1][1] - dt * G_ACCEL * (z_P[1][2] - dt * G_ACCEL * z_P[2][2]) - dt * G_ACCEL * z_P[2][1] + Q_VAR[1] * G_ACCEL * G_ACCEL * dt * dt;
  preP[1][2] = z_P[1][2] - dt * G_ACCEL * z_P[2][2];
  preP[2][0] = preP[0][2];
  preP[2][1] = preP[1][2];
  preP[2][2] = z_P[2][2] + Q_VAR[2] * dt * dt;
  e = baro_altitude - prex[0];
  S = preP[0][0] + R_VAR;
  S_inv = 1.0 / S;
  K[0] = preP[0][0] * S_inv;
  K[1] = preP[1][0] * S_inv;
  K[2] = preP[2][0] * S_inv;
  z_state[0] = prex[0] + K[0] * e;
  z_state[1] = prex[1] + K[1] * e;
  z_state[2] = prex[2] + K[2] * e;
  z_P[0][0] = (1 - K[0]) * preP[0][0];
  z_P[0][1] = (1 - K[0]) * preP[0][1];
  z_P[0][2] = (1 - K[0]) * preP[0][1];
  z_P[1][0] = -K[1] * preP[0][0] + preP[1][0];
  z_P[1][1] = -K[1] * preP[0][1] + preP[1][1];
  z_P[1][2] = -K[1] * preP[0][2] + preP[1][2];
  z_P[2][0] = -K[2] * preP[0][0] + preP[2][0];
  z_P[2][1] = -K[2] * preP[0][1] + preP[2][1];
  z_P[2][2] = -K[2] * preP[0][2] + preP[2][2];
}

void kalman_filter_altitude_v3() {
  double prex[4], preP[4][4];
  double e, S, S_inv, K[4];
  prex[0] = z_state[0] + dt * z_state[1];
  prex[1] = z_state[1] - dt * G_ACCEL * z_state[2] + dt * G_ACCEL * (accel_z + 1);
  prex[2] = z_state[2];
  prex[3] = z_state[3];
  // P1
  preP[0][0] = z_P[0][0] + dt * z_P[1][0] + dt * (z_P[0][1] + dt * z_P[1][1]) + Q_VAR[0] * dt * dt;
  preP[0][1] = z_P[0][1] + dt * z_P[1][1] - dt * G_ACCEL * (z_P[0][2] + dt * z_P[1][2]);
  preP[1][0] = z_P[1][0] + dt * (z_P[1][1] - dt * G_ACCEL * z_P[2][1]) - dt * G_ACCEL * z_P[2][0];
  preP[1][1] = z_P[1][1] - dt * G_ACCEL * (z_P[1][2] - dt * G_ACCEL * z_P[2][2]) - dt * G_ACCEL * z_P[2][1] + Q_VAR[1] * G_ACCEL * G_ACCEL * dt * dt;
  // P2
  preP[0][2] = z_P[0][2] + dt * z_P[1][2];
  preP[0][3] = z_P[0][3] + dt * z_P[1][3];
  preP[1][2] = z_P[1][2] - dt * G_ACCEL * z_P[2][2];
  preP[1][3] = z_P[1][3] - dt * G_ACCEL * z_P[2][3];
  // P3
  preP[2][0] = preP[0][2];
  preP[2][1] = preP[1][2];
  preP[3][0] = preP[0][3];
  preP[3][1] = preP[1][3];
  // P4
  preP[2][2] = z_P[2][2] + Q_VAR[2] * dt * dt;
  preP[2][3] = z_P[2][3];
  preP[3][2] = z_P[3][2];
  preP[3][3] = z_P[3][3] + Q_VAR[3] * dt * dt;
  e = baro_altitude - prex[0] - prex[3];
  S = preP[0][0] + preP[0][3] + preP[3][0] + preP[3][3] + R_VAR;
  S_inv = 1.0 / S;
  K[0] = (preP[0][0] + preP[0][3]) * S_inv;
  K[1] = (preP[1][0] + preP[1][3]) * S_inv;
  K[2] = (preP[2][0] + preP[2][3]) * S_inv;
  K[3] = (preP[3][0] + preP[3][3]) * S_inv;
  for (int i = 0; i < 4; i++) z_state[i] = prex[i] + K[i] * e;
  z_P[0][0] = - K[0] * preP[3][0] - preP[0][0] * (K[0] - 1);
  z_P[0][1] = - K[0] * preP[3][1] - preP[0][1] * (K[0] - 1);
  z_P[0][2] = - K[0] * preP[3][2] - preP[0][2] * (K[0] - 1);
  z_P[0][3] = - K[0] * preP[3][3] - preP[0][3] * (K[0] - 1);
  z_P[1][0] = z_P[0][1];
  z_P[1][1] = preP[1][1] - K[1] * preP[0][1] - K[1] * preP[3][1];
  z_P[1][2] = preP[1][2] - K[1] * preP[0][2] - K[1] * preP[3][2];
  z_P[1][3] = preP[1][3] - K[1] * preP[0][3] - K[1] * preP[3][3];
  z_P[2][0] = z_P[0][2];
  z_P[2][1] = z_P[1][2];
  z_P[2][2] = preP[2][2] - K[2] * preP[0][2] - K[2] * preP[3][2];
  z_P[2][3] = preP[2][3] - K[2] * preP[0][3] - K[2] * preP[3][3];
  z_P[3][0] = z_P[0][3];
  z_P[3][1] = z_P[1][3];
  z_P[3][2] = z_P[2][3];
  z_P[3][3] = - K[3] * preP[0][3] - preP[3][3] * (K[3] - 1);
}
#endif
