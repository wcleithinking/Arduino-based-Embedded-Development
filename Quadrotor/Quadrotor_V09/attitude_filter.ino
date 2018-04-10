float M_north = 40;
float roll_accel, pitch_accel;
float roll_state[2] = {hatroll, gyro_x_bias};
float pitch_state[2] = {hatpitch, gyro_y_bias};
float yaw_state[4] = {hatyaw, M_north, mag_x_bias, mag_y_bias};
float roll_P[2][2] = {100, 0,
                      0, 100
                     };
float pitch_P[2][2] = {100, 0,
                       0, 100
                      };
float yaw_P[4][4] = {100, 0, 0, 0,
                     0, 100, 0, 0,
                     0, 0, 100, 0,
                     0, 0, 0, 100
                    };

void attitude_filter() {
  accel_rollpitch();
  kalman_filter_roll();
  kalman_filter_pitch();
  kalman_filter_yaw();
  hatroll = roll_state[0];
  gyro_x_bias = roll_state[1];
  hatpitch = pitch_state[0];
  gyro_y_bias = pitch_state[1];
  hatyaw = yaw_state[0];
}

void accel_rollpitch() {
  float tagroll, tagpitch;
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
  float prestate[2], preP[2][2];
  float e, S, K[2];
  // predict
  prestate[0] = roll_state[0] - dt * roll_state[1] + dt * gyro_x;
  prestate[1] = roll_state[1];
  preP[0][0] = roll_P[0][0] - 2 * dt * roll_P[0][1] + dt * dt * roll_P[1][1] +  dt * dt ;
  preP[0][1] = roll_P[0][1] - dt * roll_P[1][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = roll_P[1][1];
  // update
  e = roll_accel - prestate[0];
  S = preP[0][0] + 1;
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
  float prestate[2], preP[2][2];
  float e, S, K[2];
  // predict
  prestate[0] = pitch_state[0] - dt * pitch_state[1] + dt * gyro_y;
  prestate[1] = pitch_state[1];
  preP[0][0] = pitch_P[0][0] - 2 * dt * pitch_P[0][1] + dt * dt * pitch_P[1][1] +  dt * dt ;
  preP[0][1] = pitch_P[0][1] - dt * pitch_P[1][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = pitch_P[1][1];
  // update
  e = pitch_accel - prestate[0];
  S = preP[0][0] + 1;
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
  float prex[4], preP[4][4];
  float e[2], h[2][2], S[2][2], S_inv[2][2], detS_inv, temp[4][4], K[4][2];
  // predict
  prex[0] = yaw_state[0] + dt * gyro_z;
  prex[1] = yaw_state[1];
  prex[2] = yaw_state[2];
  prex[3] = yaw_state[3];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) preP[i][j] = yaw_P[i][j];
  }
  preP[0][0] += dt * dt;
  // update
  h[0][0] = -prex[1] * sin(prex[0]);
  h[0][1] = cos(prex[0]);
  h[1][0] = -prex[1] * cos(prex[0]);
  h[1][1] = -sin(prex[0]);
  e[0] = mag_x - (prex[1] * cos(prex[0]) + prex[2]);
  e[1] = mag_y - (-prex[1] * sin(prex[0]) + prex[3]);
  temp[0][0] = preP[2][0] + h[0][0] * preP[0][0] + h[0][1] * preP[1][0];
  temp[0][1] = preP[2][1] + h[0][0] * preP[0][1] + h[0][1] * preP[1][1];
  S[0][0] = preP[2][2] + h[0][0] * (temp[0][0]) + h[0][1] * (temp[0][1]) + h[0][0] * preP[0][2] + h[0][1] * preP[1][2] + 1;
  S[0][1] = preP[2][3] + h[1][0] * (temp[0][0]) + h[1][1] * (temp[0][1]) + h[0][0] * preP[0][3] + h[0][1] * preP[1][3];
  S[1][0] = S[0][1];
  S[1][1] = preP[3][3] + h[1][0] * (preP[3][0] + h[1][0] * preP[0][0] + h[1][1] * preP[1][0])
            + h[1][1] * (preP[3][1] + h[1][0] * preP[0][1] + h[1][1] * preP[1][1]) + h[1][0] * preP[0][3] + h[1][1] * preP[1][3] + 1;
  detS_inv = 1.0f / (S[0][0] * S[1][1] - S[0][1] * S[1][0]);
  S_inv[0][0] =  S[1][1] * detS_inv;
  S_inv[0][1] = -S[0][1] * detS_inv;
  S_inv[1][0] = -S[1][0] * detS_inv;
  S_inv[1][1] =  S[0][0] * detS_inv;
  for (int i = 0; i < 4; i++) {
    temp[i][0] = preP[i][2] + preP[i][0] * h[0][0] + preP[i][1] * h[0][1];
    temp[i][1] = preP[i][3] + preP[i][0] * h[1][0] + preP[i][1] * h[1][1];
  }
  for (int i = 0; i < 4; i++) {
    K[i][0] = temp[i][0] * S_inv[0][0] + temp[i][1] * S_inv[1][0];
    K[i][1] = temp[i][0] * S_inv[0][1] + temp[i][1] * S_inv[1][1];
  }
  for (int i = 0; i < 4; i++) {
    yaw_state[i] = prex[i] + K[i][0] * e[0] + K[i][1] * e[1];
  }
  for (int i = 0; i < 4; i++) {
    temp[i][0] = -(K[i][0] * h[0][0] + K[i][1] * h[1][0]);
    temp[i][1] = -(K[i][0] * h[0][1] + K[i][1] * h[1][1]);
    temp[i][2] = -K[i][0];
    temp[i][3] = -K[i][1];
  }
  for (int i = 0; i < 4; i++) {
    temp[i][i] = 1 + temp[i][i];
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      yaw_P[i][j] = 0;
      for (int k = 0; k < 4; k++) yaw_P[i][j] += temp[i][k] * preP[k][j];
    }
  }
}
