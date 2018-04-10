#include "IMU.h"

IMU::IMU() {
  // initialize the covariance matrix
  for (int i = 0; i < 2; i++) {
    roll_state[i] = 0;
    pitch_state[i] = 0;
    for (int j = 0; j < 2; j++) {
      if (i == j) {
        roll_P[i][j] = 100;
        pitch_P[i][j] = 100;
      }
      else {
        roll_P[i][j] = 0;
        pitch_P[i][j] = 0;
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    yaw_state[i] = 0;
    for (int j = 0; j < 4; j++) {
      if (i == j) yaw_P[i][j] = 100;
      else yaw_P[i][j] = 0;
    }
  }
  yaw_state[1] = 20;
}

void IMU::init(uint8_t AFS_SEL, uint8_t ADLPF_CFG, uint8_t GFS_SEL, uint8_t GDLPF_CFG, uint8_t MSS_SEL, float dt) {
  Wire.begin();
  mySensor.all_config(AFS_SEL, ADLPF_CFG, GFS_SEL, GDLPF_CFG, MSS_SEL);
  period = dt;
}

void IMU::sample() {
  mySensor.accel_sample(&accel_x, &accel_y, &accel_z);
  mySensor.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
  mySensor.mag_sample(&mag_x, &mag_y, &mag_z);
}

void IMU::copyaccel(float Accel[3]) {
  Accel[0] = accel_x;
  Accel[1] = accel_y;
  Accel[2] = accel_z;
}

void IMU::copygyro(float Gyro[3]) {
  Gyro[0] = gyro_x;
  Gyro[1] = gyro_y;
  Gyro[2] = gyro_z;
}

void IMU::copymag(float Mag[3]) {
  Mag[0] = mag_x;
  Mag[1] = mag_y;
  Mag[2] = mag_z;
}

void IMU::attitude_filter(float Angle_estimate[3], float Gyro_bias_estimate[3]) {
  accel_rollpitch();
  mag_yaw();
  kalman_filter_roll();
  kalman_filter_pitch();
  kalman_filter_yaw();
  Angle_estimate[0] = roll_state[0];
  Angle_estimate[1] = pitch_state[0];
  //Angle_estimate[2] = yaw_state[0];
  Angle_estimate[2] = yaw_mag;
  Gyro_bias_estimate[0] = roll_state[1];
  Gyro_bias_estimate[1] = pitch_state[1];
  Gyro_bias_estimate[2] = 0;
}

void IMU::accel_rollpitch() {
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

void IMU::mag_yaw() {
  float tagyaw;
  float My = -(mag_y*cos(roll_state[0])-mag_z*sin(roll_state[0]));
  float Mx = (mag_x*cos(pitch_state[0])+mag_y*sin(roll_state[0])*sin(pitch_state[0])+mag_z*cos(roll_state[0])*sin(pitch_state[0]));
  tagyaw =  My/Mx;
  //yaw_mag = atan2(My,Mx);
  if (Mx>0) { 
    yaw_mag = atan(tagyaw);
  }
  else if ((Mx==0) && (My>0)) {
    yaw_mag = PI/2;
  }
  else if ((Mx<0) && (My>=0)) {
    yaw_mag = atan(tagyaw) + PI;
  }
  else if ((Mx<0) && (My<0)) {
    yaw_mag = atan(tagyaw) - PI;
  } 
  else if ((Mx==0) && (My<0)) {
    yaw_mag = -PI/2;
  }
}

void IMU::kalman_filter_roll() {
  float prestate[2], preP[2][2];
  float e, S, K[2];
  // predict
  prestate[0] = roll_state[0] - period * roll_state[1] + period * gyro_x;
  prestate[1] = roll_state[1];
  preP[0][0] = roll_P[0][0] - 2 * period * roll_P[0][1] + period * period * roll_P[1][1] +  period * period ;
  preP[0][1] = roll_P[0][1] - period * roll_P[1][1];
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

void IMU::kalman_filter_pitch() {
  float prestate[2], preP[2][2];
  float e, S, K[2];
  // predict
  prestate[0] = pitch_state[0] - period * pitch_state[1] + period * gyro_y;
  prestate[1] = pitch_state[1];
  preP[0][0] = pitch_P[0][0] - 2 * period * pitch_P[0][1] + period * period * pitch_P[1][1] +  period * period ;
  preP[0][1] = pitch_P[0][1] - period * pitch_P[1][1];
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
  pitch_P[1][0] = pitch_P[0][1];
  pitch_P[1][1] = -K[1] * preP[0][1] + preP[1][1];
}

void IMU::kalman_filter_yaw() {
  float prestate[2], preP[2][2];
  float e, S, K[2];
  // predict
  prestate[0] = yaw_state[0] - period * yaw_state[1] + period * gyro_z;
  prestate[1] = yaw_state[1];
  preP[0][0] = yaw_P[0][0] - 2 * period * yaw_P[0][1] + period * period * yaw_P[1][1] +  period * period ;
  preP[0][1] = yaw_P[0][1] - period * yaw_P[1][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = yaw_P[1][1];
  // update
  e = yaw_mag - prestate[0];
  S = preP[0][0] + 1;
  K[0] = preP[0][0] / S;
  K[1] = preP[0][1] / S;
  yaw_state[0] = prestate[0] + K[0] * e;
  yaw_state[1] = prestate[1] + K[1] * e;
  yaw_P[0][0] = (1 - K[0]) * preP[0][0];
  yaw_P[0][1] = (1 - K[0]) * preP[0][1];
  yaw_P[1][0] = yaw_P[0][1];
  yaw_P[1][1] = -K[1] * preP[0][1] + preP[1][1];
}

// void IMU::kalman_filter_yaw() {
//   float prex[4], preP[4][4];
//   float e[2], h[2][2], S[2][2], S_inv[2][2], detS_inv, temp[4][4], K[4][2];
//   // predict
//   prex[0] = yaw_state[0] + period * gyro_z;
//   prex[1] = yaw_state[1];
//   prex[2] = yaw_state[2];
//   prex[3] = yaw_state[3];
//   for (int i = 0; i < 4; i++) {
//     for (int j = 0; j < 4; j++) preP[i][j] = yaw_P[i][j];
//   }
// preP[0][0] += period * period;
//   // update
// h[0][0] = -prex[1] * sin(prex[0]);
// h[0][1] = cos(prex[0]);
// h[1][0] = -prex[1] * cos(prex[0]);
// h[1][1] = -sin(prex[0]);
// e[0] = mag_x - (prex[1] * cos(prex[0]) + prex[2]);
// e[1] = mag_y - (-prex[1] * sin(prex[0]) + prex[3]);
// temp[0][0] = preP[2][0] + h[0][0] * preP[0][0] + h[0][1] * preP[1][0];
// temp[0][1] = preP[2][1] + h[0][0] * preP[0][1] + h[0][1] * preP[1][1];
// S[0][0] = preP[2][2] + h[0][0] * (temp[0][0]) + h[0][1] * (temp[0][1]) + h[0][0] * preP[0][2] + h[0][1] * preP[1][2] + 1;
// S[0][1] = preP[2][3] + h[1][0] * (temp[0][0]) + h[1][1] * (temp[0][1]) + h[0][0] * preP[0][3] + h[0][1] * preP[1][3];
// S[1][0] = S[0][1];
// S[1][1] = preP[3][3] + h[1][0] * (preP[3][0] + h[1][0] * preP[0][0] + h[1][1] * preP[1][0])
// + h[1][1] * (preP[3][1] + h[1][0] * preP[0][1] + h[1][1] * preP[1][1]) + h[1][0] * preP[0][3] + h[1][1] * preP[1][3] + 1;
// detS_inv = 1.0f / (S[0][0] * S[1][1] - S[0][1] * S[1][0]);
// S_inv[0][0] =  S[1][1] * detS_inv;
// S_inv[0][1] = -S[0][1] * detS_inv;
// S_inv[1][0] = -S[1][0] * detS_inv;
// S_inv[1][1] =  S[0][0] * detS_inv;
// for (int i = 0; i < 4; i++) {
//   temp[i][0] = preP[i][2] + preP[i][0] * h[0][0] + preP[i][1] * h[0][1];
//   temp[i][1] = preP[i][3] + preP[i][0] * h[1][0] + preP[i][1] * h[1][1];
// }
// for (int i = 0; i < 4; i++) {
//   K[i][0] = temp[i][0] * S_inv[0][0] + temp[i][1] * S_inv[1][0];
//   K[i][1] = temp[i][0] * S_inv[0][1] + temp[i][1] * S_inv[1][1];
// }
// for (int i = 0; i < 4; i++) {
//   yaw_state[i] = prex[i] + K[i][0] * e[0] + K[i][1] * e[1];
// }
// for (int i = 0; i < 4; i++) {
//   temp[i][0] = -(K[i][0] * h[0][0] + K[i][1] * h[1][0]);
//   temp[i][1] = -(K[i][0] * h[0][1] + K[i][1] * h[1][1]);
//   temp[i][2] = -K[i][0];
//   temp[i][3] = -K[i][1];
// }
// for (int i = 0; i < 4; i++) {
//   temp[i][i] = 1 + temp[i][i];
// }
// for (int i = 0; i < 4; i++) {
//   for (int j = 0; j < 4; j++) {
//     yaw_P[i][j] = 0;
//     for (int k = 0; k < 4; k++) yaw_P[i][j] += temp[i][k] * preP[k][j];
//   }
// }
// }
