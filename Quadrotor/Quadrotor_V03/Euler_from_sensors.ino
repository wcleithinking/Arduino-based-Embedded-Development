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

void gyro_Euler() {
  roll_gyro += (gyro_x - gyro_x_bias) * dt;
  pitch_gyro += (gyro_y - gyro_y_bias) * dt;
  yaw_gyro += (gyro_z - gyro_z_bias) * dt;
}
