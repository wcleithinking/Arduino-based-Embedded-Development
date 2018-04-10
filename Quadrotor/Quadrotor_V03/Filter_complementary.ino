void complementary_filter() {
  roll = 0.02 * roll_accel + 0.98 * (roll + gyro_x * dt);
  pitch = 0.02 * pitch_accel + 0.98 * (pitch + gyro_y * dt);
}
