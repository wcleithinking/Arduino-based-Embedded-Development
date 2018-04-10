void sample() {
  my_adxl345.accel_sample(&accel_x, &accel_y, &accel_z);
  my_itg3200.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
}
