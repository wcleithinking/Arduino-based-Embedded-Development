void sensor_sample() {
  my_mpu9250.accel_sample(&accel_x, &accel_y, &accel_z);
  my_mpu9250.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
  my_mpu9250.mag_sample(&mag_x, &mag_y, &mag_z);
}

