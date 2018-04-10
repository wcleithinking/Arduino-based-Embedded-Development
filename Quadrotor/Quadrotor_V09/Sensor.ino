void Sensor() {
  Wire.begin();
  my_mpu9250.acceltempgyro_config(3, 4, 3, 4);
  delay(100);
  my_mpu9250.mag_config(1);
  delay(100);
}

void sensor_sample() {
  my_mpu9250.accel_sample(&accel_x, &accel_y, &accel_z);
  my_mpu9250.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
  my_mpu9250.mag_sample(&mag_x, &mag_y, &mag_z);
}
