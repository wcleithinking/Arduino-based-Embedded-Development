void sensor_config() {
  my_mpu9250.accelgyro_config();
  delay(100);
  my_mpu9250.mag_config();
  delay(100);
}
