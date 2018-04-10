void sensor_config() {
  my_mpu9250.acceltempgyro_config();
  delay(100);
  my_mpu9250.mag_config();
  delay(100);
}


