void sensor_config() {
  my_mpu9250.acceltempgyro_config(3,4,3,4);
  delay(100);
  my_mpu9250.mag_config(1);
  delay(100);
}

