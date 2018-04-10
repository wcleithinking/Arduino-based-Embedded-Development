void sensor() {
  my_adxl345.accel_config();
  delay(100);
  my_itg3200.gyro_config();
  delay(100);
}

