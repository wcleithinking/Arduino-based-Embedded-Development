void Sensor() {
  Wire.begin();
  // attitude
  my_mpu9250.acceltempgyro_config(3, 4, 3, 4);
  delay(100);
  my_mpu9250.mag_config(1);
  delay(100);
  // altitude
#if defined(BARO)
  my_ms5611.baro_reset();
  delay(100);
  my_ms5611.prom_read(C_PROM);
  delay(100);
  time_convert_TEMP = millis();
  my_ms5611.adc_D2_convert(BARO_OSR);
  TEMP_update = 0;
  P_update = 1;
  // accel_z_start
  for (int i = 0; i < 20; i++) {
    sample_IMU();
    delay(10);
  }
  for (int i = 0; i < 100; i++) {
    sample_IMU();
    accel_z_start += accel_z;
    delay(10);
  }
  accel_z_start *= 0.01;
#elif defined(SONAR)
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  sample_sonar();
  time_sonar_sample = millis();
#endif
}
