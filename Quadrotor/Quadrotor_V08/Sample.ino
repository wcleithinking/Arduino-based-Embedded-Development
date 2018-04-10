void sample_IMU() {
  my_mpu9250.accel_sample(&accel_x, &accel_y, &accel_z);
  my_mpu9250.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
  my_mpu9250.mag_sample(&mag_x, &mag_y, &mag_z);
}

#if defined(BARO)
void sample_baro() {
  if ((TEMP_update == 0) && (P_update == 1)) {
    if (millis()  >= time_convert_TEMP + BARO_DELAY ) {
      my_ms5611.adc_read(&Data2);
      calculate_temp();
      TEMP_update = 1;
      P_update = 0;
      time_convert_P = millis();
      my_ms5611.adc_D1_convert(BARO_OSR);
    }
  }
  else if ((TEMP_update == 1) && (P_update == 0)) {
    if (millis() >= (time_convert_P + BARO_DELAY)) {
      my_ms5611.adc_read(&Data1);
      calculate_pressure();
      if (loop_index <= 50) baro_P = PRESSURE;
      else baro_P = (1 - BARO_CPF) * baro_P + BARO_CPF * PRESSURE;
      TEMP_update = 0;
      P_update = 1;
      time_convert_TEMP = millis();
      my_ms5611.adc_D2_convert(BARO_OSR);
    }
  }
}

void calculate_temp() {
  diff_T = Data2 - C_PROM[5] * pow(2, 8);
  TEMP = 2000 + diff_T * C_PROM[6] / pow(2, 23);
}

void calculate_pressure() {
  OFF  = C_PROM[2] * pow(2, 16) + (C_PROM[4] * diff_T) / pow(2, 7);
  SENS = C_PROM[1] * pow(2, 15) + (C_PROM[3] * diff_T) / pow(2, 8);
  if (TEMP < 2000) {
    TEMP2 = pow(diff_T, 2) / pow(2, 31);
    OFF2  = 5 * pow(TEMP - 2000, 2) / 2;
    SENS2 = 5 * pow(TEMP - 2000, 2) / 4;
    if (TEMP < -1500) {
      OFF2  = OFF2 + 7 * pow(TEMP + 1500, 2);
      SENS2 = SENS2 + 11 * pow(TEMP + 1500, 2) / 2;
    }
  }
  else {
    TEMP2 = 0;
    OFF2 = 0;
    SENS2 = 0;
  }
  TEMP = TEMP - TEMP2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;
  PRESSURE = (Data1 * SENS / pow(2, 21) - OFF) / pow(2, 15);
}

void calculate_altitude() {
  baro_hatz = -((pow((101325 / baro_P), 1 / 5.257) - 1.0) * (TEMP / 100 + 273.15)) / 0.0065;
}
#elif defined(SONAR)
void sample_sonar() {
  if (millis() > time_sonar_sample + SONAR_DELAY) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    distance = duration / SONAR_SCALE;
    if (distance >= SONAR_MAX || distance <= SONAR_MIN) {
      sonar_error_flag = 1;
      sonar_hatz = altitude_d;
    }
    else {
      sonar_error_flag = 0;
      sonar_hatz = -distance * 0.01;
    }
  }
}
#endif

