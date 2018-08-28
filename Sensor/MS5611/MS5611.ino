#include <Wire.h>
#include "MS5611.h"
MS5611 my_ms5611;

// Variables of MS5611
uint16_t C[8];
uint8_t state;
uint32_t D1, D2;
uint32_t D1_new, D2_new;
int32_t dT, TEMP;
int64_t OFF, OFF2, SENS, SENS2;
int32_t P, P_sum, P_start;
int32_t T2;
// altitude
float hataltitude, altitude_start, altitude;
// Loop
float dt = 0.01;
int loop_index = 0;
int time_diff;
unsigned long time_old, time_new;

void setup() {
  Wire.begin();
  my_ms5611.baro_reset();
  delay(100);
  my_ms5611.prom_read(C);
  Serial.begin(115200);
  time_old = millis();
}

void loop() {
  time_new = millis();
  if ( time_new >= time_old + dt * 1000) {
    time_diff = time_new - time_old;
    time_old = time_new;
    my_ms5611.adc_D1_convert(3);
    delayMicroseconds(4600);
    my_ms5611.adc_read(&D1_new);
    my_ms5611.adc_D2_convert(3);
    delayMicroseconds(4600);
    my_ms5611.adc_read(&D2_new);
    if (loop_index == 1) {
      D1 = D1_new;
      D2 = D2_new;
    }
    else {
      D1 = 0.55 * D1 + 0.45 * D1_new;
      D2 = 0.55 * D2 + 0.45 * D2_new;
    }
    dT = D2 - C[5] * pow(2, 8);
    TEMP = 2000 + dT * C[6] / pow(2, 23);
    OFF = C[2] * pow(2, 16) + (C[4] * dT) / pow(2, 7);
    SENS = C[1] * pow(2, 15) + (C[3] * dT) / pow(2, 8);
    if (TEMP < 2000) {
      T2 = pow(dT, 2) / pow(2, 31);
      OFF2 = 5 * pow(TEMP - 2000, 2) / pow(2, 1);
      SENS2 = 5 * pow(TEMP - 2000, 2) / pow(2, 2);
      if (TEMP < -1500) {
        OFF2 = OFF2 + 7 * pow(TEMP + 1500, 2);
        SENS2 = SENS2 + 11 * pow(TEMP + 1500, 2) / pow(2, 1);
      }
    }
    else {
      T2 = 0;
      OFF2 = 0;
      SENS2 = 0;
    }
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    P = (D1 * SENS / pow(2, 21) - OFF) / pow(2, 15);
    if (loop_index < 100) {
      loop_index++;
    }
    else if (loop_index >= 100 && loop_index < 300) {
      P_sum += P;
      loop_index++;
    }
    else if (loop_index == 300) {
      P_start = P_sum / 200;
      hataltitude = -0.09 * (P - P_start);
      // hataltitude = ((pow((101325 / P), 1 / 5.257) - 1.0) * (TEMP * 0.01 + 273.15)) / 0.0065;
      altitude = hataltitude - altitude_start;
      loop_index++;
    }
    else {
      hataltitude = 0.09 * (P - P_start);
      altitude = 0.55 * altitude + 0.45 * (hataltitude - altitude_start);
    }
  }
  Serial.println(altitude * 100);
}
