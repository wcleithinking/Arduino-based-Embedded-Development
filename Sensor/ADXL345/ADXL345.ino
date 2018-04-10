#include <Wire.h>
#include "ADXL345.h"

ADXL345 my_adxl345;

// The variables hold the x,y and z axes acceleration.
double accel_x, accel_y, accel_z;

void setup() {
  Wire.begin();
  my_adxl345.accel_config();
  Serial.begin(9600);
}

void loop() {
  my_adxl345.accel_sample(&accel_x, &accel_y, &accel_z);
  Serial.println(accel_z);
  delay(100);
}
