#include <Wire.h>
#include "ITG3200.h"

ITG3200 my_itg3200;

// The variables hold the x,y and z axes angular velocites.
double temp;
double gyro_x, gyro_y, gyro_z;

void setup() {
  Wire.begin();
  my_itg3200.gyro_config();
  Serial.begin(9600);
}

void loop() {
  my_itg3200.temp_sample(&temp);
  my_itg3200.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
  Serial.println(temp);
  delay(100);
}
