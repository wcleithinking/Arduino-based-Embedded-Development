#include "IMU.h"

IMU myIMU;

// for sensor
float Gyro_bias_estimate[3] = {0, 0, 0};
float Angle_estimate[3]     = {0, 0, 0};
float dt = 0.01;
unsigned long time_old, time_new, time_diff;
void setup() {
  Serial.begin(115200);
  myIMU.init(3, 4, 3, 4, 1, dt);
}

void loop() {
  time_new = millis();
  if (time_new >= time_old + dt * 1000) {
    time_diff = time_new - time_old;
    time_old = time_new;
    myIMU.sample();
    myIMU.attitude_filter(Angle_estimate, Gyro_bias_estimate);
  }
  Serial.println(Angle_estimate[2] * 180 / PI);
}
