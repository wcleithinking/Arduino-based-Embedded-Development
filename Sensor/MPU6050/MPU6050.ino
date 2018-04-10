#include <Wire.h>
#include "MPU6050.h"
MPU6050 my_mpu6050;

#define DegToRad  (PI/180)
#define RadToDeg  (180/PI)

// for sensor
double accel_x = 0, accel_y = 0, accel_z = 0;
double temp = 0;
double gyro_x = 0, gyro_y = 0, gyro_z = 0;
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;

// for attitude
double roll_accel = 0, pitch_accel = 0;
double hatroll = 0, hatpitch = 0;
double roll = 0, pitch = 0;

// for loop
double dt = 0.01;
int time_diff;
unsigned long time_old, time_new;

// for debug
double debug_value;

void setup() {
  Wire.begin();
  my_mpu6050.acceltempgyro_config(3, 3, 4);
  delay(100);
  Serial.begin(115200);
  time_old = millis();
}

void loop() {
  time_new = millis();
  if (time_new >= time_old + dt * 1000) {
    time_diff = time_new - time_old;
    time_old = time_new;
    my_mpu6050.accel_sample(&accel_x, &accel_y, &accel_z);
    my_mpu6050.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
    complementary_filter_rollpitch();
  }
  Serial.println(hatpitch*RadToDeg);
}

void complementary_filter_rollpitch() {
  accel_rollpitch();
  hatroll = 0.98 * (hatroll  + gyro_x * dt) + 0.02 * roll_accel;
  hatpitch  = 0.98 * (hatpitch  + gyro_y * dt)  + 0.02 * pitch_accel;
}

void accel_rollpitch() {
  double tagroll, tagpitch;
  tagroll = accel_y / accel_z;
  tagpitch = accel_x / sqrt(accel_y * accel_y + accel_z * accel_z);
  if (tagroll > 100) {
    roll_accel = PI / 2 - 0.01;
  }
  else if (tagroll < -100) {
    roll_accel = -PI / 2 + 0.01;
  }
  else {
    roll_accel = atan(tagroll);
  }
  if (tagpitch > 100) {
    pitch_accel = PI / 2 - 0.01;
  }

  else if (tagpitch < -100) {
    pitch_accel = -PI / 2 + 0.01;
  }
  else {
    pitch_accel = atan(tagpitch);
  }
}
