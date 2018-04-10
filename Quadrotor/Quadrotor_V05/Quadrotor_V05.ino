// Attitude (Roll, Pitch and Yaw) Control using MPU9250 with ZOH and Kalman Filter for Roll, Pitch and Yaw.
// can fly and good
// Do Not Delete!!!
#include "MPU9250.h"

MPU9250 my_mpu9250;

#define QuadP
// for sensors
float accel_x, accel_y, accel_z;
float gyro_x,  gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
float gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
float mag_x_bias = 0, mag_y_bias = 0, mag_z_bias;
float M_north = 10;
// for attitude
float hatroll = 0, hatpitch = 0, hatyaw = 0;
float roll = 0, pitch = 0, yaw = 0;
float roll_bias = 0, pitch_bias = 0, yaw_bias = 0;
float roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
// for ISR
const int pulse_min = 1015, pulse_max = 1815;
volatile int pulse1, pulse2, pulse3, pulse4;
volatile boolean prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising1, rising2, rising3, rising4;
// for sample
const float dt = 0.01;
int index = 0;
int time_diff;
unsigned long time_old, time_new;
// for controllers
const float kp = 1.8, ki = 0.002, kd = 4.8;
const float umax = 50;
float u[4];
// for ESC
boolean esc_switch = 0;
int MOTOR[4] = {pulse_min, pulse_min, pulse_min, pulse_min};
// for debug
float debug_output;

void setup() {
  ESC();
  RX_config();
  sensor_config();
  //while (pulse1 <= 2000 || pulse2 <= 2000);
  esc_switch = 1;
  Serial.begin(115200);
  time_old = millis();
}

void loop() {
  time_new = millis();
  if (time_new - time_old >= dt * 1000 ) {
    time_diff = time_new - time_old;
    time_old = time_new;
    sensor_sample();
    attitude_filter();
    if (index < 100) {
      for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
    }
    else if ((index >= 100) && (index < 300)) {
      roll_bias += hatroll;
      pitch_bias += hatpitch;
      yaw_bias += hatyaw;
      for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
    }
    else if ((index >= 300) && (index < 350)) {
      if (index == 300) {
        roll_bias = roll_bias * 0.005;
        pitch_bias = pitch_bias * 0.005;
        yaw_bias = yaw_bias * 0.005;
      }
      roll = hatroll - roll_bias;
      pitch = hatpitch - pitch_bias;
      yaw = hatyaw - yaw_bias;
      for (int i = 0; i < 4; i++) MOTOR[i] = pulse_min;
    }
    else {
      roll = hatroll - roll_bias;
      pitch = hatpitch - pitch_bias;
      yaw = hatyaw - yaw_bias;
      roll_sum += roll * dt;
      pitch_sum += pitch * dt;
      yaw_sum += yaw * dt;
      if (pulse1 <= 1020 && pulse2 >= 2000) {
        esc_switch = 0;
        roll_sum = 0;
        pitch_sum = 0;
        yaw_sum = 0;
      }
      if (pulse1 >= 2000 && pulse2 >= 2000) {
        esc_switch = 1;
      }
      dynamics_control();
    }
    ESC();
    if (index < 350) index++;
  }
  Serial.println(gyro_y);
}
