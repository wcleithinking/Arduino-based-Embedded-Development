// 1. Attitude Control using MPU9250 with ZOH
// 2. Kalman Filter for Roll, Pitch and Yaw
// Testing Digital PID by using Servo.h

#include <Servo.h>
#include "MPU9250.h"

MPU9250 my_mpu9250(0);
Servo motor0, motor1, motor2, motor3;

#define QuadX

//#define MODIFIED_PID
//#define DEBUG

#define RadToDeg  (180/PI)
#define DegToRad  (PI/180)

// for sensors
double accel_x, accel_y, accel_z;
double gyro_x,  gyro_y, gyro_z;
double mag_x, mag_y, mag_z;
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
double mag_x_bias = 0, mag_y_bias = 0, mag_z_bias;
double M_north = 10;
// for attitude
double hatroll = 0, hatpitch = 0, hatyaw = 0;
double roll = 0, pitch = 0, yaw = 0;
double roll_bias = 0, pitch_bias = 0, yaw_bias = 0;
double roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
// for ISR
const int pulse_min = 1016, pulse_max = 2016;
volatile int pulse1, pulse2, pulse3, pulse4;
volatile boolean prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising1, rising2, rising3, rising4;
// for sample
const double dt = 0.01;
int index = 0;
int time_diff;
unsigned long time_old, time_new;

// for controllers
double K = 2.0, Ti = 200, Td = 0.6, N = 8; // Ti decrese, K increse and Td left and right change
double yd[3] = {0, 0, 0};
double e[3] = {0, 0, 0};
double P[3] = {0, 0, 0};
double I[3] = {0, 0, 0};
double D[3] = {0, 0, 0};
double PID[3] = {0, 0, 0};
double eold[3] = {0, 0, 0};
double Dold[3] = {0, 0, 0};
double roll_feedback = 0, pitch_feedback = 0, altitude_feedback = 0, yaw_feedback = 0;
double roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
double roll_controller, pitch_controller, altitude_controller, yaw_controller;
const double umax = 50;
double u[4];
// for ESC
boolean esc_switch = 0;
int MOTOR[4] = {pulse_min, pulse_min, pulse_min, pulse_min};
// for debug
double debug_output;

void setup() {
  motor0.attach(3);
  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  ESC();
  RX_config();
  sensor_config();
#ifdef DEBUG
  Serial.begin(115200);
#else
  while (pulse1 <= 2000 || pulse2 <= 2000);
#endif
  // calibration
  for (int i = 0; i < 110; i++) {
    sensor_sample();
    attitude_filter();
    if (i >= 10) {
      roll_sum += roll;
      pitch_sum += pitch;
      yaw_sum += yaw;
    }
    delay(dt*1000);
  }
  roll_bias = roll_sum * 0.01;
  pitch_bias = pitch_sum * 0.01;
  yaw_bias = yaw_sum * 0.01;
  // initialize the input
  sensor_sample();
  attitude_filter();
  esc_switch = 1;
  time_old = millis();
}

void loop() {
  time_new = millis();
  if (time_new - time_old >= dt * 1000 ) {
    time_diff = time_new - time_old;
    time_old = time_new;
    sensor_sample();
    attitude_filter();
    if (pulse1 <= 1020 && pulse2 >= 2000) {
      esc_switch = 0;
      for (int i = 0; i < 3; i++) I[i] = 0;
    }
    if (pulse1 >= 2000 && pulse2 >= 2000) {
      esc_switch = 1;
    }
    dynamics_control();
#ifdef DEBUG
    for (int i = 0; i < 4; i++) MOTOR[0] = pulse_min;
#endif
    ESC();
  }
#ifdef DEBUG
  Serial.println(roll_feedback);
#endif
}
