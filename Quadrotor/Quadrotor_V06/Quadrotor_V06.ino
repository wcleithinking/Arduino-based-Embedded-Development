// STR test for Attitude (Roll, Pitch and Yaw) Control using MPU9250 with ZOH and Kalman Filter for Roll, Pitch and Yaw.
// Important: STR----------------------need modification and test.
// Do Not Delete!!!
#include <Servo.h>
#include "MPU9250.h"

MPU9250 my_mpu9250(0);
Servo motor1, motor2, motor3, motor4;

// for sensors
double accel_x, accel_y, accel_z;
double accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0;
double gyro_x, gyro_y, gyro_z;
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
int M_north = 0;
double mag_x, mag_y, mag_z;
double mag_x_bias = 0, mag_y_bias = 0, mag_z_bias;
// for attitude and altitude
double hatroll = 0, hatpitch = 0, hatyaw = 0;
double roll_bias = 0, pitch_bias = 0, yaw_bias = 0;
double roll_ooo = 0, pitch_ooo = 0, yaw_ooo = 0, altitude_ooo = 0;
double roll_oo = 0, pitch_oo = 0, yaw_oo = 0, altitude_oo = 0;
double roll_o = 0, pitch_o = 0, yaw_o = 0, altitude_o = 0;
double roll = 0, pitch = 0, yaw = 0, altitude = 0;
double roll_sum = 0, pitch_sum = 0, yaw_sum = 0, altitude_sum = 0;
// for ISR
const int pulse_min = 1000, pulse_max = 2000;
volatile int pulse1, pulse2, pulse3, pulse4;
volatile boolean prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising1, rising2, rising3, rising4;
// for ESC
boolean esc_switch;
double PWM_percent[4] = {0, 0, 0, 0};
int PWM[4] = {1000, 1000, 1000, 1000};
// for sample
const double dt = 0.01;
unsigned long time_old, time_new;
int time_diff;
int index = 0;
// for estimators
double hata1[4], hata2[4], hata3[4], hatb0[4], hatb1[4], hatb2[4];
// for controllers
const double kp_accel = 0, kp = 2, ki = 0.01, kd = 20;
const double am1[4];
const double am2[4];
const double am3[4];
const double ao1[4];
const double ao2[4];
const double uc_oo[4] = {0, 0, 0, -1};
const double uc_o[4] = {0, 0, 0, -1};
const double uc[4] = {0, 0, 0, -1};
double u_ooo[4] = {0, 0, 0, 0};
double u_oo[4] = {0, 0, 0, 0};
double u_o[4] = {0, 0, 0, 0};
double u[4] = {0, 0, 0, 0};
// for debug
double debug_output;

void setup() {
  // choose PWM pins
  motor1.attach(3); // pin 3
  motor2.attach(5); // pin 5
  motor3.attach(6); // pin 6
  motor4.attach(9); // pin 9
  // set pin change interrupt control register
  PCICR |= (1 << PCIE0);  // pin change interrupt enable 0: PCINT[7:0], which are enabled by PCMSK0
  // set pin change mask register 0
  PCMSK0 |= (1 << PCINT2);  // pin 10
  PCMSK0 |= (1 << PCINT3);  // pin 11
  PCMSK0 |= (1 << PCINT4);  // pin 12
  PCMSK0 |= (1 << PCINT5);  // pin 13
  while (pulse1 >= 1010 || pulse2 <= 1990);
  ESC();
  sensor_config();
  while (pulse1 <= 1990 || pulse2 <= 1990);
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
    if (index < 100) esc_switch = 0;
    else if ((index >= 100) && (index < 300)) {
      roll_sum += hatroll;
      pitch_sum += hatpitch;
      yaw_sum += hatyaw;
      esc_switch = 0;
    }
    else {
      if (index == 300) {
        roll_bias = roll_sum * 0.005;
        pitch_bias = pitch_sum * 0.005;
        yaw_bias = yaw_sum * 0.005;
        roll_sum = 0;
        pitch_sum = 0;
        yaw_sum = 0;
      }
      roll = hatroll - roll_bias;
      pitch = hatpitch - pitch_bias;
      yaw = hatyaw - yaw_bias;
      roll_sum += roll * dt;
      pitch_sum += pitch * dt;
      yaw_sum += yaw * dt;
      if ((pulse1 <= 1010) && (pulse2 >= 1990)) esc_switch = 0;
      if ((pulse1 >= 1990) && (pulse2 >= 1990)) esc_switch = 1;
      dynamics_identify();
    }
    dynamics_control();
    data_update();
    if (index <= 300) index++;
  }
  ESC();
  debug_output = yaw * 180 / PI;
  Serial.println(debug_output);
}
