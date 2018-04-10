// Attitude (Roll, Pitch and Yaw) Control using MPU9250 without ZOH and Kalman Filter for Roll, Pitch and Yaw with modification in Roll and Pitch.
// change in attitude_filter-----------------need modification and test.
// Complicated and Time Consuming!!!
#include "MPU9250.h"

MPU9250 my_mpu9250(0);

// for sensors
double accel_x = 0, accel_y = 0, accel_z = 0;
double accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0;
double gyro_x = 0, gyro_y = 0, gyro_z = 0;
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
double M_north = 0;
double mag_x = 0, mag_y = 0, mag_z = 0;
double mag_x_bias = 0, mag_y_bias = 0, mag_z_bias;
// for attitude
double roll = 0, pitch = 0, yaw = 0;
double roll_bias = 0, pitch_bias = 0, yaw_bias = 0;
double roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
// for ISR
const int pulse_min = 1000, pulse_max = 2000;
volatile boolean prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long current_time, rising1, rising2, rising3, rising4;
volatile int pulse1, pulse2, pulse3, pulse4;
// for ESC
boolean esc_switch = 0;
unsigned long start_time, end_time_1, end_time_2, end_time_3, end_time_4;
int PWM1, PWM2, PWM3, PWM4;
// for sample
const double dt = 0.04;
unsigned long time_old = 0, time_new = 0;
int time_diff;
// for controllers
const double kp = 2, ki = 0.01, kd = 20;
double u1, u2, u3, u4;
// for debug
double debug_output;

void setup() {
  // set pins 4, 5, 6 and 7 as output
  DDRD |= B11110000;
  // set pin change interrupt control register
  PCICR |= (1 << PCIE0);
  // set pin change mask register 0
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  sensor_config();
  // configure esc
  esc_switch = 1;
  // esc_thrust_setting();
  delay(1000);
  // set esc thrust as the minimum value to void the bi-bi sound
  //  while (pulse1 <= 1990 || pulse2 <= 1990) {
  //    PWM1 = pulse_min;
  //    PWM2 = pulse_min;
  //    PWM3 = pulse_min;
  //    PWM4 = pulse_min;
  //    esc_thrust();
  //  }
  esc_switch = 1;
  Serial.begin(115200);
  time_old = millis();
}

void loop() {
  time_new = millis();
  if (time_new >= time_old + dt * 1000) {
    time_diff = time_new - time_old;
    time_old = time_new;
    sensor_sample();
    attitude_filter();
    roll_sum += roll * dt;
    pitch_sum += pitch * dt;
    yaw_sum += yaw * dt;
    if (esc_switch == 1) {
      pid_controller();
      if (pulse1 <= 1010 && pulse2 >= 1990) esc_switch = 0;
    }
    else if (esc_switch == 0) {
      PWM1 = pulse_min;
      PWM2 = pulse_min;
      PWM3 = pulse_min;
      PWM4 = pulse_min;
      if (pulse1 >= 1990 && pulse2 >= 1990) esc_switch = 1;
    }
    esc_thrust();
  }
  debug_output = yaw * 180 / PI;
  Serial.println(debug_output);
}
