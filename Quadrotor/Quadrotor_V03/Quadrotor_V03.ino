// Attitude (Roll, Pitch and Yaw) Control using MPU9250 without ZOH and Kalman Filter for Roll, Pitch and Yaw.
// can fly
// Do Not Delete!!!
#include "MPU9250.h"

MPU9250 my_mpu9250(0);

// for sensors
double accel_x = 0, accel_y = 0, accel_z = 0;
double accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0;
double gyro_x = 0, gyro_y = 0, gyro_z = 0;
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
double mag_x = 0, mag_y = 0, mag_z = 0;
double mag_x_bias = 0, mag_y_bias = 0, mag_z_bias;
double roll_accel = 0, pitch_accel = 0, yaw_accel = 0;
double roll_gyro = 0, pitch_gyro = 0, yaw_gyro = 0;
double roll_mag = 0, pitch_mag = 0, yaw_mag = 0;
double roll = 0, pitch = 0, yaw = 0;
double roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
// for ISR
int pulse_value_min = 1000, pulse_value_max = 2000;
volatile int pulse_value_1, pulse_value_2, pulse_value_3, pulse_value_4;
volatile boolean previous_state_1 = 0, previous_state_2 = 0, previous_state_3 = 0, previous_state_4 = 0;
volatile unsigned long current_time, rising_time_1, rising_time_2, rising_time_3, rising_time_4;
// for ESC
boolean esc_state_switch = 0;
int esc_value_1, esc_value_2, esc_value_3, esc_value_4;
unsigned long start_time, end_time_1, end_time_2, end_time_3, end_time_4;
// for sample
double dt = 0.01;
unsigned long previous_sample_time = 0, system_timer = 0;
int time_diff;
// for kalman filter
double roll_state[2] = {roll, gyro_x_bias};
double pitch_state[2] = {pitch, gyro_y_bias};
double M_north = 0;
double state2[5] = {yaw, gyro_z_bias, M_north, mag_x_bias, mag_y_bias};
double P2[5][5];
// for controllers
double kp = 2, ki = 0.01, kd = 20;
// for debug
double debug_value;

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
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++ ) {
      if (j == i) P2[i][j] = 100;
      else P2[i][j] = 0;
    }
  }
  my_mpu9250.acceltempgyro_config();
  delay(100);
  my_mpu9250.mag_config();
  delay(100);
  // configure esc
  esc_state_switch = 1;
  esc_thrust_setting();
  delay(3000);
  // set esc thrust as the minimum value to void the bi-bi sound
  while (pulse_value_1 <= 1990 || pulse_value_2 <= 1990) {
    esc_value_1 = pulse_value_min;
    esc_value_2 = pulse_value_min;
    esc_value_3 = pulse_value_min;
    esc_value_4 = pulse_value_min;
    esc_thrust();
  }
  esc_state_switch = 1;
  Serial.begin(115200);
  delay(3000);
  previous_sample_time = millis();
}

void loop() {
  system_timer = millis();
  if (system_timer >= previous_sample_time + dt * 1000) {
    time_diff = system_timer - previous_sample_time;
    previous_sample_time = system_timer;
    my_mpu9250.accel_sample(&accel_x, &accel_y, &accel_z);
    my_mpu9250.accel_sample(&gyro_x, &gyro_y, &gyro_z);
    my_mpu9250.mag_sample(&mag_x, &mag_y, &mag_z);
    accel_Euler();
    kalman_filter(roll_accel, gyro_x, 1);
    kalman_filter(pitch_accel, gyro_y, 2);
    roll = roll_state[0];
    pitch = pitch_state[0];
    gyro_x_bias = roll_state[1];
    gyro_y_bias = pitch_state[1];
    kalman_filter_yaw();
    yaw = state2[0];
    roll_sum += roll * dt;
    pitch_sum += pitch * dt;
    yaw_sum += yaw * dt;
    if (esc_state_switch == 1) {
      pid_controller(kp, ki, kd);
      if (pulse_value_1 <= 1010 && pulse_value_2 >= 1990) esc_state_switch = 0;
    }
    else if (esc_state_switch == 0) {
      esc_value_1 = pulse_value_min;
      esc_value_2 = pulse_value_min;
      esc_value_3 = pulse_value_min;
      esc_value_4 = pulse_value_min;
      if (pulse_value_1 >= 1990 && pulse_value_2 >= 1990) esc_state_switch = 1;
    }
  }
  esc_thrust();
  debug_value = yaw * 180 / PI;
  Serial.println(debug_value);
}
