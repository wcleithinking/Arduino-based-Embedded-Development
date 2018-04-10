// Attitude (Roll, Pitch and Yaw) Control using MPU9250 without ZOH and Kalman Filter for Roll and Pitch, a test algorithm for Yaw (not stable).
// can fly but no yaw
#include "MPU9250.h"

MPU9250 my_mpu9250(0);

// for sensors
double accel_x = 0, accel_y = 0, accel_z;
double accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0;
double gyro_x = 0, gyro_y = 0, gyro_z = 0;
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
double mag_x_start = 0, mag_y_start = 0, mag_z_start = 0;
double mag_x = 0, mag_y = 0, mag_z = 0;
double mag_x_bias = 0, mag_y_bias = 0, mag_z_bias;
double roll_accel = 0, pitch_accel = 0, yaw_accel = 0;
double roll_gyro = 0, pitch_gyro = 0, yaw_gyro = 0;
double roll_mag = 0, pitch_mag = 0, yaw_mag = 0;
double roll = 0, pitch = 0, yaw = 0;
double roll_bias = 0, pitch_bias = 0, yaw_bias = 0;
double roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
// for ISR
int pulse_value_min = 1000, pulse_value_max = 2000;
volatile boolean previous_state_1 = 0, previous_state_2 = 0, previous_state_3 = 0, previous_state_4 = 0;
volatile unsigned long current_time, rising_time_1, rising_time_2, rising_time_3, rising_time_4;
volatile int pulse_value_1, pulse_value_2, pulse_value_3, pulse_value_4;
// for ESC
boolean esc_state_switch = 0;
int esc_value_1, esc_value_2, esc_value_3, esc_value_4;
unsigned long start_time, end_time_1, end_time_2, end_time_3, end_time_4;
// for sample
double dt = 0.01;
unsigned long previous_sample_time = 0, system_timer = 0;
int time_diff;
// for kalman filter
double F[2][2] = {1, -dt, 0, 1};
double Q[2][2] = {0.001, 0, 0, 0}, R = 0.01;
double roll_P[2][2] = {100, 0, 0, 100};
double pitch_P[2][2] = {100, 0, 0, 100};
double roll_state[2] = {roll, gyro_x_bias};
double pitch_state[2] = {pitch, gyro_y_bias};
// for controllers
double kp = 2, ki = 0.01, kd = 20;
// for debug
double debug_value;

void setup() {
  // set pins 4, 5, 6 and 7 as output
  DDRD |= 0b11110000;
  // set pin change interrupt control register
  PCICR |= (1 << PCIE0);
  // set pin change mask register 0
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  my_mpu9250.acceltempgyro_config();
  delay(100);
  my_mpu9250.mag_config();
  delay(100);
  my_mpu9250.mag_sample(&mag_x, &mag_y, &mag_z);
  mag_x_start = mag_x;
  mag_y_start = mag_y;
  mag_z_start = mag_z;
  esc_state_switch = 1;
  esc_thrust_setting();
  delay(3000);
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
    my_mpu9250.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
    my_mpu9250.mag_sample(&mag_x, &mag_y, &mag_z);
    accel_Euler();
    kalman_filter(roll_accel, gyro_x, 1);
    kalman_filter(pitch_accel, gyro_y, 2);
    roll = roll_state[0];
    pitch = pitch_state[0];
    gyro_x_bias = roll_state[1];
    gyro_y_bias = pitch_state[1];
    mag_yaw();
    yaw = yaw_mag;
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
  debug_value = time_diff;
  Serial.println(debug_value);
}

void esc_thrust_setting() {
  if (esc_state_switch == 1) {
    while (pulse_value_3 <= 1995);
    while (pulse_value_1 <= 1990 && pulse_value_2 <= 1990) {
      esc_value_1 = pulse_value_3;
      esc_value_2 = pulse_value_3;
      esc_value_3 = pulse_value_3;
      esc_value_4 = pulse_value_3;
      esc_thrust();
    }
  }
}

void esc_thrust() {
  start_time = micros();
  PORTD |= 0b11110000;
  end_time_1 = start_time + esc_value_1;
  end_time_2 = start_time + esc_value_2;
  end_time_3 = start_time + esc_value_3;
  end_time_4 = start_time + esc_value_4;
  while ((int)PORTD >= 16) {
    if (micros() >= end_time_1) PORTD &= 0b11101111;
    if (micros() >= end_time_2) PORTD &= 0b11011111;
    if (micros() >= end_time_3) PORTD &= 0b10111111;
    if (micros() >= end_time_4) PORTD &= 0b01111111;
  }
}

void accel_Euler() {
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

void kalman_filter(double angle_accel, double rate_gyro, int angle_flag) {
  double state[2], P[2][2];
  if (angle_flag == 1) {
    state[0] = roll_state[0];
    state[1] = roll_state[1];
    P[0][0] = roll_P[0][0];
    P[0][1] = roll_P[0][1];
    P[1][0] = roll_P[1][0];
    P[1][1] = roll_P[1][1];
  }
  else if (angle_flag == 2) {
    state[0] = pitch_state[0];
    state[1] = pitch_state[1];
    P[0][0] = pitch_P[0][0];
    P[0][1] = pitch_P[0][1];
    P[1][0] = pitch_P[1][0];
    P[1][1] = pitch_P[1][1];
  }
  double prestate[2], preP[2][2];
  double e, S, K[2];
  // predict
  prestate[0] = F[0][0] * state[0] + F[0][1] * state[1] + dt * rate_gyro;
  prestate[1] = F[1][0] * state[0] + F[1][1] * state[1];
  preP[0][0] = F[0][0] * F[0][0] * P[0][0] + 2 * F[0][0] * F[0][1] * P[0][1] + F[0][1] * F[0][1] * P[1][1] + Q[0][0];
  preP[0][1] = F[0][0] * F[1][1] * P[0][1] + F[0][1] * F[1][1] * P[1][1] + Q[0][1];
  preP[1][0] = preP[0][1];
  preP[1][1] = F[1][1] * F[1][1] * P[1][1] + Q[1][1];
  // update
  e = angle_accel - prestate[0];
  S = preP[0][0] + R;
  K[0] = preP[0][0] / S;
  K[1] = preP[0][1] / S;
  state[0] = prestate[0] + K[0] * e;
  state[1] = prestate[1] + K[1] * e;
  P[0][0] = (1 - K[0]) * preP[0][0];
  P[0][1] = (1 - K[0]) * preP[0][1];
  P[1][0] = P[0][1];
  P[1][1] = -K[1] * preP[0][1] + preP[1][1];
  if (angle_flag == 1) {
    roll_state[0] = state[0];
    roll_state[1] = state[1];
    roll_P[0][0] = P[0][0];
    roll_P[0][1] = P[0][1];
    roll_P[1][0] = P[1][0];
    roll_P[1][1] = P[1][1];
  }
  else if (angle_flag == 2) {
    pitch_state[0] = state[0];
    pitch_state[1] = state[1];
    pitch_P[0][0] = P[0][0];
    pitch_P[0][1] = P[0][1];
    pitch_P[1][0] = P[1][0];
    pitch_P[1][1] = P[1][1];
  }
}

void mag_yaw() {
  double mag_all_start, mag_x_norm_start, mag_y_norm_start, mag_z_norm_start;
  double mag_all, mag_x_norm, mag_y_norm, mag_z_norm;
  double mag_x_norm_middle, mag_y_norm_middle, mag_z_norm_middle, sign;
  // normalize the start measurement
  mag_all_start = sqrt(mag_x_start * mag_x_start + mag_y_start * mag_y_start + mag_z_start * mag_z_start);
  mag_x_norm_start = mag_x_start / mag_all_start;
  mag_y_norm_start = mag_y_start / mag_all_start;
  mag_z_norm_start = mag_z_start / mag_all_start;
  // normalize the measured data
  mag_all = sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
  mag_x_norm = mag_x / mag_all;
  mag_y_norm = mag_y / mag_all;
  mag_z_norm = mag_z / mag_all;
  // transform to middle frame by using roll and pitch
  // roll = 0; pitch = 0;
  mag_x_norm_middle = mag_x_norm * cos(roll * PI / 180) + (mag_y_norm * sin(roll * PI / 180) + mag_z_norm * cos(pitch * PI / 180)) * sin(pitch * PI / 180);
  mag_y_norm_middle = mag_y_norm * cos(roll * PI / 180) - mag_z_norm * sin(pitch * PI / 180);
  mag_z_norm_middle = -mag_x_norm * sin(roll * PI / 180) + (mag_y_norm * sin(roll * PI / 180) + mag_z_norm * cos(pitch * PI / 180)) * cos(pitch * PI / 180);
  // compute the yaw angle from the magnetometer.
  if ((mag_x_norm_start * mag_y_norm_middle - mag_y_norm_start * mag_x_norm_middle) > 0) sign = -1;
  else sign = 1;
  yaw_mag = sign * acos((mag_x_norm_start * mag_x_norm_middle + mag_y_norm_start * mag_y_norm_middle) / (sqrt(mag_x_norm_start * mag_x_norm_start + mag_y_norm_start * mag_y_norm_start) * sqrt(mag_x_norm_middle * mag_x_norm_middle + mag_y_norm_middle * mag_y_norm_middle)));
}

void pid_controller(float kp, float ki, float kd) {
  double altitude_feedforward = 0;
  double roll_feedback = 0, pitch_feedback = 0, yaw_feedback = 0;
  altitude_feedforward = pulse_value_3;
  roll_feedback   = kp * roll * 180 / PI  + ki * roll_sum * 180 / PI  + kd * (gyro_x - gyro_x_bias) * 180 / PI;
  pitch_feedback  = kp * pitch * 180 / PI + ki * pitch_sum * 180 / PI + kd * (gyro_y - gyro_y_bias) * 180 / PI;
  yaw_feedback    = kp * yaw * 180 / PI ;
  roll_feedback   = constrain(roll_feedback, -100, 100);
  pitch_feedback  = constrain(pitch_feedback, -100, 100);
  yaw_feedback    = constrain(yaw_feedback, -100, 100);
  altitude_feedforward = map(pulse_value_3, pulse_value_min, pulse_value_max, pulse_value_min, pulse_value_max - 100);
  esc_value_1 = altitude_feedforward - pitch_feedback - yaw_feedback ;
  esc_value_2 = altitude_feedforward + roll_feedback  + yaw_feedback ;
  esc_value_3 = altitude_feedforward + pitch_feedback - yaw_feedback;
  esc_value_4 = altitude_feedforward - roll_feedback  + yaw_feedback;
  esc_value_1 = constrain(esc_value_1, pulse_value_min, pulse_value_max);
  esc_value_2 = constrain(esc_value_2, pulse_value_min, pulse_value_max);
  esc_value_3 = constrain(esc_value_3, pulse_value_min, pulse_value_max);
  esc_value_4 = constrain(esc_value_4, pulse_value_min, pulse_value_max);
}

ISR(PCINT0_vect) {
  current_time = micros();
  // channel 1: digital pin  8 : roll signal
  if (PINB & 0b00000001) {
    if (previous_state_1 == 0) {
      previous_state_1 = 1;
      rising_time_1 = current_time;
    }
  }
  else if (previous_state_1 == 1) {
    previous_state_1 = 0;
    pulse_value_1 = current_time - rising_time_1 - 16;
    pulse_value_1 = constrain(pulse_value_1, pulse_value_min, pulse_value_max);
  }
  // channel 2: digital pin 9 : pitch signal
  if (PINB & 0b00000010) {
    if (previous_state_2 == 0) {
      previous_state_2 = 1;
      rising_time_2 = current_time;
    }
  }
  else if (previous_state_2 == 1) {
    previous_state_2 = 0;
    pulse_value_2 = current_time - rising_time_2 - 16;
    pulse_value_2 = constrain(pulse_value_2, pulse_value_min, pulse_value_max);
  }
  // channel 3: digital pin 10 : thrust signal
  if (PINB & 0b00000100) {
    if (previous_state_3 == 0) {
      previous_state_3 = 1;
      rising_time_3 = current_time;
    }
  }
  else if (previous_state_3 == 1) {
    previous_state_3 = 0;
    pulse_value_3 = current_time - rising_time_3 - 16;
    pulse_value_3 = constrain(pulse_value_3, pulse_value_min, pulse_value_max);
  }
  // channel 4: digital pin 11 : yaw signal
  if (PINB & 0b00001000) {
    if (previous_state_4 == 0) {
      previous_state_4 = 1;
      rising_time_4 = current_time;
    }
  }
  else if (previous_state_4 == 1) {
    previous_state_4 = 0;
    pulse_value_4 = current_time - rising_time_4 - 16;
    pulse_value_4 = constrain(pulse_value_4, pulse_value_min, pulse_value_max);
  }
}
