// Attitude (Roll and Pitch) Control using ADXL345, ITG3200 and MPU9250 with ZOH and Kalman Filter for Roll and Pitch.
// can fly but no yaw
#include <Servo.h>
#include "ADXL345.h"
#include "ITG3200.h"

Servo motor1, motor2, motor3, motor4;
ADXL345 my_adxl345;
ITG3200 my_itg3200;

// for sensors
double accel_x = 0, accel_y = 0, accel_z = 0;
double accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0;
double gyro_x = 0, gyro_y = 0, gyro_z = 0;
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
double roll_accel = 0, pitch_accel = 0;
double hatroll = 0, hatpitch = 0;
int count = 1000;
double roll = 0, pitch = 0;
double roll_bias = 0, pitch_bias = 0;
double roll_sum = 0, pitch_sum = 0;
// for ISR
int pulse_value_min = 1000, pulse_value_max = 2000;
volatile boolean previous_state_1 = 0, previous_state_2 = 0, previous_state_3 = 0, previous_state_4 = 0;
volatile unsigned long current_time, rising_time_1, rising_time_2, rising_time_3, rising_time_4;
volatile int pulse_value_1, pulse_value_2, pulse_value_3, pulse_value_4;
// for ESC
boolean esc_state_switch = 0;
int esc_value_1, esc_value_2, esc_value_3, esc_value_4;
// for sample
double dt = 0.01;
unsigned long previous_sample_time = 0, system_timer = 0;
int time_diff;
// for controllers
double kp = 2, ki = 0.01, kd = 20;
// for debug
double debug_value;

void setup() {
  // choose PWM pins
  motor1.attach(3); // pin 3
  motor2.attach(5); // pin 5
  motor3.attach(6); // pin 6
  motor4.attach(9); // pin 9
  // set pin change interrupt control register
  PCICR |= (1 << PCIE0);
  // set pin change mask register 0
  PCMSK0 |= (1 << PCINT2);  // pin 10
  PCMSK0 |= (1 << PCINT3);  // pin 11
  PCMSK0 |= (1 << PCINT4);  // pin 12
  PCMSK0 |= (1 << PCINT5);  // pin 13
  while (pulse_value_1 >= 1990 || pulse_value_2 <= 1990);
  esc_value_1 = pulse_value_min;
  esc_value_2 = pulse_value_min;
  esc_value_3 = pulse_value_min;
  esc_value_4 = pulse_value_min;
  esc_thrust();
  Wire.begin();
  sensor();
  for (int i = 0; i < 100; i++) {
    sample();
    delay(10);
  }
  for (int i = 0; i < count; i++) {
    sample();
    attitude();
    roll_sum += hatroll;
    pitch_sum += hatpitch;
  }
  roll_bias = roll_sum / count;
  pitch_bias = pitch_sum / count;
  roll_sum = 0;
  pitch_sum = 0;
  while (pulse_value_1 <= 1990 || pulse_value_2 <= 1990);
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
    sample();
    attitude();
    roll = hatroll - roll_bias;
    pitch = hatpitch - pitch_bias;
    roll_sum += roll * dt;
    pitch_sum += pitch * dt;
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
  debug_value  = accel_x;
  Serial.println(debug_value);
}
