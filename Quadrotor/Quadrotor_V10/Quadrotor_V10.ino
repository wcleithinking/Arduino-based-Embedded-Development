/*
   Quadrotor_V12 Code to test Sparkfun MPU9250 library,
   by: Wenchao Lei
   date: Nov. 15, 2016
*/

#include "Sparkfun_MPU9250.h"
#include "quaternionFilters.h"
#include <Servo.h>

#define SerialDebug false
#define QuadP
#define PID_v1

// DLPF
#define DLPF_ROLL     1
#define DLPF_PITCH    1
#define DLPF_YAW      1
// PWM
#define Pulse_Min_O   1016
#define Pulse_Max_O   2016
#define Pulse_Min     1000
#define Pulse_Max     2000
#define PWM_MIN       1000
#define PWM_MAX       2000
#define RadToDeg      (180/PI)
#define DegToRad      (PI/180)
// PIN
#define motor0Pin     3
#define motor1Pin     9
#define motor2Pin     10
#define motor3Pin     11
#define intPin        12
#define ledPin        13

// ATIITUDE
float hatroll = 0, hatpitch = 0, hatyaw = 0;
float roll_bias = 0, pitch_bias = 0, yaw_bias = 0;
float roll = 0, pitch = 0, yaw = 0;
float roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
float droll = 0, dpitch = 0, dyaw = 0;
float roll_d = 0, pitch_d = 0, yaw_d = 0;

// PID
#if defined(PID_v1)
float kp = 1.8, ki = 0.02, kd = 4.8;
//float kp = 0.4165, ki = 0.04451, kd = 0.9744;
#elif defined(PID_v2)
float K[3] = {2, 2, 2};
float Ti[3] = {36.3, 36.3, 36.3};
float Td[3] = {9.075, 9.075, 9.075};
float N[3] = {8, 8, 8};
float e[3] = {0, 0, 0};
float eold[3] = {0, 0, 0};
float P[3] = {0, 0, 0};
float I[3] = {0, 0, 0};
float D[3] = {0, 0, 0};
float Dold[3] = {0, 0, 0};
float PID[3] = {0, 0, 0};
#endif

// CONTROL
float umax = 50;
float roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
float roll_feedback = 0, pitch_feedback = 0, altitude_feedback = 0, yaw_feedback = 0;
float roll_controller = 0, pitch_controller = 0, altitude_controller = 0, yaw_controller = 0;

// RX
volatile int pulse1, pulse2, pulse3, pulse4;
volatile bool prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising1, rising2, rising3, rising4;

// ESC
bool esc_switch = 0;
int PWM_ref[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_out[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};

// LOOP
float dt = 0.01;
uint16_t loop_index = 0;
uint16_t time_diff;
unsigned long time_old, time_new, time_1, time_2;

MPU9250 myIMU;
Servo motor0, motor1, motor2, motor3;

void setup() {
  LED_init();
  delay(1000);
  LED_high();
  RX_init();
  ESC_init();
  IMU_init();
  LED_low();
  delay(1000);
  LED_high();
  delay(1000);
  if (SerialDebug) Serial.begin(115200);
  else while (pulse1 <= (PWM_MAX - 10) || pulse2 <= (PWM_MAX - 10));
  LED_low();      // indicate unlock
  esc_switch = 1; // unlock motor
  time_old = millis();
}

void loop() {
  time_new = millis();
  if (time_new >= time_old + dt * 1000) {
    if (SerialDebug) time_diff = time_new - time_old;
    time_old = time_new;
    IMU_get();
    attitude_calibrate();
    motor_control();
    motor_update();
    if (loop_index < 700) loop_index++;
  }
  if (SerialDebug) Serial.println(yaw);
}
