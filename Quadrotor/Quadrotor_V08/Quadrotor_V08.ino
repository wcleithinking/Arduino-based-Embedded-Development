/*   Created by Wenchao Lei, Oct. 25, 2016: initial version
     Modified by Wenchao Lei, Nov. 6, 2016: more simple and easy to read
      1. IMU: MPU9250;
      2. Sample: 100HZ;
      3. Attitude: Kalman Filter;
      4. Control: PID with ZOH;
      5. Actuator: PWM by using anologWrite in PWM pins: 3, 9, 10, 11;
      6. Receiver: ISR in pins: 4, 5, 6, 7;
      // Add altitude hold function
      7. BARO: MS5611;
      8. SONAR: HC-SR04: ISR in the echo pin: 2.
*/

/*
   Header Files
*/
#include "Config.h"
#include <Wire.h>
#include "MPU9250.h"
MPU9250 my_mpu9250(0);
#ifdef BARO
#include "MS5611.h"
MS5611 my_ms5611(0);
#endif

/*
   Vehicle Outputs(i.e. from Sensors)
*/
// IMU Variables
double accel_x, accel_y, accel_z, gyro_x,  gyro_y, gyro_z, mag_x, mag_y, mag_z; // output of MPU9250
double accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0;  // bias of accel
double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0; // bias of gyro
double mag_x_bias = 0, mag_y_bias = 0, mag_z_bias = 0; // bias of mag
#if defined(BARO)
// Baro Variables
uint16_t C_PROM[8];
uint32_t Data1, Data2; // output of MS5611
int32_t diff_T, TEMP, TEMP2, PRESSURE;
int64_t OFF, OFF2, SENS, SENS2;
bool P_update = 1, TEMP_update = 1;
unsigned long time_convert_P, time_convert_TEMP;
#elif defined(SONAR)
// Sonar Variables
bool sonar_error_flag = 0;
double distance;
unsigned long time_sonar_sample;
#endif

/*
   Vehicle States(i.e. from Filters)
*/
// Attitude Variables
double hatroll = 0, hatpitch = 0, hatyaw = 0;
double roll_bias = 0, pitch_bias = 0, yaw_bias = 0; // bias at t=0
double roll = 0, pitch = 0, yaw = 0;
double roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
// Altitude Variables
#if defined(BARO)
double baro_P, baro_P_start;
double baro_hatz = 0;
double baro_altitude = 0, baro_altitude_bias = 0;
double hatv_z = 0, accel_z_start = -1;
double hataltitude = 0, hataltitude_bias = 0;
double altitude_bias = 0;
double altitude = 0;
double altitude_sum = 0;
#elif defined(SONAR)
double sonar_hatz = 0;
double hataltitude = 0, hataltitude_bias = 0;
double altitude_bias = 0;
double altitude = 0;
double altitude_sum = 0;
#endif

/*
   Vehicle Inputs(i.e. from Control)
*/
double roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
double roll_feedback = 0, pitch_feedback = 0, altitude_feedback = 0, yaw_feedback = 0;
double roll_controller = 0, pitch_controller = 0, altitude_controller = 0, yaw_controller = 0;
double umax = 50;
double u[4];

/*
   PID Variables
*/
#if defined(PID_v1)
double roll_d = 0, pitch_d = 0, yaw_d = 0;
double kp = 1.8, ki = 0.002, kd = 4.8;
#if defined(BARO) || defined(SONAR)
double altitude_d = -0.4;
double kp_z = 0.25, ki_z = 0.001, kd_z = 2.4;
#endif
#elif defined(PID_v2)
double roll_d = 0, pitch_d = 0, yaw_d = 0;
double yd[3] = {roll_d, pitch_d, yaw_d};
double K[3] = {2.8, 2.8, 2.8};
double Ti[3] = {20, 20, 20};
double Td[3] = {0.2, 0.2, 0.2};
double N[3] = {8, 8, 8};
double e[3] = {0, 0, 0};
double eold[3] = {0, 0, 0};
double P[3] = {0, 0, 0};
double I[3] = {0, 0, 0};
double D[3] = {0, 0, 0};
double Dold[3] = {0, 0, 0};
double PID[3] = {0, 0, 0};
#endif

/*
   Interrupt Varibales
*/
const int pulse_min = Throttle_MIN, pulse_max = Throttle_MAX - 400;
volatile int duration, pulse1, pulse2, pulse3, pulse4;
volatile boolean prestate0 = 0, prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising0, rising1, rising2, rising3, rising4;

/*
   Electronic Speed Controller
*/
boolean esc_switch = 0;
int MOTOR[4] = {pulse_min, pulse_min, pulse_min, pulse_min};

/*
   Loop Variables
*/
double dt = 0.01;
int loop_index = 0;
int time_diff;
unsigned long time_old, time_new;
unsigned long time_1, time_2; // debug


void setup() {
  LED();
  ESC();
  Receiver();
  Sensor();
  led_high(); // indicate config finished and wait the order
#ifdef DEBUG
  Serial.begin(115200);
#else
  while (pulse1 <= (Throttle_MAX - 10) || pulse2 <= (Throttle_MAX - 10));
#endif
  led_low();  // inidicate unlock
  esc_switch = 1; // unlock motor
  time_old = millis();
}

void loop() {
  time_new = millis();
  if (time_new - time_old >= dt * 1000 ) {
    // loop time
#ifdef DEBUG
    time_diff = time_new - time_old;
#endif
    time_old = time_new;
    // altitude
#if defined(BARO)
    sample_baro();
    altitude_filter();
    altitude_calibrate();
#elif defined(SONAR)
    sample_sonar();
    altitude_filter();
    altitude_calibrate();
#endif
    // attitude
    sample_IMU();
    attitude_filter();
    attitude_calibrate();
    // control
    Control();
    ESC();
    if (loop_index < 700) loop_index++;
  }
#ifdef DEBUG
  Debug();
#endif
}
