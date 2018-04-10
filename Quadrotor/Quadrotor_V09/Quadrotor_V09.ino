/*   **************************   Log   **********************
     @Project name: Quadrotor_V9, to test the PID controller and the STR controller.
     @Created by Wenchao Lei, Oct. 25, 2016: initial version, copy the original code from project: Quadrotor_V5 and Quadrotor_V8.
     @Roll and Pitch Dynamics are controlled by a well tunned PID controller!
     Modified by Wenchao Lei, Oct. 26, 2016--Nov. 5, 2016: add and modify the baro sensor and sonar sensor to test altitude hold, failed!!!
     Modified by Wenchao Lei, Nov. 6, 2016: modify the structure to make read and debug easier, i.e. add #define command.
     Modified by Wenchao Lei, Nov. 7, 2016: add new control algorithm to test yaw control by using Indirect STR Controller.
     Modified by Wenchao Lei, Nov. 8, 2016: modify the attitude filter for yaw, i.e. reduce the order of the kalman filter by ignore gyro_z_bias.
     Modified by Wenchao Lei, Nov. 11, 2016: modify the STR controller for yaw-dynamics, version 1, using Rad signal.
     Modified by Wenchao Lei, Nov. 12, 2016: modify the STR controller for yaw-dynamics, version 2, using Voltage percentage to decide the PWM.
     Modified by Wenchao Lei, Nov. 13, 2016: modify the STR controller for yaw-dynamics, version 3, using Degree instead of Rad, success!!!
     Modified by Wenchao Lei, Nov. 14, 2016: modify the STR controller for yaw-dynamics, version 4 and add EEPROM function.
     Modified by Wenchao Lei, Nov. 15, 2016: remove the gyro_x_bias and gyro_y_bias in PID controller.
     Modified by Wenchao Lei, Nov. 16, 2016: (1) simplify the structure of the code;
                                             (2) add PID_v3;
                                             (3) modify the STR controller for yaw-dynamics, version 5, considering controller windup;
                                             (4) remove BARO and SONAR for simplicity.
*/

/*
   Header Files
*/
#include "Config.h"
#include "MPU9250.h"
MPU9250 my_mpu9250;

#ifdef EEPROM_SAVE
#include <EEPROM.h>
#endif
#ifdef SD_SAVE
#include <SD.h>
#endif

/*
   Vehicle Outputs, i.e. from Sensors
*/
// IMU Variables
float accel_x, accel_y, accel_z;// output of accel
float gyro_x, gyro_y, gyro_z;// output of gyro
float mag_x, mag_y, mag_z;// output of mag
float accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0;// bias of accel
float gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;// bias of gyro
float mag_x_bias = 0, mag_y_bias = 0, mag_z_bias = 0;// bias of mag

/*
   Vehicle States, i.e. from Filters
*/
// Attitude Variables
float hatroll = 0, hatpitch = 0, hatyaw = 0;// from attitude filter
float roll_bias = 0, pitch_bias = 0, yaw_bias = 0;// initial bias
float roll = 0, pitch = 0, yaw = 0;// used by controller
float roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
float droll_sum = 0, dpitch_sum = 0, dyaw_sum = 0;
float roll_d = 0, pitch_d = 0, yaw_d = 0;// desired attitude

/*
   Vehicle Inputs, i.e. from Control
*/
float umax = 50;
float roll_feedforward = 0, pitch_feedforward = 0, altitude_feedforward = 0, yaw_feedforward = 0;
float roll_feedback    = 0, pitch_feedback    = 0, altitude_feedback    = 0, yaw_feedback    = 0;
float roll_controller  = 0, pitch_controller  = 0, altitude_controller  = 0, yaw_controller  = 0;

/*
	PID Variables
*/
#if defined(PID_v1)
float kp = 1.8, ki = 0.02, kd = 4.8;
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
#elif defined(PID_v3)
float kp = 1.8, ki = 0.02, kd = 4;
float kp_d = 1, ki_d = 0.02;
#endif

/*
   STR Variables
*/
#ifdef STR_YAW
float yaw_d_oo = 0, yaw_d_o = 0;
float yaw_ooo = 0, yaw_oo = 0, yaw_o = 0;
float v_yaw_oo = 0, v_yaw_o = 0, v_yaw = 0;
float u_yaw_ooo = 0, u_yaw_oo = 0, u_yaw_o = 0;
float am1 = -1.4431986644, am2 = 0.770898934, am3 = -0.13806924, ao1 = 0.8, ao2 = 0.16;
float hata1 = 1, hata2 = 1, hata3 = 1, hatb0 = 1, hatb1 = 1, hatb2 = 1;
bool STR_YAW_flag = 0;
int time_yaw_diff;
unsigned long time_yaw_new, time_yaw_old;
#endif

/*
   Interrupt Varibales
*/
volatile int pulse1, pulse2, pulse3, pulse4;
volatile bool prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising1, rising2, rising3, rising4;

/*
   Electronic Speed Controller
*/
bool esc_switch = 0;
int PWM_ref[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_out[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int MOTOR[4] = {127, 127, 127, 127};

/*
   Loop Variables
*/
float dt = 0.01;
int loop_index = 0;
int time_diff;
unsigned long time_old, time_new, time_1, time_2;


void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // indicate start config
  Motor();
  Receiver();
  Sensor();
  digitalWrite(ledPin, LOW);  // indicate config finished
  delay(2000);
  digitalWrite(ledPin, HIGH);
  delay(2000);
#ifdef DEBUG
  Serial.begin(115200);
#else
  while (pulse1 <= (PWM_MAX - 10) || pulse2 <= (PWM_MAX - 10));
#endif
  digitalWrite(ledPin, LOW);  // indicate unlock
  esc_switch = 1;             // unlock motor
#ifdef STR_YAW
  time_yaw_old = millis();
#endif
  time_old = millis();
}

void loop() {
  time_new = millis();
  if (time_new >= time_old + dt * 1000 ) {
#ifdef DEBUG
    time_diff = time_new - time_old;
#endif
    time_old = time_new;
    // attitude
    sensor_sample();
    attitude_filter();  // 7~8ms
    attitude_calibrate();
    // control
    motor_control();
    motor_update();
    PID_updatedata();
#ifdef STR_YAW
    STR_YAW_updatedata();
#endif
    if (loop_index < 350) loop_index++;
  }
#ifdef DEBUG
  Debug();
#endif
}
