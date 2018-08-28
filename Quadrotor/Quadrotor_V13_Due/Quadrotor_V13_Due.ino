/****************************************************************************/
/*
   This project is maintained by Wenchao Lei, email: wcleithinking@gmail.com.
   PID + STR Mode
*/
/****************************************************************************/
// SdFat
#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <SysCall.h>
// Servo
#include <Servo.h>
// IMU
#include <IMU.h>
// Config
#include "Config.h"
/****************************************************************************/
/*
                    (CW)          |                 (CW)      (CCW)
   QuadP :           0            |   QuadX :         0         1
           (CCW)3   CoM   1(CCW)  |                       CoM
                     2            |                   3         2
                    (CW)          |                 (CCW)      (CW)
*/
/****************************************************************************/
Servo     motor[4];
IMU       myIMU;
/****************************************************************************/
// States of Vehicle, from Sensors and Filters
float Gyro_measure_old[3]   = {0, 0, 0};
float Gyro_measure_new[3]   = {0, 0, 0};
float Gyro_measure[3]       = {0, 0, 0};
float Gyro_bias_estimate[3] = {0, 0, 0};
float Angle_estimate[3]     = {0, 0, 0};
float Angle_measure[3]      = {0, 0, 0};
float Angle_bias[3]         = {0, 0, 0};
float Angle_desire[3]       = {0, 0, 0};
float Rate_measure[3]       = {0, 0, 0};
float Rate_bias[3]          = {0, 0, 0};
float Rate_desire[3]        = {0, 0, 0};
/****************************************************************************/
// Motor
bool ARM_flag = 0;
uint16_t RC[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_ref[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_out[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
// Main Loop
float dt = 0.01;
int loop_index  = 0;
// timer0, timer1, timer2, timer3 for loop, roll, pitch, yaw, respectively
int time_diff[4]   = {0, 0, 0, 0};
uint8_t time_Index = 0;
unsigned long time_previous[4], time_current[4];
unsigned long time_start, time_end;
// Log
#ifdef LOG
unsigned long errorCount = 0;
#endif
/****************************************************************************/
/****************************************************************************/
void setup() {
#ifdef DEBUG
  Serial.begin(BaudRate);
#endif
  Led_init();
  Led_low();
  Motor_init();
  Motor_update();
  Receiver_init();
  myIMU.init(3, 4, 3, 4, 1, dt);
#ifdef LOG
  Log_init();
#endif
  Led_blink(3);
  ARM_flag = 0;
  Led_armstate();
}

void loop() {
  while (ARM_flag == 0) {
    Led_armstate();
    Receiver_copy();
    if (RC[IndexRoll] <= (PWM_MAX - 10) || RC[IndexPitch] <= (PWM_MAX - 10)) {
      ARM_flag = 0;
      Led_armstate();
    }
    else {
      ARM_flag = 1;
      Led_armstate();
      // prepare the controller
      Controller_cleardata();
      for (int i=0;i<4;i++) time_previous[i] = millis();
    }
  }
  time_current[0] = millis();
  if (time_current[0] - time_previous[0] >= dt * 1000 ) {
#ifdef DEBUG
    time_diff[0] = time_current[0] - time_previous[0];
#endif
    time_previous[0] = time_current[0];
    myIMU.sample();
    myIMU.copygyro(Gyro_measure_old);
    myIMU.attitude_filter(Angle_estimate, Gyro_bias_estimate);
    Angle_calibrate();
    Receiver_copy();
    Motor_control();
    Motor_update();
    Controller_updatedata();
#ifdef LOG
#ifdef DEBUG
    time_start = millis();
    Log_savedata();
    time_end = millis();
#else
    Log_savedata();
#endif
#endif
    if (loop_index < 350) loop_index++;
  }
#ifdef DEBUG
  Serial_debug();
#endif
}
/****************************************************************************/
/****************************************************************************/
