/****************************************************************************/
/*
   This project is maintained by Wenchao Lei, email: wcleithinking@gmail.com.
   PID + STR Mode
   Test the library: STR.h
*/
/****************************************************************************/
/*
                    (CW)          |                 (CW)      (CCW)
   QuadP :           0            |   QuadX :         0         1
           (CCW)3   CoM   1(CCW)  |                       CoM
                     2            |                   3         2
                    (CW)          |                 (CCW)      (CW)
*/
/****************************************************************************/
#include "Config.h"
#include <Servo.h>
#include <IMU.h>
#include <STR.h>
#include <SdFat.h>
Servo     motor[4];
IMU       myIMU;
STR       mySTR;
/****************************************************************************/
// States
float Gyro_threemeasure[3]  = {0, 0, 0};
float Gyro_biasestimate[3]  = {0, 0, 0};
float Angle_oldestimate[3]  = {0, 0, 0};
float Angle_newestimate[3]  = {0, 0, 0};
float Angle_offlinebias[3]  = {0, 0, 0};
float Angle_eulerdesire[3]  = {0, 0, 0};
float Rate_threemeasure[3]  = {0, 0, 0};
float Rate_middledesire[3]  = {0, 0, 0};
// Motor
bool ARM_flag = 0;
uint16_t RC[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_ref[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_out[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
// Main Loop
float dt = LOOP_period / 1000;
uint32_t loop_index  = 0;
uint32_t time_diff[4] = {0, 0, 0, 0}; // timer0, timer1, timer2, timer3 for loop, roll, pitch, yaw, respectively
uint8_t time_Index = 0;
volatile unsigned long time_previous[4], time_current[4];
volatile unsigned long time_start, time_end;
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
#ifdef STR_v1
  float atemp = 54, btemp = 64;
  float a1temp = -1 - exp(atemp * STR_period / 1000);
  float a2temp = exp(atemp * STR_period / 1000);
  float b0temp = btemp / atemp * STR_period + btemp / (atemp * atemp) * (a2temp - 1);
  float b1temp = btemp / atemp * (1 - a2temp) * (1 - a2temp) / atemp - b0temp * a2temp;
  mySTR.init_estimator((float)0.99, (float)5, a1temp, a2temp, b0temp, b1temp);
  mySTR.init_controller(0.7, 5, 0, STR_period / 1000);
#endif
#ifdef LOG
  Log_init();
#endif
  Led_blink(3);
  ARM_flag = 0;
  Led_armstate(ARM_flag);
}

void loop() {
  while (ARM_flag == 0) {
    Led_armstate(ARM_flag);
    Receiver_copy();
    if (RC[IndexRoll] <= (PWM_MAX - 10) || RC[IndexPitch] <= (PWM_MAX - 10)) {
      ARM_flag = 0;
      Led_armstate(ARM_flag);
    }
    else {
      ARM_flag = 1;
      Led_armstate(ARM_flag);
      Controller_cleardata();
      for (int i = 0; i < 4; i++) time_previous[i] = millis();
    }
  }
  time_current[0] = millis();
  if (time_current[0] - time_previous[0] >= LOOP_period ) {
#ifdef DEBUG
    time_diff[0] = time_current[0] - time_previous[0];
#endif
    time_previous[0] = time_current[0];
    myIMU.sample();
    myIMU.copygyro(Gyro_threemeasure);
    myIMU.attitude_filter(Angle_oldestimate, Gyro_biasestimate);
    Sensor_calibrate();
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
void Sensor_calibrate() {
  if (loop_index < 100) {
  }
  else if ((loop_index >= 100) && (loop_index < 300)) {
    for (int i = 0; i < 3; i++) Angle_offlinebias[i]  += Angle_oldestimate[i];
  }
  else {
    if (loop_index == 300) {
      for (int i = 0; i < 3; i++) Angle_offlinebias[i] *= 0.005;
    }
    for (int i = 0; i < 3; i++) {
      Angle_newestimate[i]  = Angle_oldestimate[i] - Angle_offlinebias[i];
      Rate_threemeasure[i]  = Gyro_threemeasure[i] - Gyro_biasestimate[i];
    }
  }
}
