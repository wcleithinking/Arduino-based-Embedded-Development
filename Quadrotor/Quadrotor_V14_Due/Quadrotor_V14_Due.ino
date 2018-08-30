/****************************************************************************/
/*
   This project is maintained by Wenchao Lei, email: wcleithinking@gmail.com.
   PID + STR Mode
   Test the library: STR.h
*/
/****************************************************************************/
#ifdef LOG
// SdFat
#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <SysCall.h>
#endif
// Servo
#include <Servo.h>
// IMU
#include <IMU.h>
#ifdef STR_v1
// STR
#include <STR.h>
#endif
// Config
#include "Config.h"
/****************************************************************************/
/*
                    (CW)          |                 (CW)      (CCW)
   QuadP :           0            |   QuadX :         0         1
           (CCW)3   CoM   1(CCW)  |                       CoM、
                     2            |                   3         2
                    (CW)          |                 (CCW)      (CW)
*/
/****************************************************************************/
Servo     motor[4];
IMU       myIMU;
#ifdef STR_v1
STR       mySTR;
#endif
/****************************************************************************/
// States of Vehicle, from Sensors and Filters
//float Gyro_measure_old[3]   = {0, 0, 0};
//float Gyro_measure_new[3]   = {0, 0, 0};
float Gyro_measure[3]       = {0, 0, 0};
float Gyro_bias_estimate[3] = {0, 0, 0};
float Angle_estimate[3]     = {0, 0, 0};
float Angle_measure[3]      = {0, 0, 0};
float Angle_bias[3]         = {0, 0, 0};
float Angle_desire[3]       = {0, 0, 0};
float Rate_measure[3]       = {0, 0, 0};
float Rate_bias[3]          = {0, 0, 0};
float Rate_desire[3]        = {0, 0, 0};
#ifdef STR_v1
uint8_t STR_flag = 0;
float a1, a2, b0, b1;
#endif
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
/****************************************************************************/
/****************************************************************************/
void setup() {
   /*
    * Initialize the baud-rate of the serial port (if DEBUG has been defined).
    */
#ifdef DEBUG
  Serial.begin(BaudRate);
#endif
  /*
   * Initialize the LED, Motor and Receiver.
   */
  Led_init();       //  set the LED pin
  Led_low();        //  close the LED
  Motor_init();     //  set the four motor pins
  Motor_update();   //  set the PWM level of four motors as minimum
  Receiver_init();  //  set the four channel pins
  myIMU.init(3, 4, 3, 4, 1, dt);  //  init the IMU device
  /*
   * Initialize the STR (if STR_v1 has been defined).
   */
#ifdef STR_v1
  float atemp = 54, btemp = 64;
  float a1temp = -1 - exp(atemp * STR_period / 1000);
  float a2temp = exp(atemp * STR_period / 1000);
  float b0temp = btemp / atemp * STR_period + btemp / (atemp * atemp) * (a2temp - 1);
  float b1temp = btemp / atemp * (1 - a2temp) * (1 - a2temp) / atemp - b0temp * a2temp;
  mySTR.init_estimator((float)0.99, (float)5, a1temp, a2temp, b0temp, b1temp);
  mySTR.init_controller(0.7, 5, 0, STR_period / 1000);
#endif
  /*
   * Initialize the logger (if LOG has been defined).
   */
#ifdef LOG
  Log_init();
#endif
  /*
   * Blink the LED three times to indicate that the setup process has been finished and then close the LED.
   */
  Led_blink(3);
  ARM_flag = 0;
  Led_armstate();
}

void loop() {
  /*
   * Wait until the signals from roll and pitch channels reach the given region (Locker).
   */
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
      Controller_cleardata(); // initialize the controller
      for (int i = 0; i < 4; i++) time_previous[i] = millis();  // record the time
    }
  }
  time_current[0] = millis();
  /*
   * Work at sampling time.
   */
  if (time_current[0] - time_previous[0] >= dt * 1000 ) {
#ifdef DEBUG
    time_diff[0] = time_current[0] - time_previous[0];
#endif
    time_previous[0] = time_current[0];
    myIMU.sample(); //  get the raw data from IMU
    myIMU.copygyro(Gyro_measure); //  get the original angular rates
    myIMU.attitude_filter(Angle_estimate, Gyro_bias_estimate);  //  get the original Euler angles and gyro biases
    Angle_calibrate();  //  calibrate the Euler angles and angular rates
    Receiver_copy();  //  get the new receiver signals
    Motor_control();  //  calculate the new motor levels based on controllers  
    Motor_update(); //  update the motor levels
    Controller_updatedata();  // update the controller data for the next calculate
    /*
     * Record the important data (if LOG has been defined).
     */
#ifdef LOG
#ifdef DEBUG
    time_start = millis();
    Log_savedata();
    time_end = millis();
#else
    Log_savedata();
#endif
#endif
    if (loop_index < 350) loop_index++; // used by calibrator
  }
  /*
   * Output the serial port data (if DEBUG has been defined). 
   */
#ifdef DEBUG
  Serial_debug();
#endif
}
/****************************************************************************/
/****************************************************************************/
