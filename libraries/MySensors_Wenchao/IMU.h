#ifndef _IMU_H_
#define _IMU_H_

#include "MPU9250.h"



class IMU {
public:
    IMU();
    void init(uint8_t AFS_SEL, uint8_t ADLPF_CFG, uint8_t GFS_SEL, uint8_t GDLPF_CFG, uint8_t MSS_SEL, float dt);
    void sample();
    void copyaccel(float Accel[3]);
    void copygyro(float Gyro[3]);
    void copymag(float Mag[3]);
    void accel_rollpitch();
    void mag_yaw();
    void kalman_filter_roll();
    void kalman_filter_pitch();
    void kalman_filter_yaw();
    void attitude_filter(float Angle_estimate[3], float Gyro_bias_estimate[3]);
private:
    MPU9250   mySensor;
    float period;
    float accel_x, accel_y, accel_z;    // output of accel
    float gyro_x, gyro_y, gyro_z;       // output of gyro
    float mag_x, mag_y, mag_z;          // output of mag
    float roll_accel, pitch_accel, yaw_mag;      // estimated roll and pitch from accel only
    float roll_state[2];                // initial value of kalman_filter_roll
    float pitch_state[2];               // initial value of kalman_filter_pitch
    float yaw_state[2];                 // initial value of kalman_filter_yaw
    //float yaw_state[4];                 // initial value of kalman_filter_yaw
    float roll_P[2][2];
    float pitch_P[2][2];
    float yaw_P[2][2];
    //float yaw_P[4][4];
};
#endif

