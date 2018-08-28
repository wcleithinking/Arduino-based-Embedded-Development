#ifndef _ITG3200_H_
#define _ITG3200_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address of ITG3200
#define ITG3200_address             0x68  // assume that pin 9 (AD0) is logic low. If it cannot work, we will try 0b1101001.

// register map
#define ITG3200_reg_who_am_i        0x00  // I2C address of the sensor
#define ITG3200_reg_smplrt_div      0x15  // sample rate divider
#define ITG3200_reg_dlpf_fs         0x16  // full-scale range and digital low pass filter
#define ITG3200_reg_int_cfg         0x17  // interrupt configuration
#define ITG3200_reg_int_status      0x1A  // interrupt status
#define ITG3200_reg_temp_out_h      0x1B  // temperature data in high bits
#define ITG3200_reg_temp_out_l      0x1C  // temperature data in low bits
#define ITG3200_reg_gyro_xout_h     0x1D  // x gyro output data in high bits
#define ITG3200_reg_gyro_xout_l     0x1E  // x gyrp output data in low bits
#define ITG3200_reg_gyro_yout_h     0x1F  // y gyro output data in high bits
#define ITG3200_reg_gyro_yout_l     0x20  // y gyro output data in low bits
#define ITG3200_reg_gyro_zout_h     0x21  // z gyrp output data in high bits
#define ITG3200_reg_gyro_zout_l     0x22  // z gyrp output data in low bts
#define ITG3200_reg_pwr_mgm         0x3E  // power management

class ITG3200 {
  public:
    ITG3200();
    void gyro_config();
    void temp_sample(float *temp);
    void gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z);
  private:
  uint8_t device_address;
};

#endif
