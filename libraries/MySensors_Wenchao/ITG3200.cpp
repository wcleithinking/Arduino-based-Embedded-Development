#include "ITG3200.h"

ITG3200::ITG3200() {
  device_address = ITG3200_address;
}

void ITG3200::gyro_config() {
  // reset the sensor and the internal registers to the power-up-default settings.
  Wire.beginTransmission(device_address);
  Wire.write(ITG3200_reg_pwr_mgm);
  Wire.write(0b10000000);
  Wire.endTransmission();
  // set the clock reference.
  Wire.beginTransmission(device_address);
  Wire.write(ITG3200_reg_pwr_mgm);
  Wire.write(0b00000011);  // set the z gyro as the clock reference. Otherwise, please use 0b00000000 to choose internal oscillator, use 0b00000001 to choose x gyro and use 0b00000010 to choose y gyro.
  Wire.endTransmission();
  // set the full-scale range and internal sample rate.
  Wire.beginTransmission(device_address);
  Wire.write(ITG3200_reg_dlpf_fs);
  Wire.write(0b00011100);  // set the sensor in full resolution mode, i.e. -2000 deg/sec<-->+2000 deg/sec, and set the bandwidth of the digital low pass filter as 20HZ which results an internal sample rate: 1kHZ.
  Wire.endTransmission();
  // set the sample rate divider.
  Wire.beginTransmission(device_address);
  Wire.write(ITG3200_reg_smplrt_div);
  Wire.write(0b00001001);  // set the sample rate divider as 9 and hence the output data rate is 1kHZ/(9+1)=100HZ (coincides with the sample rate of the accelerometer).
  Wire.endTransmission();
}

void ITG3200::temp_sample(float *temp) {
  Wire.beginTransmission(device_address);
  Wire.write(ITG3200_reg_temp_out_h);
  Wire.endTransmission();
  Wire.requestFrom(device_address, 2);
  int i = 0;
  uint16_t buff[2];
  int16_t temp_reg;
  while (Wire.available() && i < 2) buff[i++] = Wire.read();
  temp_reg = (buff[1] | (buff[0] << 8));
  *temp = temp_reg/280 + 82.142857;
}

void ITG3200::gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z) {
  Wire.beginTransmission(device_address);
  Wire.write(ITG3200_reg_gyro_xout_h);
  Wire.endTransmission();
  Wire.requestFrom(device_address, 6);
  int i = 0;
  uint16_t buff[6];
  int16_t gyro_x_reg, gyro_y_reg, gyro_z_reg;
  while (Wire.available() && i < 6) buff[i++] = Wire.read();
  gyro_x_reg = (buff[1] | (buff[0] << 8));
  gyro_y_reg = (buff[3] | (buff[2] << 8));
  gyro_z_reg = (buff[5] | (buff[4] << 8));
  *gyro_x = gyro_x_reg * 0.069565217391304 * 0.017453292519943;
  *gyro_y = -gyro_y_reg * 0.069565217391304 * 0.017453292519943;
  *gyro_z = -gyro_z_reg * 0.069565217391304 * 0.017453292519943;
}

