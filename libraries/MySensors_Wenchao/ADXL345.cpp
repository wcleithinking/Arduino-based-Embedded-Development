#include "ADXL345.h"

ADXL345::ADXL345() {
  device_address = ADXL345_address;
}

void ADXL345::accel_config() {
  // place the sensor into standby mode by setting the measure bit as 0.
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_power_ctl);
  Wire.write(0b00000000);
  Wire.endTransmission();
  // place the sensor into measurement mode by setting the measure bit as 1.
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_power_ctl);
  Wire.write(0b00001000);
  Wire.endTransmission();
  // set the offset of x-axis with maximum offset 2g: 0x7f, i.e. about 15.6mg/LSB.
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_ofsx);
  Wire.write(0x00);
  Wire.endTransmission();
  // set the offset of y-axis with maximum offset 2g: 0x7f, i.e. about 15.6mg/LSB.
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_ofsy);
  Wire.write(0x00);
  Wire.endTransmission();
  // set the offset of z-axis with maximum offset 2g: 0x7f, i.e. about 15.6mg/LSB.
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_ofsz);
  Wire.write(0x00);
  Wire.endTransmission();
  // set the data rate.
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_bw_rate);
  Wire.write(0b00001010);  // set the output data rate as 100HZ (default).
  Wire.endTransmission();
  // set the data format.
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_data_format);
  Wire.write(0b00001011);  // set the sensor in full resolution mode to maintain a 4mg/LSB scale factor, and set the g range as -16g<-->+16g.
  Wire.endTransmission();
}

void ADXL345::accel_sample(float *accel_x, float *accel_y, float *accel_z) {
  Wire.beginTransmission(device_address);
  Wire.write(ADXL345_reg_datax0);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_address, 6);
  int i = 0;
  uint16_t buff[6];
  int16_t accel_x_reg, accel_y_reg, accel_z_reg;
  while (Wire.available() && i < 6) buff[i++] = Wire.read();
  accel_x_reg = (buff[0] | (buff[1] << 8));
  accel_y_reg = (buff[2] | (buff[3] << 8));
  accel_z_reg = (buff[4] | (buff[5] << 8));
  *accel_x = accel_x_reg * 0.004;
  *accel_y = -accel_y_reg * 0.004;
  *accel_z = -accel_z_reg * 0.004;
}

