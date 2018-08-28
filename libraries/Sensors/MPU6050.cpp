#include "MPU6050.h"

MPU6050::MPU6050() {
  device_address = MPU6050_address;
}

void MPU6050::acceltempgyro_config(uint8_t AFS_SEL, uint8_t GFS_SEL, uint8_t DLPF_CFG) {
  // set the power management 1.
  	reg_write(device_address, MPU6050_reg_pwr_mgmt_1, 0b00000001);  // select the X axis gyroscope reference as the clock source.
  // set the power management 2.
    reg_write(device_address, MPU6050_reg_pwr_mgmt_2, 0b00000000);  // all sensors are on.
  // set the configuration 1 of accelerometer
  	switch (AFS_SEL) {
    // [4:3]
  		case 0:
    // set the resolution as +-2g(00) with sensitivity scale factor 16384 LSB/g
  		reg_write(device_address, MPU6050_reg_accel_config, 0b00000000);
  		accel_ssf = 0.00006103515625;
  		break;
  		case 1:
    // set the resolution as +-4g(01) with sensitivity scale factor 8192 LSB/g
  		reg_write(device_address, MPU6050_reg_accel_config, 0b00001000);
  		accel_ssf = 0.0001220703125;
  		break;
  		case 2:
    // set the resolution as +-8g(10) with sensitivity scale factor 4096 LSB/g
  		reg_write(device_address, MPU6050_reg_accel_config, 0b00010000);
  		accel_ssf = 0.000244140625;
  		break;
  		case 3:
    // set the resolution as +-16g(11) with sensitivity scale factor 2048 LSB/g
  		reg_write(device_address, MPU6050_reg_accel_config, 0b00011000);
  		accel_ssf = 0.00048828125;
  		break;
  		default:
    // set the default resolution as +-16g(11) with sensitivity scale factor 2048 LSB/g
  		reg_write(device_address, MPU6050_reg_accel_config, 0b00011000); 
  		accel_ssf = 0.00048828125;
  		break;
  	}
  // set the configuration of gyroscope
  	switch (GFS_SEL) {
    // [4:3]
  		case 0:
    // set the resolution as +-250dps(00) with sensitivity scale factor 131 LSB/dps
  		reg_write(device_address, MPU6050_reg_gyro_config, 0b00000000);
  		gyro_ssf = 0.00763358778625954198473282442748;
  		break;
  		case 1:
    // set the resolution as +-500dps(01) with sensitivity scale factor 65.5 LSB/dps
  		reg_write(device_address, MPU6050_reg_gyro_config, 0b00001000);
  		gyro_ssf = 0.01526717557251908396946564885496;
  		break;
  		case 2:
    // set the resolution as +-1000dps(10) with sensitivity scale factor 32.8 LSB/dps
  		reg_write(device_address, MPU6050_reg_gyro_config, 0b00010000);
  		gyro_ssf = 0.03048780487804878048780487804878;
  		break;
  		case 3:
    // set the resolution as +-2000dps(11) with sensitivity scale factor 16.4 LSB/dps
  		reg_write(device_address, MPU6050_reg_gyro_config, 0b00011000);
  		gyro_ssf = 0.06097560975609756097560975609756;
  		break;
  		default:
    // set the resolution as +-2000dps(11) with sensitivity scale factor 16.4 LSB/dps
  		reg_write(device_address, MPU6050_reg_gyro_config, 0b00011000);
  		gyro_ssf = 0.06097560975609756097560975609756;
  		break; 
  	}
  // set the configuration of device.
  switch (DLPF_CFG) {  // DLPF for accel & gyro
    // [2:0]
  	case 0:
    // set the DLPF bandwidth as 260 & 256HZ with output data rate 8kHZ, delay 0.00 & 0.97ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000000);
  	break;
  	case 1:
    // set the DLPF bandwidth as 184 & 188HZ with output data rate 1kHZ, delay 2.00 & 1.90ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000001);
  	break;
  	case 2:
    // set the DLPF bandwidth as 94 & 98HZ with output data rate 1kHZ, delay 3.00 & 2.80ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000010);
  	break;
  	case 3:
    // set the DLPF bandwidth as 44 & 42HZ with output data rate 1kHZ, delay 4.90 & 4.80ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000011);
  	break;
  	case 4:
    // set the DLPF bandwidth as 21 & 20HZ with output data rate 1kHZ, delay 8.50 & 8.30ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000100);
  	break;
  	case 5:
    // set the DLPF bandwidth as 10 & 10HZ with output data rate 1kHZ, delay 13.80 & 13.40ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000101);
  	break;
  	case 6:
    // set the DLPF bandwidth as 5 & 5HZ with output data rate 1kHZ, delay 19.60 & 18.6ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000110);
  	break;
  	case 7:
    // reserved with output data rate 8kHZ
  	reg_write(device_address, MPU6050_reg_config, 0b00000111);
  	break;
  	default:
    // set the default DLPF bandwidth as 20HZ with output data rate 1kHZ, delay 9.90ms
  	reg_write(device_address, MPU6050_reg_config, 0b00000100);
  	break;
  }
  // set I2C bypass mode.
  reg_write(device_address, MPU6050_reg_int_pin_cfg, 0b00000010);
}


void MPU6050::accel_sample(float *accel_x, float *accel_y, float *accel_z) {
	reg_read(device_address, MPU6050_reg_accel_xout_h, 6);
	uint16_t buff[6];
	int16_t accel_x_reg, accel_y_reg, accel_z_reg;
	int i = 0;
	while (Wire.available() && i < 6) buff[i++] = Wire.read();
	accel_x_reg = (buff[1]  | (buff[0] << 8));
	accel_y_reg = (buff[3]  | (buff[2] << 8));
	accel_z_reg = (buff[5]  | (buff[4] << 8));
	*accel_x =  accel_x_reg * accel_ssf;
	*accel_y = -accel_y_reg * accel_ssf;
	*accel_z = -accel_z_reg * accel_ssf;
}

void MPU6050::temp_sample(float *temp) {
	reg_read(device_address, MPU6050_reg_temp_out_h, 2);
	uint16_t buff[2];
	int16_t temp_reg;
	int i = 0;
	while (Wire.available() && i < 2) buff[i++] = Wire.read();
	temp_reg = (buff[1]  | (buff[0] << 8));
	*temp = temp_reg/340+36.53;
}

void MPU6050::gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z) {
	reg_read(device_address, MPU6050_reg_gyro_xout_h, 6);
	uint16_t buff[6];
	int16_t gyro_x_reg, gyro_y_reg, gyro_z_reg;
	int i = 0;
	while (Wire.available() && i < 6) buff[i++] = Wire.read();
	gyro_x_reg  = (buff[1] | (buff[0] << 8));
	gyro_y_reg  = (buff[3] | (buff[2] << 8));
	gyro_z_reg  = (buff[5] | (buff[4] << 8)); 
	*gyro_x =  gyro_x_reg * gyro_ssf * 0.01745329251;
	*gyro_y = -gyro_y_reg * gyro_ssf * 0.01745329251;
	*gyro_z = -gyro_z_reg * gyro_ssf * 0.01745329251;
}

void MPU6050::reg_write(uint8_t address, uint8_t reg, uint8_t value) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

void MPU6050::reg_read(uint8_t address, uint8_t reg, uint8_t num) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, num);
}