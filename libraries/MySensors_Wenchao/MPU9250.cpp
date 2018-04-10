#include "MPU9250.h"

MPU9250::MPU9250() {
	device_address = MPU9250_address;
	device_mag_address = AK8963_address;
}

void MPU9250::all_config(uint8_t AFS_SEL, uint8_t ADLPF_CFG, uint8_t GFS_SEL, uint8_t GDLPF_CFG, uint8_t MSS_SEL) {
	acceltempgyro_config(AFS_SEL, ADLPF_CFG, GFS_SEL, GDLPF_CFG);
	delay(100);
	mag_config(MSS_SEL);
	delay(100);
}

void MPU9250::acceltempgyro_config(uint8_t AFS_SEL, uint8_t ADLPF_CFG, uint8_t GFS_SEL, uint8_t GDLPF_CFG) {
  // set the power management 1.
  reg_write(device_address, MPU9250_reg_pwr_mgmt_1, 0b10000000);  // reset the internal registers and restores the default settings.
  // set the power management 1.
  reg_write(device_address, MPU9250_reg_pwr_mgmt_1, 0b00000000);  // select the internal 20MHZ oscillator as the clock source.
  // set the power management 2.
  reg_write(device_address, MPU9250_reg_pwr_mgmt_2, 0b00000000);  // all sensors are on.
  // set the configuration 1 of accelerometer
  switch (AFS_SEL) {
    // [4:3]
  	case 0:
    // set the resolution as +-2g(00) with sensitivity scale factor 16384 LSB/g
  	reg_write(device_address, MPU9250_reg_accel_config_1, 0b00000000);
  	accel_ssf = 1.0/16384;
  	break;
  	case 1:
    // set the resolution as +-4g(01) with sensitivity scale factor 8192 LSB/g
  	reg_write(device_address, MPU9250_reg_accel_config_1, 0b00001000);
  	accel_ssf = 1.0/8192;
  	break;
  	case 2:
    // set the resolution as +-8g(10) with sensitivity scale factor 4096 LSB/g
  	reg_write(device_address, MPU9250_reg_accel_config_1, 0b00010000);
  	accel_ssf = 1.0/4096;
  	break;
  	case 3:
    // set the resolution as +-16g(11) with sensitivity scale factor 2048 LSB/g
  	reg_write(device_address, MPU9250_reg_accel_config_1, 0b00011000);
  	accel_ssf = 1.0/2048;
  	break;
  	default:
    // set the default resolution as +-16g(11) with sensitivity scale factor 2048 LSB/g
  	reg_write(device_address, MPU9250_reg_accel_config_1, 0b00011000); 
  	accel_ssf = 1.0/2048;
  	break;
  }
  // set the configuration 2 of accelerometer
  switch (ADLPF_CFG) {
    // [2:0]
  	case 0:
    // set the DLPF bandwidth as 460HZ with output data rate 1kHZ, delay 1.94ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000000);
  	break;
  	case 1:
    // set the DLPF bandwidth as 184HZ with output data rate 1kHZ, delay 5.80ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000001); 
  	break; 
  	case 2:
    // set the DLPF bandwidth as 92HZ with output data rate 1kHZ, delay 7.80ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000010); 
  	break;
  	case 3:
    // set the DLPF bandwidth as 41HZ with output data rate 1kHZ, delay 11.80ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000011); 
  	break;
  	case 4:
    // set the DLPF bandwidth as 20HZ with output data rate 1kHZ, delay 19.80ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000100); 
  	break;
  	case 5:
    // set the DLPF bandwidth as 10HZ with output data rate 1kHZ, delay 35.70ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000101); 
  	break;
  	case 6:
    // set the DLPF bandwidth as 5HZ with output data rate 1kHZ, delay66.96ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000110); 
  	break;
  	case 7:
    // set the DLPF bandwidth as 460HZ with output data rate 1kHZ, delay 1.94ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000111); 
  	break;
  	default:
    // set the default DLPF bandwidth as 20HZ with output data rate 1kHZ, delay 19.80ms
  	reg_write(device_address, MPU9250_reg_accel_config_2, 0b00000100);
  	break;
  }
  // set the configuration of gyroscope
  switch (GFS_SEL) {
    // [4:3]
  	case 0:
    // set the resolution as +-250dps(00) with sensitivity scale factor 131 LSB/dps
  	reg_write(device_address, MPU9250_reg_gyro_config, 0b00000000);
  	gyro_ssf = 1.0/131;
  	break;
  	case 1:
    // set the resolution as +-500dps(01) with sensitivity scale factor 65.5 LSB/dps
  	reg_write(device_address, MPU9250_reg_gyro_config, 0b00001000);
  	gyro_ssf = 1.0/65.5;
  	break;
  	case 2:
    // set the resolution as +-1000dps(10) with sensitivity scale factor 32.8 LSB/dps
  	reg_write(device_address, MPU9250_reg_gyro_config, 0b00010000);
  	gyro_ssf = 1.0/32.8;
  	break;
  	case 3:
    // set the resolution as +-2000dps(11) with sensitivity scale factor 16.4 LSB/dps
  	reg_write(device_address, MPU9250_reg_gyro_config, 0b00011000);
  	gyro_ssf = 1.0/16.4;
  	break;
  	default:
    // set the resolution as +-2000dps(11) with sensitivity scale factor 16.4 LSB/dps
  	reg_write(device_address, MPU9250_reg_gyro_config, 0b00011000);
  	gyro_ssf = 1.0/16.4;
  	break; 
  }
  // set the configuration of device.
  switch (GDLPF_CFG) {  // DLPF for gyro and temp
    // [2:0]
  	case 0:
    // set the DLPF bandwidth as 250HZ with output data rate 8kHZ, delay 0.97ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000000);
  	break;
  	case 1:
    // set the DLPF bandwidth as 184HZ with output data rate 1kHZ, delay 2.90ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000001);
  	break;
  	case 2:
    // set the DLPF bandwidth as 92HZ with output data rate 1kHZ, delay 3.90ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000010);
  	break;
  	case 3:
    // set the DLPF bandwidth as 41HZ with output data rate 1kHZ, delay 5.90ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000011);
  	break;
  	case 4:
    // set the DLPF bandwidth as 20HZ with output data rate 1kHZ, delay 9.90ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000100);
  	break;
  	case 5:
    // set the DLPF bandwidth as 10HZ with output data rate 1kHZ, delay 17.85ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000101);
  	break;
  	case 6:
    // set the DLPF bandwidth as 5HZ with output data rate 1kHZ, delay 33.48ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000110);
  	break;
  	case 7:
    // set the DLPF bandwidth as 3600HZ with output data rate 8kHZ, delay 0.17ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000111);
  	break;
  	default:
    // set the default DLPF bandwidth as 20HZ with output data rate 1kHZ, delay 9.90ms
  	reg_write(device_address, MPU9250_reg_config, 0b00000100);
  	break;
  }
  // set I2C bypass mode.
  reg_write(device_address, MPU9250_reg_int_pin_cfg, 0b00000010);
}

void MPU9250::mag_config(uint8_t MSS_SEL) {
  // initialize all registers
	reg_write(device_mag_address, AK8963_reg_cnt2, 0b00000001);
  // set power down mode
	reg_write(device_mag_address, AK8963_reg_cntl, 0b00000000);
	delay(1);
  // set read fuseROM mode
	reg_write(device_mag_address, AK8963_reg_cntl, 0b00001111);
	delay(1);
  // read sensitivity adjustment vlues
	reg_read(device_mag_address, AK8963_reg_asax, 3);
	uint8_t buff[3];
	int i=0;
	while (Wire.available() && i < 3) buff[i++] = Wire.read();
	asax = (float)(buff[0] - 128) / 256.0 + 1.0;
	asay = (float)(buff[1] - 128) / 256.0 + 1.0;
	asaz = (float)(buff[2] - 128) / 256.0 + 1.0;
  // set power down mode
	reg_write(device_mag_address, AK8963_reg_cntl, 0b00000000);
	delay(1);
  // set control 1 of magnetometer
	switch (MSS_SEL) {
		case 0:
    // set 14-bit output with sensitivity scale factor 0.6 uT/LSB and continous measurement mode 2: 100HZ
		reg_write(device_mag_address, AK8963_reg_cntl, 0b00000110);
		mag_ssf = 4912.0/8190.0;
		break;
		case 1:
    // set 16-bit output with sensitivity scale factor 0.15 uT/LSB and continous measurement mode 2: 100HZ
		reg_write(device_mag_address, AK8963_reg_cntl, 0b00010110);
		mag_ssf = 4912.0/32760.0;
		break;
		default:
    // set the default output as 16-bit with sensitivity scale factor 0.15 uT/LSB and continous measurement mode 2: 100HZ
		reg_write(device_mag_address, AK8963_reg_cntl, 0b00010110);
		mag_ssf = 4912.0/32760.0;
		break;
	} 
	delay(1);
}

void MPU9250::accel_sample(float *accel_x, float *accel_y, float *accel_z) {
	reg_read(device_address, MPU9250_reg_accel_xout_h,6);
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

void MPU9250::temp_sample(float *temp) {
	reg_read(device_address, MPU9250_reg_temp_out_h,2);
	uint16_t buff[2];
	int16_t temp_reg;
	int i = 0;
	while (Wire.available() && i < 2) buff[i++] = Wire.read();
	temp_reg    = (buff[1]  | (buff[0] << 8));
	*temp = 21 + (temp_reg * 0.0029951777638);
}

void MPU9250::gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z) {
	reg_read(device_address, MPU9250_reg_gyro_xout_h, 6);
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


void MPU9250::mag_sample(float *mag_x, float *mag_y, float *mag_z) {
	uint8_t data_status_1, data_status_2;
  // read register status 1.
	reg_read(device_mag_address, AK8963_reg_st1, 1);
	int i = 0;
	while (Wire.available() && i < 1) {
		data_status_1 = Wire.read();
		i++;
	}
  // read measurement data from AK8963.
	reg_read(device_mag_address, AK8963_reg_hxl, 6);
	uint16_t buff[6];
	int16_t mag_x_reg, mag_y_reg, mag_z_reg;
	i = 0;
	while (Wire.available() && i < 6) buff[i++] = Wire.read();
	mag_x_reg = (buff[0]  | (buff[1] << 8));
	mag_y_reg = (buff[2]  | (buff[3] << 8));
	mag_z_reg = (buff[4]  | (buff[5] << 8));

  // read register status 2.
	reg_read(device_mag_address, AK8963_reg_st2,1);
	i = 0;
	while (Wire.available() && i < 1) {
		data_status_2 = Wire.read();
		i++;
	}
  // check whether the sensor overflow
	if ((data_status_2 & 0b00001000) == 0) {
		*mag_x =  (mag_y_reg - 130) * mag_ssf;// * asay;  // 130 test by hand
		*mag_y = -(mag_x_reg - 243) * mag_ssf;// * asax;  // 243 test by hand
		*mag_z =  mag_z_reg * mag_ssf;// * asaz;
	}
}

// this function is used to calibrate the biases.
void MPU9250::mag_getRaw(uint16_t *mag_x_reg, uint16_t *mag_y_reg, uint16_t *mag_z_reg) {
	uint8_t data_status_1, data_status_2;
  // read register status 1.
	reg_read(device_mag_address, AK8963_reg_st1, 1);
	int i = 0;
	while (Wire.available() && i < 1) {
		data_status_1 = Wire.read();
		i++;
	}
  // read measurement data from AK8963.
	reg_read(device_mag_address, AK8963_reg_hxl, 6);
	uint16_t buff[6];
	int16_t mag_x_reg_buff, mag_y_reg_buff, mag_z_reg_buff;
	i = 0;
	while (Wire.available() && i < 6) buff[i++] = Wire.read();
	mag_x_reg_buff = (buff[0]  | (buff[1] << 8));
	mag_y_reg_buff = (buff[2]  | (buff[3] << 8));
	mag_z_reg_buff = (buff[4]  | (buff[5] << 8));

  // read register status 2.
	reg_read(device_mag_address, AK8963_reg_st2,1);
	i = 0;
	while (Wire.available() && i < 1) {
		data_status_2 = Wire.read();
		i++;
	}
  // check whether the sensor overflow
	if ((data_status_2 & 0b00001000) == 0) {
    *mag_x_reg =  mag_y_reg_buff;//* asay;
    *mag_y_reg = -mag_x_reg_buff;// * asax;
    *mag_z_reg =  mag_z_reg_buff;// * asaz;
}
}

void MPU9250::reg_write(uint8_t address, uint8_t reg, uint8_t value) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

void MPU9250::reg_read(uint8_t address, uint8_t reg, uint8_t num) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, num);
}