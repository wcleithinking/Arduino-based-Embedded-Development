#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address of HMC5883L
#define HMC5883L_address			0x1E
#define HMC5883L_address_write		0x3C
#define HMC5883L_address_read		0x3D

// register map of HMC5883L
#define HMC5883L_reg_config_a		0x00
#define HMC5883L_reg_config_b		0x01
#define HMC5883L_reg_mode			0x02

#define HMC5883L_reg_mag_xout_h		0x03
#define HMC5883L_reg_mag_xout_l		0x04
#define HMC5883L_reg_mag_zout_h		0x05
#define HMC5883L_reg_mag_zout_l		0x06
#define HMC5883L_reg_mag_yout_h		0x07
#define HMC5883L_reg_mag_yout_l		0x08

#define HMC5883L_reg_status			0x09
#define HMC5883L_reg_identify_a		0x0A
#define HMC5883L_reg_identify_b		0x0B
#define HMC5883L_reg_identify_c		0x0C

class HMC5883L{
public:
	HMC5883L();
	void mag_config();
	void mag_sample(float *mag_x, float *mag_y, float *mag_z);
	void reg_write(uint8_t address, uint8_t reg, uint8_t value);
	void reg_read(uint8_t address, uint8_t reg, uint8_t num);
private:
	uint8_t device_address;
	uint8_t device_address_write;
	uint8_t device_address_read;
};
#endif