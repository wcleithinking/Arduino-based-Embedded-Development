#include "HMC5883L.h"

HMC5883L::HMC5883L() {
	device_address = HMC5883L_address;
}

void HMC5883L::mag_config() {
	// set the device in continuous  measurment mode
	reg_write(device_address, HMC5883L_reg_mode, 0b00000000);
}

void HMC5883L::mag_sample(float *mag_x, float *mag_y, float *mag_z) {
	reg_read(device_address, HMC5883L_reg_mag_xout_h, 6);
	uint16_t buff[6];
	int16_t mag_x_reg, mag_y_reg, mag_z_reg;
	int i = 0;
	while (Wire.available() && i < 6) buff[i++] = Wire.read();
	mag_x_reg = (buff[1] | (buff[0] << 8));
	mag_z_reg = (buff[3] | (buff[2] << 8));
	mag_y_reg = (buff[5] | (buff[4] << 8));
	*mag_x =  (mag_x_reg -  55) * 100.0/1090;
	*mag_y = -(mag_y_reg + 225) * 100.0/1090;
	*mag_z = -(mag_z_reg)       * 100.0/1090;
}

void HMC5883L::reg_write(uint8_t address, uint8_t reg, uint8_t value) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

void HMC5883L::reg_read(uint8_t address, uint8_t reg, uint8_t num) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, num);
}