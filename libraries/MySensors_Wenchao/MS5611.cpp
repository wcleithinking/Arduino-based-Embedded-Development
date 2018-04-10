#include "MS5611.h"

MS5611::MS5611() {
	device_address = MS5611_address;
}

void MS5611::baro_reset() {
	cmd_send(MS5611_cmd_reset);
}

void MS5611::prom_read(uint16_t C[8]) {
	uint16_t buff[2];
	for (int i = 0; i < 8; i++) {
		cmd_send(MS5611_cmd_prom_read + 2 * i);
		uint8_t j = 0;
		Wire.requestFrom(device_address, (uint8_t)2);
		while (Wire.available() && j < 2) buff[j++] = Wire.read();
		C[i] = (buff[1] | (buff[0] << 8));
	}
}

void MS5611::adc_D1_convert(uint8_t OSR) {
	switch (OSR) {
		case 0:
		cmd_send(MS5611_cmd_convert_d1_256);
		break;
		case 1:
		cmd_send(MS5611_cmd_convert_d1_512);
		break;
		case 2:
		cmd_send(MS5611_cmd_convert_d1_1024);
		break;
		case 3:
		cmd_send(MS5611_cmd_convert_d1_2048);
		break;
		case 4:
		cmd_send(MS5611_cmd_convert_d1_4096);
		break;
		default:
		cmd_send(MS5611_cmd_convert_d1_4096);
		break;
	}
}

void MS5611::adc_D2_convert(uint8_t OSR) {
	switch (OSR) {
		case 0:
		cmd_send(MS5611_cmd_convert_d2_256);
		break;
		case 1:
		cmd_send(MS5611_cmd_convert_d2_512);
		break;
		case 2:
		cmd_send(MS5611_cmd_convert_d2_1024);
		break;
		case 3:
		cmd_send(MS5611_cmd_convert_d2_2048);
		break;
		case 4:
		cmd_send(MS5611_cmd_convert_d2_4096);
		break;
		default:
		cmd_send(MS5611_cmd_convert_d2_4096);
		break;
	}
}

void MS5611::adc_read(uint32_t *D) {
	uint32_t buff[3];
	cmd_send(MS5611_cmd_adc_read);
	uint8_t j = 0;
	Wire.requestFrom(device_address, (uint8_t)3);
	while (Wire.available() && j < 3) buff[j++] = Wire.read();
	*D = (buff[2] | (buff[1] << 8) | (buff[0] << 16));
}

void MS5611::cmd_send(uint8_t cmd) {
	Wire.beginTransmission(device_address);
	Wire.write(cmd);
	Wire.endTransmission();
}