#ifndef _MS5611_H_
#define _MS5611_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address of MS5611
#define MS5611_address              0x77

// I2C command (1 byte) of MS5611
#define MS5611_cmd_reset            0x1E    // Reset

#define MS5611_cmd_convert_d1_256   0x40    // D1 conversion
#define MS5611_cmd_convert_d1_512   0x42
#define MS5611_cmd_convert_d1_1024  0x44
#define MS5611_cmd_convert_d1_2048  0x46
#define MS5611_cmd_convert_d1_4096  0x48

#define MS5611_cmd_convert_d2_256   0x50    // D2 conversion
#define MS5611_cmd_convert_d2_512   0x52
#define MS5611_cmd_convert_d2_1024  0x54
#define MS5611_cmd_convert_d2_2048  0x56
#define MS5611_cmd_convert_d2_4096  0x58

#define MS5611_cmd_adc_read         0x00    // Read adc result (24 bit pressure / temperature)

#define MS5611_cmd_prom_read        0xA0    // read PROM (128 bit of calibration words) 
                                            // 0b 1|0|1|0|ad2|ad1|ad0|0| 
                                            // 0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC, 0xAE

class MS5611 {
public:
    MS5611();
    void baro_reset();
    void prom_read(uint16_t C[8]);
    void adc_D1_convert(uint8_t OSR);
    void adc_D2_convert(uint8_t OSR);
    void adc_read(uint32_t *D);
    void cmd_send(uint8_t cmd);
private:
    uint8_t device_address;
};

#endif
