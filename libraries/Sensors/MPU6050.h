#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address of MPU6050
#define MPU6050_address						0x68

// register map of MPU6050
#define MPU6050_reg_self_test_x				0x0D	// self-test registers
#define MPU6050_reg_self_test_y				0x0E
#define MPU6050_reg_self_test_z				0x0F
#define MPU6050_reg_self_test_a				0x10

#define MPU6050_reg_smprt_div				0x19	// sample rate divider
#define MPU6050_reg_config                  0x1A  // all configuration
#define MPU6050_reg_gyro_config             0x1B  // gyroscope configuration
#define MPU6050_reg_accel_config          	0x1C  // accelerometer configuration
#define MPU6050_reg_fifo_en                 0x23  // FIFO enable

#define MPU6050_reg_i2c_mst_ctrl            0x24  // I2C master control
#define MPU6050_reg_i2c_slv0_addr           0x25  // I2C slave 0 control
#define MPU6050_reg_i2c_slv0_reg            0x26
#define MPU6050_reg_i2c_slv0_ctrl           0x27
#define MPU6050_reg_i2c_slv1_addr           0x28  // I2C slave 1 control
#define MPU6050_reg_i2c_slv1_reg            0x29
#define MPU6050_reg_i2c_slv1_ctrl           0x2A
#define MPU6050_reg_i2c_slv2_addr           0x2B  // I2C slave 2 control
#define MPU6050_reg_i2c_slv2_reg            0x2C
#define MPU6050_reg_i2c_slv2_ctrl           0x2D
#define MPU6050_reg_i2c_slv3_addr           0x2E  // I2C slave 3 control
#define MPU6050_reg_i2c_slv3_reg            0x2F
#define MPU6050_reg_i2c_slv3_ctrl           0x30
#define MPU6050_reg_i2c_slv4_addr           0x31  // I2C slave 4 control
#define MPU6050_reg_i2c_slv4_reg            0x32
#define MPU6050_reg_i2c_slv4_do             0x33
#define MPU6050_reg_i2c_slv4_ctrl           0x34
#define MPU6050_reg_i2c_slv4_di             0x35
#define MPU6050_reg_i2c_mst_status          0x36  // I2C master status

#define MPU6050_reg_int_pin_cfg             0x37  // INT pin/bypass enable configuration
#define MPU6050_reg_int_enable              0x38  // interrupt enable
#define MPU6050_reg_int_status              0x3A  // interrupt status

#define MPU6050_reg_accel_xout_h            0x3B  // accelerometer measurements
#define MPU6050_reg_accel_xout_l            0x3C
#define MPU6050_reg_accel_yout_h            0x3D
#define MPU6050_reg_accel_yout_l            0x3E
#define MPU6050_reg_accel_zout_h            0x3F
#define MPU6050_reg_accel_zout_l            0x40

#define MPU6050_reg_temp_out_h              0x41  // temperature measurements
#define MPU6050_reg_temp_out_l              0x42

#define MPU6050_reg_gyro_xout_h             0x43  // gyroscope measurements
#define MPU6050_reg_gyro_xout_l             0x44
#define MPU6050_reg_gyro_yout_h             0x45
#define MPU6050_reg_gyro_yout_l             0x46
#define MPU6050_reg_gyro_zout_h             0x47
#define MPU6050_reg_gyro_zout_l             0x48

#define MPU6050_reg_ext_sens_data_00        0x49  // external sensor data
#define MPU6050_reg_ext_sens_data_01        0x4A
#define MPU6050_reg_ext_sens_data_02        0x4B
#define MPU6050_reg_ext_sens_data_03        0x4C
#define MPU6050_reg_ext_sens_data_04        0x4D
#define MPU6050_reg_ext_sens_data_05        0x4E
#define MPU6050_reg_ext_sens_data_06        0x4F
#define MPU6050_reg_ext_sens_data_07        0x50
#define MPU6050_reg_ext_sens_data_08        0x51
#define MPU6050_reg_ext_sens_data_09        0x52
#define MPU6050_reg_ext_sens_data_10        0x53
#define MPU6050_reg_ext_sens_data_11        0x54
#define MPU6050_reg_ext_sens_data_12        0x55
#define MPU6050_reg_ext_sens_data_13        0x56
#define MPU6050_reg_ext_sens_data_14        0x57
#define MPU6050_reg_ext_sens_data_15        0x58
#define MPU6050_reg_ext_sens_data_16        0x59
#define MPU6050_reg_ext_sens_data_17        0x5A
#define MPU6050_reg_ext_sens_data_18        0x5B
#define MPU6050_reg_ext_sens_data_19        0x5C
#define MPU6050_reg_ext_sens_data_20        0x5D
#define MPU6050_reg_ext_sens_data_21        0x5E
#define MPU6050_reg_ext_sens_data_22        0x5F
#define MPU6050_reg_ext_sens_data_23        0x60

#define MPU6050_reg_i2c_slv0_do             0x63  // I2C slave 0 data out
#define MPU6050_reg_i2c_slv1_do             0x64  // I2C slave 1 data out
#define MPU6050_reg_i2c_slv2_do             0x65  // I2C slave 2 data out
#define MPU6050_reg_i2c_slv3_do             0x66  // I2C slave 3 data out
#define MPU6050_reg_i2c_mst_delay_ctrl      0x67  // I2C master delay control

#define MPU6050_reg_signal_path_reset       0x68  // signal path reset
#define MPU6050_reg_user_ctrl               0x6A  // user control
#define MPU6050_reg_pwr_mgmt_1              0x6B  // power management 1
#define MPU6050_reg_pwr_mgmt_2              0x6C  // power management 2

#define MPU6050_reg_fifo_count_h            0x72  // FIFO count registers
#define MPU6050_reg_fifo_count_l            0x73
#define MPU6050_reg_fifo_r_w                0x74  // FIFO read write

#define MPU6050_reg_who_am_i                0x75  // who am i

class MPU6050{
public:
	MPU6050();
	void acceltempgyro_config(uint8_t AFS_SEL, uint8_t GFS_SEL, uint8_t DLPF_CFG);
	void accel_sample(float *accel_x, float *accel_y, float *accel_z);
	void temp_sample(float *temp);
	void gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z);
	void reg_write(uint8_t address, uint8_t reg, uint8_t value);
	void reg_read(uint8_t address, uint8_t reg, uint8_t num);
private:
	uint8_t device_address;
	float accel_ssf;
	float gyro_ssf;
};

#endif
