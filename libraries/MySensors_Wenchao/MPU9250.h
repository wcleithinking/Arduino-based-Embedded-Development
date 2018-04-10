#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address of MPU9250 and AK8963
#define MPU9250_address                     0x68
#define AK8963_address                      0x0C

// register map for gyroscope and accelerometer
#define MPU9250_reg_self_test_x_gyro        0x00  // gyroscope self-test registers
#define MPU9250_reg_self_test_y_gyro        0x01
#define MPU9250_reg_self_test_z_gyro        0x02

#define MPU9250_reg_self_test_x_accel       0x0D  // accelerometer self-test registers
#define MPU9250_reg_self_test_y_accel       0x0E
#define MPU9250_reg_self_test_z_accel       0x0F

#define MPU9250_reg_xg_offset_h             0x13  // gyro offset registers
#define MPU9250_reg_xg_offset_l             0x14
#define MPU9250_reg_yg_offset_h             0x15
#define MPU9250_reg_yg_offset_l             0x16
#define MPU9250_reg_zg_offset_h             0x17
#define MPU9250_reg_zg_offset_l             0x18

#define MPU9250_reg_smplrt_div              0x19  // sample rate divider
#define MPU9250_reg_config                  0x1A  // all configuration
#define MPU9250_reg_gyro_config             0x1B  // gyroscope configuration
#define MPU9250_reg_accel_config_1          0x1C  // accelerometer configuration 1
#define MPU9250_reg_accel_config_2          0x1D  // accelerometer configuration 2
#define MPU9250_reg_lp_accel_odr            0x1E  // low power accelerometer output data rate(ODR) control 
#define MPU9250_reg_wom_thr                 0x1F  // wake-on motion threshold
#define MPU9250_reg_fifo_en                 0x23  // FIFO enable

#define MPU9250_reg_i2c_mst_ctrl            0x24  // I2C master control
#define MPU9250_reg_i2c_slv0_addr           0x25  // I2C slave 0 control
#define MPU9250_reg_i2c_slv0_reg            0x26
#define MPU9250_reg_i2c_slv0_ctrl           0x27
#define MPU9250_reg_i2c_slv1_addr           0x28  // I2C slave 1 control
#define MPU9250_reg_i2c_slv1_reg            0x29
#define MPU9250_reg_i2c_slv1_ctrl           0x2A
#define MPU9250_reg_i2c_slv2_addr           0x2B  // I2C slave 2 control
#define MPU9250_reg_i2c_slv2_reg            0x2C
#define MPU9250_reg_i2c_slv2_ctrl           0x2D
#define MPU9250_reg_i2c_slv3_addr           0x2E  // I2C slave 3 control
#define MPU9250_reg_i2c_slv3_reg            0x2F
#define MPU9250_reg_i2c_slv3_ctrl           0x30
#define MPU9250_reg_i2c_slv4_addr           0x31  // I2C slave 4 control
#define MPU9250_reg_i2c_slv4_reg            0x32
#define MPU9250_reg_i2c_slv4_do             0x33
#define MPU9250_reg_i2c_slv4_ctrl           0x34
#define MPU9250_reg_i2c_slv4_di             0x35
#define MPU9250_reg_i2c_mst_status          0x36  // I2C master status

#define MPU9250_reg_int_pin_cfg             0x37  // INT pin/bypass enable configuration
#define MPU9250_reg_int_enable              0x38  // interrupt enable
#define MPU9250_reg_int_status              0x3A  // interrupt status

#define MPU9250_reg_accel_xout_h            0x3B  // accelerometer measurements
#define MPU9250_reg_accel_xout_l            0x3C
#define MPU9250_reg_accel_yout_h            0x3D
#define MPU9250_reg_accel_yout_l            0x3E
#define MPU9250_reg_accel_zout_h            0x3F
#define MPU9250_reg_accel_zout_l            0x40

#define MPU9250_reg_temp_out_h              0x41  // temperature measurements
#define MPU9250_reg_temp_out_l              0x42

#define MPU9250_reg_gyro_xout_h             0x43  // gyroscope measurements
#define MPU9250_reg_gyro_xout_l             0x44
#define MPU9250_reg_gyro_yout_h             0x45
#define MPU9250_reg_gyro_yout_l             0x46
#define MPU9250_reg_gyro_zout_h             0x47
#define MPU9250_reg_gyro_zout_l             0x48

#define MPU9250_reg_ext_sens_data_00        0x49  // external sensor data
#define MPU9250_reg_ext_sens_data_01        0x4A
#define MPU9250_reg_ext_sens_data_02        0x4B
#define MPU9250_reg_ext_sens_data_03        0x4C
#define MPU9250_reg_ext_sens_data_04        0x4D
#define MPU9250_reg_ext_sens_data_05        0x4E
#define MPU9250_reg_ext_sens_data_06        0x4F
#define MPU9250_reg_ext_sens_data_07        0x50
#define MPU9250_reg_ext_sens_data_08        0x51
#define MPU9250_reg_ext_sens_data_09        0x52
#define MPU9250_reg_ext_sens_data_10        0x53
#define MPU9250_reg_ext_sens_data_11        0x54
#define MPU9250_reg_ext_sens_data_12        0x55
#define MPU9250_reg_ext_sens_data_13        0x56
#define MPU9250_reg_ext_sens_data_14        0x57
#define MPU9250_reg_ext_sens_data_15        0x58
#define MPU9250_reg_ext_sens_data_16        0x59
#define MPU9250_reg_ext_sens_data_17        0x5A
#define MPU9250_reg_ext_sens_data_18        0x5B
#define MPU9250_reg_ext_sens_data_19        0x5C
#define MPU9250_reg_ext_sens_data_20        0x5D
#define MPU9250_reg_ext_sens_data_21        0x5E
#define MPU9250_reg_ext_sens_data_22        0x5F
#define MPU9250_reg_ext_sens_data_23        0x60

#define MPU9250_reg_i2c_slv0_do             0x63  // I2C slave 0 data out
#define MPU9250_reg_i2c_slv1_do             0x64  // I2C slave 1 data out
#define MPU9250_reg_i2c_slv2_do             0x65  // I2C slave 2 data out
#define MPU9250_reg_i2c_slv3_do             0x66  // I2C slave 3 data out
#define MPU9250_reg_i2c_mst_delay_ctrl      0x67  // I2C master delay control

#define MPU9250_reg_signal_path_reset       0x68  // signal path reset
#define MPU9250_reg_mot_detect_ctrl         0x69  // accelerometer interrupt control
#define MPU9250_reg_user_ctrl               0x6A  // user control
#define MPU9250_reg_pwr_mgmt_1              0x6B  // power management 1
#define MPU9250_reg_pwr_mgmt_2              0x6C  // power management 2

#define MPU9250_reg_fifo_count_h            0x72  // FIFO count registers
#define MPU9250_reg_fifo_count_l            0x73
#define MPU9250_reg_fifo_r_w                0x74  // FIFO read write

#define MPU9250_reg_who_am_i                0x75  // who am i

#define MPU9250_reg_xa_offset_h             0x77  // accelerometer offset registers
#define MPU9250_reg_xa_offset_l             0x78
#define MPU9250_reg_ya_offset_h             0x7A
#define MPU9250_reg_ya_offset_l             0x7B
#define MPU9250_reg_za_offset_h             0x7D
#define MPU9250_reg_za_offset_l             0x7E

// register map for magnetometer: AK8963
#define AK8963_reg_wia                      0x00  // who am i
#define AK8963_reg_info                     0x01  // information
#define AK8963_reg_st1                      0x02  // status 1

#define AK8963_reg_hxl                      0x03  // measurement data
#define AK8963_reg_hxh                      0x04
#define AK8963_reg_hyl                      0x05
#define AK8963_reg_hyh                      0x06
#define AK8963_reg_hzl                      0x07
#define AK8963_reg_hzh                      0x08

#define AK8963_reg_st2                      0x09  // status 2
#define AK8963_reg_cntl                     0x0A  // control 1
#define AK8963_reg_cnt2                     0x0B  // control 2
#define AK8963_reg_astc                     0x0C  // self-test
#define AK8963_reg_ts1                      0x0D  // test 1
#define AK8963_reg_ts2                      0x0E  // test 2
#define AK8963_reg_i2cdis                   0x0F  // I2C disable

#define AK8963_reg_asax                     0x10  // x-axis sensitivity adjustment value
#define AK8963_reg_asay                     0x11  // y-axis sensitivity adjustment value
#define AK8963_reg_asaz                     0x12  // z-axis sensitivity adjustment value

class MPU9250 {
  public:
      MPU9250();
      void all_config(uint8_t AFS_SEL, uint8_t ADLPF_CFG, uint8_t GFS_SEL, uint8_t GDLPF_CFG, uint8_t MSS_SEL);
      void acceltempgyro_config(uint8_t AFS_SEL, uint8_t ADLPF_CFG, uint8_t GFS_SEL, uint8_t GDLPF_CFG);
      void mag_config(uint8_t MSS_SEL);
      void accel_sample(float *accel_x, float *accel_y, float *accel_z);
      void temp_sample(float *temp);
      void gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z);
      void mag_sample(float *mag_x, float *mag_y, float *mag_z);
      void mag_getRaw(uint16_t *mag_x_reg, uint16_t *mag_y_reg, uint16_t *mag_z_reg);
      void reg_write(uint8_t address, uint8_t reg, uint8_t value);
      void reg_read(uint8_t address, uint8_t reg, uint8_t num);
  private:
      int device_address;
      int device_mag_address;
      float accel_ssf;
      float gyro_ssf;
      float mag_ssf;
      float asax, asay, asaz;
};

#endif
