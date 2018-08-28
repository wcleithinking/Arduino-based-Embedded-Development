#ifndef _ADXL345_H_
#define _ADXL345_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address of ADXL345
#define ADXL345_address             0x53  // assume that SDO/ALT address pin is low.

// register mag
#define ADXL345_reg_devid           0x00  // device id
#define ADXL345_reg_thresh_tap      0x1D  // tap threshold
#define ADXL345_reg_ofsx            0x1E  // x-axis offset
#define ADXL345_reg_ofsy            0x1F  // y-axis offset
#define ADXL345_reg_ofsz            0x20  // z-axis offset
#define ADXL345_reg_dur             0x21  // tap duration
#define ADXL345_reg_latent          0x22  // tap latent
#define ADXL345_reg_window          0x23  // tap window
#define ADXL345_reg_thresh_act      0x24  // activity threshold
#define ADXL345_reg_thresh_inact    0x25  // inactivity threshold
#define ADXL345_reg_time_inact      0x26  // inactivity time
#define ADXL345_reg_act_inact_ctl   0x27  // axis enable control for activity and inactivity detection
#define ADXL345_reg_thresh_ff       0x28  // free-fall threshold
#define ADXL345_reg_time_ff         0x29  // free-fall time
#define ADXL345_reg_tap_axes        0x2A  // axis control for single/float tap
#define ADXL345_reg_act_tap_status  0x2B  // source for single/float tap
#define ADXL345_reg_bw_rate         0x2C  // data rate and power mode control
#define ADXL345_reg_power_ctl       0x2D  // power-saving features control
#define ADXL345_reg_int_enable      0x2E  // interrupt enable control
#define ADXL345_reg_int_map         0x2F  // interrupt mapping control
#define ADXL345_reg_int_source      0x30  // source of interrupts
#define ADXL345_reg_data_format     0x31  // data format control
#define ADXL345_reg_datax0          0x32  // x-axis data 0
#define ADXL345_reg_datax1          0x33  // x-axis data 1
#define ADXL345_reg_datay0          0x34  // y-axis data 0
#define ADXL345_reg_datay1          0x35  // y-axis data 1
#define ADXL345_reg_dataz0          0x36  // z-axis data 0
#define ADXL345_reg_dataz1          0x37  // z-axis data 1
#define ADXL345_reg_fifo_ctl        0x38  // fifo control
#define ADXL345_reg_fifo_status     0x39  // fifo status

class ADXL345 {
  public:
    ADXL345();
    void accel_config();
    void accel_sample(float *accel_x, float *accel_y, float *accel_z);
  private:
    int device_address;
};
#endif
