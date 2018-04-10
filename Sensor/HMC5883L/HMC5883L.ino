#include "MPU6050.h"
MPU6050 my_mpu6050;
#include "HMC5883L.h"
HMC5883L my_hmc5883l;

float mag_x, mag_y, mag_z;
float yaw;

void setup() {
  Wire.begin();
  my_mpu6050.acceltempgyro_config(3, 3, 4);
  delay(100);
  my_hmc5883l.mag_config();
  delay(100);
  Serial.begin(115200);
}

void loop() {
  my_hmc5883l.mag_sample(&mag_x, &mag_y, &mag_z);
  yaw = atan2(-mag_y, mag_x);
  Serial.println(yaw * 180 / PI);
  delay(10);
}

