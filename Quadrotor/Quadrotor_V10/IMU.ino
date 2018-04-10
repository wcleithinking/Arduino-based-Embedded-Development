void IMU_init() {
  Wire.begin();
  // set up pins
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  // calibrate accel and gyro
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  // user environmental correction in milliGauss, should be automatically calculated
  myIMU.magbias[0] = +470.;
  myIMU.magbias[1] = +120.;
  myIMU.magbias[2] = +125.;
  myIMU.initAK8963(myIMU.magCalibration);
}

void IMU_get() {
  // check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    // ACCEL
    myIMU.readAccelData(myIMU.accelCount);
    myIMU.getAres();
    // calculate the accleration value into actual g's
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes - myIMU.accelBias[2];
    // GYRO
    myIMU.readGyroData(myIMU.gyroCount);
    myIMU.getGres();
    // calculate the gyro value into actual degrees per second
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes - myIMU.gyroBias[0];
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes - myIMU.gyroBias[1];
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes - myIMU.gyroBias[2];
    // MAG
    myIMU.readMagData(myIMU.magCount);
    myIMU.getMres();
    // calculate the magnetometer values in milliGauss
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] - myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] - myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] - myIMU.magbias[2];
  }
  // must be called before updating quaternions!
  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                              *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
  myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                              *(getQ() + 2)));
  myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                              *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
  hatroll = myIMU.roll * RadToDeg;
  hatpitch = -myIMU.pitch * RadToDeg;
  hatyaw = -myIMU.yaw * RadToDeg;
  droll = myIMU.gx;
  dpitch = -myIMU.gy;
  dyaw = -myIMU.gz;
}

void attitude_calibrate() {
  if (loop_index < 100) {
  }
  else if ((loop_index >= 100) && (loop_index < 300)) {
    roll_bias += hatroll;
    pitch_bias += hatpitch;
    yaw_bias += hatyaw;
  }
  else {
    if (loop_index == 300) {
      roll_bias *= 0.005;
      pitch_bias *= 0.005;
      yaw_bias *= 0.005;
      LED_high(); // indicate that complete the attitude calibration
    }
    roll = hatroll - roll_bias;
    pitch = hatpitch - pitch_bias;
    yaw = hatyaw - yaw_bias;
  }
}

