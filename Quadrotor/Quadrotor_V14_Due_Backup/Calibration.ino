float DLPF_factor = 0.02;

void Angle_calibrate() {
  if (loop_index < 100) {
  }
  else if ((loop_index >= 100) && (loop_index < 300)) {
    for (int i = 0; i < 3; i++) {
      Angle_bias[i]  += Angle_estimate[i];
    }
  }
  else {
    if (loop_index == 300) {
      for (int i = 0; i < 3; i++) Angle_bias[i] *= 0.005;
      //for (int i = 0; i < 3; i++) Angle_bias[i] = 0;
    }
    for (int i = 0; i < 3; i++) {
      Angle_measure[i]  = Angle_estimate[i] - Angle_bias[i];
      Rate_measure[i]   = Gyro_measure[i] - Gyro_bias_estimate[i];
    }
  }
}
