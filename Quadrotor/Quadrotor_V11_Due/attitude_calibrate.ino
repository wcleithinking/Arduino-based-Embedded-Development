void attitude_calibrate() {
  if (loop_index < 100) {
  }
  else if ((loop_index >= 100) && (loop_index < 300)) {
    roll_bias  += hatroll;
    pitch_bias += hatpitch;
    yaw_bias   += hatyaw;
  }
  else {
    if (loop_index == 300) {
      roll_bias  *= 0.005;
      pitch_bias *= 0.005;
      yaw_bias   *= 0.005;
      led_high(); // indicate that complete the attitude calibration
    }
    roll  = (hatroll  - roll_bias);
    pitch = (hatpitch - pitch_bias);
    yaw = (hatyaw - yaw_bias);
  }
}

