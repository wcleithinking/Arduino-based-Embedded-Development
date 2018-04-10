#if defined(SONAR)
void altitude_calibrate() {
  altitude_bias = 0;
  altitude = hataltitude - altitude_bias;
}

#elif defined(BARO)
void altitude_calibrate() {
  if (loop_index < 400) {
  }
  else if ((loop_index >= 400) && (loop_index < 600)) {
    altitude_bias += hataltitude;
  }
  else {
    if (loop_index == 600) {
      altitude_bias = altitude_bias * 0.005;
    }
    altitude = hataltitude - altitude_bias;
  }
}
#endif
