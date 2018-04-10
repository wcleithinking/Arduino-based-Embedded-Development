void ESC() {
  for (int i = 0; i < 4; i++) {
    MOTOR[i] = map(MOTOR[i], pulse_min, pulse_max, 127, 230);
  }
  analogWrite( 3, MOTOR[0]);
  analogWrite( 9, MOTOR[1]);
  analogWrite(10, MOTOR[2]);
  analogWrite(11, MOTOR[3]);
}
