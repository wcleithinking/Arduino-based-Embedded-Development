void ESC() {
  for (int i = 0; i < 4; i++) {
    MOTOR[i] = map(MOTOR[i], pulse_min, pulse_max, 1000, 1800);
  }
  motor0.writeMicroseconds(MOTOR[0]);
  motor1.writeMicroseconds(MOTOR[1]);
  motor2.writeMicroseconds(MOTOR[2]);
  motor3.writeMicroseconds(MOTOR[3]);
}
