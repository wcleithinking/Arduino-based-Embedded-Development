void ESC() {
  for (int i = 0; i < 4; i++) MOTOR[i] = map(MOTOR[i], pulse_min, pulse_max, 127, (int)(127 + 0.128 * (pulse_max - 1016)));
  analogWrite(motor0Pin, MOTOR[0]);
  analogWrite(motor1Pin, MOTOR[1]);
  analogWrite(motor2Pin, MOTOR[2]);
  analogWrite(motor2Pin, MOTOR[3]);
}
