void ESC() {
  motor1.writeMicroseconds(PWM[0]);
  motor2.writeMicroseconds(PWM[1]);
  motor3.writeMicroseconds(PWM[2]);
  motor4.writeMicroseconds(PWM[3]);
}
