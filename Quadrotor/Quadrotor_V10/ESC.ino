void ESC_init() {
  motor0.attach(motor0Pin);
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor0.writeMicroseconds(PWM_MIN);
  motor1.writeMicroseconds(PWM_MIN);
  motor2.writeMicroseconds(PWM_MIN);
  motor3.writeMicroseconds(PWM_MIN);
}
void motor_update() {
  motor0.writeMicroseconds(PWM_out[0]);
  motor1.writeMicroseconds(PWM_out[1]);
  motor2.writeMicroseconds(PWM_out[2]);
  motor3.writeMicroseconds(PWM_out[3]);
}
