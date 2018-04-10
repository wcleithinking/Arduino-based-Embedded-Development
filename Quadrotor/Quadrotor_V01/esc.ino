void esc_thrust() {
  motor1.writeMicroseconds(esc_value_1);
  motor2.writeMicroseconds(esc_value_2);
  motor3.writeMicroseconds(esc_value_3);
  motor4.writeMicroseconds(esc_value_4);
}
