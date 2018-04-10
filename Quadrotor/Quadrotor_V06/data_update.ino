void data_update() {
  roll_ooo = roll_oo;
  pitch_ooo = pitch_oo;
  yaw_ooo = yaw_oo;
  roll_oo = roll_o;
  pitch_oo = pitch_o;
  yaw_oo = yaw_o;
  roll_o = roll;
  pitch_o = pitch;
  yaw_o = yaw;
  for (int i = 0; i < 4; i++) {
    u_ooo[i] = u_oo[i];
    u_oo[i] = u_o[i];
    u_o[i] = u[i];
  }
}
