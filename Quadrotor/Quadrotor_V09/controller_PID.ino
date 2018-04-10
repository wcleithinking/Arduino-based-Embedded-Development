void PID_controller_roll() {
#if defined(PID_v1)
  roll_sum  += (roll - roll_d) * dt;
  roll_feedback  = (- kp * roll - ki * roll_sum - kd * (gyro_x - gyro_x_bias)) * RadToDeg;
#elif defined(PID_v2)
  e[0] = roll_d - roll;
  P[0] = K[0] * e[0];
  D[0] = K[0] * (exp(-dt * N[0] / Td[0]) * Dold[0] + N[0] * (e[0] - eold[0]));
  PID[0] = P[0] + I[0] + D[0];
  I[0] = K[0] * (I[0] + (dt / Ti[0]) * e[0]);
  roll_feedback  = PID[0];
#elif defined(PID_v3)
  float droll_ref = - kp * roll - ki * roll_sum - kd * (gyro_x);
  droll_ref = constrain(droll_ref, -umax / 2, umax / 2);
  droll_sum += (gyro_x * RadToDeg - droll_ref) * dt;
  roll_feedback = -kp_d * (gyro_x * RadToDeg - droll_ref) - ki_d * droll_sum;
#endif
  roll_feedback  = constrain(roll_feedback, -umax, umax);
}

void PID_controller_pitch() {
#if defined(PID_v1)
  pitch_sum += (pitch - pitch_d) * dt;
  pitch_feedback = (- kp * pitch - ki * pitch_sum - kd * (gyro_y - gyro_y_bias)) * RadToDeg;
#elif defined(PID_v2)
  e[1] = pitch_d - pitch;
  P[1] = K[1] * e[1];
  D[1] = K[1] * (exp(-dt * N[1] / Td[1]) * Dold[1] + N[1] * (e[1] - eold[1]));
  PID[1] = P[1] + I[1] + D[1];
  I[1] = K[1] * (I[1] + (dt / Ti[1]) * e[1]);
  pitch_feedback  = PID[1];
#elif defined(PID_v3)
  float dpitch_ref = - kp * pitch - ki * pitch_sum - kd * gyro_y;
  dpitch_ref = constrain(dpitch_ref, -umax / 2, umax / 2);
  dpitch_sum += (gyro_y * RadToDeg - dpitch_ref) * dt;
  pitch_feedback = -kp_d * (gyro_y * RadToDeg - dpitch_ref) - ki_d * dpitch_sum;
#endif
  pitch_feedback = constrain(pitch_feedback, -umax, umax);
}

void PID_controller_yaw() {
#ifdef PID_v1
  yaw_sum   += (yaw - yaw_d) * dt;
  yaw_feedback = (- kp * yaw - ki * yaw_sum - kd * gyro_z) * RadToDeg;
#elif defined(PID_v2)
  e[2] = yaw_d - yaw;
  P[2] = K[2] * e[2];
  D[2] = K[2] * (exp(-dt * N[2] / Td[2]) * Dold[2] + N[2] * (e[2] - eold[2]));
  PID[2] = P[2] + I[2] + D[2];
  I[2] = K[2] * (I[2] + (dt / Ti[2]) * e[2]);
  yaw_feedback  = PID[2];
#elif defined(PID_v3)
  float dyaw_ref = - kp * yaw - ki * yaw_sum - kd * gyro_z;
  dyaw_sum += (gyro_z * RadToDeg - dyaw_ref) * dt;
  yaw_feedback = -kp_d * (gyro_z * RadToDeg - dyaw_ref) - ki_d * dyaw_sum;
#endif
  yaw_feedback = constrain( yaw_feedback, -2 * umax, 2 * umax);
}


void PID_updatedata() {
#ifdef PID_v2
  for (int i = 0; i < 3; i++) {
    eold[i] = e[i];
    Dold[i] = D[i];
  }
#endif
}

