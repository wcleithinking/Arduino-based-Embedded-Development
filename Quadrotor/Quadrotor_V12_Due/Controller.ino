// Feed Control of Vehicle, from Controllers
float FeedForward[4]  = {0, 0, 0, PWM_MIN};
float FeedBack[4]     = {0, 0, 0, 0};
float FeedAll[4]      = {0, 0, 0, PWM_MIN};
float FeedBackMax[4]  = {50, 50, 100, PWM_MAX};

void Controller() {
#ifdef FEEDFORWARD
  // feedforward
  Angle_desire[IndexRoll]  = constrain(map(RC[IndexRoll], Pulse_MIN, Pulse_MAX, Roll_MIN, Roll_MAX), Roll_MIN, Roll_MAX);
  Angle_desire[IndexPitch] = constrain(map(RC[IndexPitch], Pulse_MIN, Pulse_MAX, Pitch_MIN, Pitch_MAX), Pitch_MIN, Pitch_MAX);
  // Angle_desire[IndexYaw]   = constrain(map(RC[IndexYaw], Pulse_MIN, Pulse_MAX, Yaw_MIN, Yaw_MAX), Yaw_MIN, Yaw_MAX);
  for (int i = 0; i < 3; i++) {
    if ((1490 <= RC[i]) && (RC[i] <= 1510)) {
      Angle_desire[i] = 0;
    }
  }
#endif
  // feedforward
  FeedForward[IndexRoll]      = 0;
  FeedForward[IndexPitch]     = 0;
  FeedForward[IndexYaw]       = 0;
  FeedForward[IndexAltitude]  = constrain(map(RC[IndexAltitude], Pulse_MIN, Pulse_MAX, PWM_MIN, PWM_MAX), PWM_MIN, PWM_MAX);
  // feedback
#ifdef STR_v1
  switch (STR_Index) {
    case 0:
      time_STR_current = millis();
      if (time_STR_current >= time_STR_previous + STR_period) {
#ifdef DEBUG
        time_STR_diff = time_STR_current - time_STR_previous;
        time_STR_previous = time_STR_current;
        time_STR_start = millis();
        FeedBack[IndexRoll]  = STR_controller(IndexRoll);
        time_STR_end = millis();
#else
        time_STR_previous = time_STR_current;
        FeedBack[IndexRoll]  = STR_controller(IndexRoll);
#endif
      }
      FeedBack[IndexPitch]  = PID_controller(IndexPitch);
      FeedBack[IndexYaw]   = PID_controller(IndexYaw);
      break;
    case 1:
      time_STR_current = millis();
      if (time_STR_current >= time_STR_previous + STR_period) {
#ifdef DEBUG
        time_STR_diff = time_STR_current - time_STR_previous;
        time_STR_previous = time_STR_current;
        time_STR_start = millis();
        FeedBack[IndexPitch]  = STR_controller(IndexPitch);
        time_STR_end = millis();
#else
        time_STR_previous = time_STR_current;
        FeedBack[IndexPitch]  = STR_controller(IndexPitch);
#endif
      }
      FeedBack[IndexRoll]  = PID_controller(IndexRoll);
      FeedBack[IndexYaw]   = PID_controller(IndexYaw);
      break;
    case 2:
      time_STR_current = millis();
      if (time_STR_current >= time_STR_previous + STR_period) {
#ifdef DEBUG
        time_STR_diff = time_STR_current - time_STR_previous;
        time_STR_previous = time_STR_current;
        time_STR_start = millis();
        FeedBack[IndexYaw]  = STR_controller(IndexYaw);
        time_STR_end = millis();
#else
        time_STR_previous = time_STR_current;
        FeedBack[IndexYaw]  = STR_controller(IndexYaw);
#endif
      }
      FeedBack[IndexRoll]   = PID_controller(IndexRoll);
      FeedBack[IndexPitch]  = PID_controller(IndexPitch);
      break;
  }
#else
  FeedBack[IndexRoll]   = PID_controller(IndexRoll);
  FeedBack[IndexPitch]  = PID_controller(IndexPitch);
  FeedBack[IndexYaw]   = PID_controller(IndexYaw);
#endif
  // all = feedforward + feedback
  for (int i = 0; i < 4; i++) FeedAll[i] = FeedForward[i] + FeedBack[i];
#if defined(QuadP)
  PWM_ref[0] =  FeedAll[IndexPitch] - FeedAll[IndexYaw] + FeedAll[IndexAltitude];
  PWM_ref[1] = -FeedAll[IndexRoll]  + FeedAll[IndexYaw] + FeedAll[IndexAltitude];
  PWM_ref[2] = -FeedAll[IndexPitch] - FeedAll[IndexYaw] + FeedAll[IndexAltitude];
  PWM_ref[3] =  FeedAll[IndexRoll]  + FeedAll[IndexYaw] + FeedAll[IndexAltitude];
#elif defined(QuadX)
  PWM_ref[0] =  FeedAll[IndexRoll] + FeedAll[IndexPitch] + FeedAll[IndexYaw] + FeedAll[IndexAltitude];
  PWM_ref[1] = -FeedAll[IndexRoll] + FeedAll[IndexPitch] - FeedAll[IndexYaw] + FeedAll[IndexAltitude];
  PWM_ref[2] = -FeedAll[IndexRoll] - FeedAll[IndexPitch] + FeedAll[IndexYaw] + FeedAll[IndexAltitude];
  PWM_ref[3] =  FeedAll[IndexRoll] - FeedAll[IndexPitch] - FeedAll[IndexYaw] + FeedAll[IndexAltitude];
#endif
#ifdef DEBUG
  for (int i = 0; i < 4; i++) PWM_out[i] = PWM_MIN;
#else
  for (int i = 0; i < 4; i++) PWM_out[i] = constrain(PWM_ref[i], PWM_MIN, PWM_MAX);
#endif
}

/*
  PID Variables
*/
#if defined(PID_v1)
// outer loop
float Kp_outer = 1.8, Ki_outer = 0.15, Kd_outer = 4.8;
float eTerm_outer[3] = {0, 0, 0};
float PTerm_outer[3] = {0, 0, 0};
float ITerm_outer[3] = {0, 0, 0};
float DTerm_outer[3] = {0, 0, 0};
// inner loop
float eTerm_inner[3] = {0, 0, 0};
#elif defined(PID_v2)
// outer loop
float Kp_outer = 1.8, Ki_outer = 0.08, Kd_outer = 4.8;
float eTerm_outer[3] = {0, 0, 0};
float PTerm_outer[3] = {0, 0, 0};
float ITerm_outer[3] = {0, 0, 0};
float DTerm_outer[3] = {0, 0, 0};
// inner loop
float Kp_inner = 1.8, Ki_inner = 0.02, Kd_inner = 4.8;
float eTerm_inner[3] = {0, 0, 0};
float PTerm_inner[3] = {0, 0, 0};
float ITerm_inner[3] = {0, 0, 0};
float DTerm_inner[3] = {0, 0, 0};
float eTerm_inner_old[3] = {0, 0, 0};
// innerinner loop
float eTerm_innerinner[3] = {0, 0, 0};
#elif defined(PID_v3)
float K[3]  = {2, 2, 2};
float Ti[3] = {36.3, 36.3, 36.3};
float Td[3] = {9.075, 9.075, 9.075};
float N[3]  = {8, 8, 8};
float eTerm[3]  = {0, 0, 0};
float PTerm[3] = {0, 0, 0};
float ITerm[3] = {0, 0, 0};
float DTerm[3] = {0, 0, 0};
float eTerm_old[3] = {0, 0, 0};
float DTerm_old[3] = {0, 0, 0};
#endif

float PID_controller(uint8_t IndexAngle) {
  uint8_t Index = IndexAngle;
  float PID_out;
#if defined(PID_v1)
  eTerm_outer[Index]  = Angle_desire[Index] - Angle_measure[Index] * RadToDeg;
  eTerm_inner[Index]  = Rate_desire[Index]  - Rate_measure[Index]  * RadToDeg;
  PTerm_outer[Index]  = Kp_outer * eTerm_outer[Index];
  ITerm_outer[Index] += Ki_outer * eTerm_outer[Index] * dt;
  DTerm_outer[Index]  = Kd_outer * eTerm_inner[Index];
  PID_out = PTerm_outer[Index] + ITerm_outer[Index] + DTerm_outer[Index];
#elif defined(PID_v2)
  eTerm_outer[Index]  = Angle_desire[Index] - Angle_measure[Index] * RadToDeg;
  eTerm_inner[Index]  = Rate_desire[Index]  - Rate_measure[Index]  * RadToDeg;
  PTerm_outer[Index]  = Kp_outer * eTerm_outer[Index];
  ITerm_outer[Index] += Ki_outer * eTerm_outer[Index] * dt;
  DTerm_outer[Index]  = Kd_outer * eTerm_inner[Index];
  Rate_desire[Index]  = PTerm_outer[Index] + ITerm_outer[Index] + DTerm_outer[Index];
  Rate_desire[Index]  = constrain(Rate_desire[Index], -FeedBackMax[Index], FeedBackMax[Index]);
  eTerm_inner[Index]  = Rate_desire[Index]  - Rate_measure[Index]  * RadToDeg;
  eTerm_innerinner[Index] = (eTerm_inner[Index] - eTerm_inner_old[Index]) / dt;
  PTerm_inner[Index]  = Kp_inner * eTerm_inner[Index];
  ITerm_inner[Index] += Ki_inner * eTerm_inner[Index] * dt;
  DTerm_inner[Index]  = Kd_inner * eTerm_innerinner[Index];
  PID_out = PTerm_inner[Index] + ITerm_inner[Index] + DTerm_inner[Index];
#elif defined(PID_v3)
  eTerm[Index]  = Angle_desire[Index] - Angle_measure[Index] * RadToDeg;
  PTerm[Index]  = K[Index] * eTerm[Index];
  DTerm[Index]  = K[Index] * (exp(-dt * N[Index] / Td[Index]) * DTerm_old[Index] + N[Index] * (eTerm[Index] - eTerm_old[Index]));
  PID_out       = PTerm[Index] + ITerm[Index] + DTerm[Index];
  ITerm[Index]  = K[Index] * (ITerm[Index] + (dt / Ti[Index]) * eTerm[Index]);
#endif
  PID_out = constrain(PID_out, -FeedBackMax[Index], FeedBackMax[Index]);
  return PID_out;
}

#ifdef STR_v1
/*
   STR Variables
*/
// Roll:  STR_flag = IndexRoll  + 1 = 1,
// Pitch: STR_flag = IndexPitch + 1 = 2,
// Yaw:   STR_flag = IndexYaw   + 1 = 3
uint8_t STR_flag = 0;
// Inputs and Outputs
float y_desire[3][4] = {0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0
                       };
float y_measure[3][4] = {0, 0, 0, 0,
                         0, 0, 0, 0,
                         0, 0, 0, 0
                        };
float u_compute[3][4] = {0, 0, 0, 0,
                         0, 0, 0, 0,
                         0, 0, 0, 0
                        };
float u_actual[3][4] = {0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0
                       };
// Parameters
float am1[3] = { -1.4431986644, -1.4431986644, -1.92701};
float am2[3] = {  0.7708989340,  0.7708989340,  1.29750};
float am3[3] = { -0.1380692400, -0.1380692400, -0.30120};
float ao1[3] = {0.8, 0.8, 0.8}, ao2[3] = {0.16, 0.16, 0.16};
// Estimators
float lambda[3] = {0.98, 0.98, 0.98};
float hattheta[3][6] = {a1[IndexRoll],  a2[IndexRoll],  a3[IndexRoll],  b0[IndexRoll],  b1[IndexRoll],  b2[IndexRoll],
                        a1[IndexPitch], a2[IndexPitch], a3[IndexPitch], b0[IndexPitch], b1[IndexPitch], b2[IndexPitch],
                        a1[IndexYaw],   a2[IndexYaw],   a3[IndexYaw],   b0[IndexYaw],   b1[IndexYaw],   b2[IndexYaw]
                       };
float P[3][6][6] = {{
    100, 0, 0, 0, 0, 0,
    0, 100, 0, 0, 0, 0,
    0, 0, 100, 0, 0, 0,
    0, 0, 0, 100, 0, 0,
    0, 0, 0, 0, 100, 0,
    0, 0, 0, 0, 0, 100
  }, {
    100, 0, 0, 0, 0, 0,
    0, 100, 0, 0, 0, 0,
    0, 0, 100, 0, 0, 0,
    0, 0, 0, 100, 0, 0,
    0, 0, 0, 0, 100, 0,
    0, 0, 0, 0, 0, 100
  }, {
    100, 0, 0, 0, 0, 0,
    0, 100, 0, 0, 0, 0,
    0, 0, 100, 0, 0, 0,
    0, 0, 0, 100, 0, 0,
    0, 0, 0, 0, 100, 0,
    0, 0, 0, 0, 0, 100
  }
};

float STR_controller(uint8_t IndexAngle) {
  uint8_t Index = IndexAngle;
  float r1, r2, s0, s1, s2, t0, t1, t2;
  float beta, detAA_inv, AA_adj[5][5], bb[5], xx[5];
  float hata1 = a1[Index], hata2 = a2[Index], hata3 = a3[Index], hatb0 = b0[Index], hatb1 = b1[Index], hatb2 = b2[Index];
  float hata12 = hata1 * hata1, hata13 = hata12 * hata1;
  float hata22 = hata2 * hata2, hata23 = hata22 * hata2;
  float hata32 = hata3 * hata3, hata33 = hata32 * hata3;
  float hatb02 = hatb0 * hatb0, hatb03 = hatb02 * hatb0;
  float hatb12 = hatb1 * hatb1, hatb13 = hatb12 * hatb1;
  float hatb22 = hatb2 * hatb2, hatb23 = hatb22 * hatb2;
  // solve the Diophantine Equation: AR + BS = A_c = A_m*A_o
  beta = (1 + am1[Index] + am2[Index] + am3[Index]) / (hatb0 + hatb1 + hatb2);
  bb[0] = am1[Index] + ao1[Index] - hata1;
  bb[1] = am2[Index] + ao1[Index] * am1[Index] + ao2[Index] - hata2;
  bb[2] = am3[Index] + ao1[Index] * am2[Index] + ao2[Index] * am1[Index] - hata3;
  bb[3] = ao1[Index] * am3[Index] + ao2[Index] * am2[Index];
  bb[4] = ao2[Index] * am3[Index];
  detAA_inv = 1.0 / (hata12 * hatb0 * hatb22 - 2 * hata1 * hatb02 * hata3 * hatb2 - hata1 * hatb0 * hata2 * hatb1 * hatb2 + hata1 * hatb0 * hatb12 * hata3
                     - hata1 * hatb1 * hatb22 + hatb03 * hata32 + hatb02 * hata22 * hatb2 - hatb02 * hata2 * hatb1 * hata3 - 2 * hatb0 * hata2 * hatb22
                     + 3 * hatb0 * hatb1 * hata3 * hatb2 + hata2 * hatb12 * hatb2 - hatb13 * hata3 + hatb23);
  AA_adj[0][0] = - hata3 * hatb13 + hata2 * hatb12 * hatb2 - hata1 * hatb1 * hatb22 + 2 * hatb0 * hata3 * hatb1 * hatb2 + hatb23 - hatb0 * hata2 * hatb22;
  AA_adj[0][1] = - hata3 * hatb02 * hatb2 + hata3 * hatb0 * hatb12 - hata2 * hatb0 * hatb1 * hatb2 + hata1 * hatb0 * hatb22;
  AA_adj[0][2] =   hata2 * hatb02 * hatb2 - hatb1 * hata3 * hatb02 - hatb0 * hatb22;
  AA_adj[0][3] =   hata3 * hatb03 - hata1 * hatb2 * hatb02 + hatb1 * hatb2 * hatb0;
  AA_adj[0][4] = - hata2 * hatb03 + hata1 * hatb02 * hatb1 + hatb2 * hatb02 - hatb0 * hatb12;
  AA_adj[1][0] = - hata3 * hatb12 * hatb2 + hata2 * hatb1 * hatb22 - hata1 * hatb23 + hatb0 * hata3 * hatb22;
  AA_adj[1][1] =   hatb23 - hatb0 * hata2 * hatb22 + hatb0 * hatb1 * hata3 * hatb2;
  AA_adj[1][2] = - hata3 * hatb02 * hatb2 + hata1 * hatb0 * hatb22 - hatb1 * hatb22;
  AA_adj[1][3] =   hata2 * hatb02 * hatb2 - hata1 * hatb0 * hatb1 * hatb2 - hatb0 * hatb22 + hatb12 * hatb2;
  AA_adj[1][4] =   hata3 * hatb03 - hata2 * hatb02 * hatb1 - hata1 * hatb2 * hatb02 + hata1 * hatb0 * hatb12 + 2 * hatb2 * hatb0 * hatb1 - hatb13;
  AA_adj[2][0] =   hata12 * hatb22 - 2 * hata1 * hatb0 * hata3 * hatb2 - hata1 * hata2 * hatb1 * hatb2 + hata1 * hatb12 * hata3 + hatb02 * hata32
                   + hatb0 * hata22 * hatb2 - hatb0 * hata2 * hatb1 * hata3 - hata2 * hatb22 + hatb1 * hata3 * hatb2;
  AA_adj[2][1] = - hata3 * hatb12 + hata2 * hatb1 * hatb2 - hata1 * hatb22 + hatb0 * hata3 * hatb2;
  AA_adj[2][2] =   hatb22 - hatb0 * hata2 * hatb2 + hatb0 * hatb1 * hata3;
  AA_adj[2][3] = - hata3 * hatb02 + hata1 * hatb2 * hatb0 - hatb1 * hatb2;
  AA_adj[2][4] =   hata2 * hatb02 - hata1 * hatb0 * hatb1 - hatb2 * hatb0 + hatb12;
  AA_adj[3][0] = - hata22 * hatb1 * hatb2 + hata2 * hatb12 * hata3 + hata1 * hata2 * hatb22 - hatb0 * hatb1 * hata32 - hata3 * hatb22;
  AA_adj[3][1] =   hatb02 * hata32 + hatb0 * hata22 * hatb2 - hatb1 * hatb0 * hata2 * hata3 - hata1 * hatb0 * hata3 * hatb2 - hata2 * hatb22
                   + hatb1 * hata3 * hatb2;
  AA_adj[3][2] =   hatb0 * hata3 * hatb2 - hatb12 * hata3 + hata2 * hatb1 * hatb2 - hata1 * hatb0 * hata2 * hatb2 + hata1 * hatb0 * hatb1 * hata3;
  AA_adj[3][3] =   hata12 * hatb0 * hatb2 - hata3 * hata1 * hatb02 - hatb1 * hata1 * hatb2 - hata2 * hatb0 * hatb2 + hatb1 * hata3 * hatb0 + hatb22;
  AA_adj[3][4] = - hata12 * hatb0 * hatb1 + hata2 * hata1 * hatb02 + hata1 * hatb12 - hata3 * hatb02 - hatb2 * hatb1;
  AA_adj[4][0] =   hatb12 * hata32 - hata2 * hatb1 * hata3 * hatb2 - hatb0 * hata32 * hatb2 + hata1 * hata3 * hatb22;
  AA_adj[4][1] = - hatb0 * hatb1 * hata32 - hata3 * hatb22 + hatb0 * hata2 * hata3 * hatb2;
  AA_adj[4][2] =   hatb02 * hata32 - hata1 * hatb2 * hatb0 * hata3 + hatb1 * hatb2 * hata3;
  AA_adj[4][3] = - hata2 * hata3 * hatb02 + hata1 * hata3 * hatb0 * hatb1 + hata3 * hatb2 * hatb0 - hata3 * hatb12;
  AA_adj[4][4] =   hata12 * hatb0 * hatb2 - hata3 * hata1 * hatb02 - hata1 * hatb0 * hata2 * hatb1 - hata1 * hatb1 * hatb2 + hatb02 * hata22
                   - 2 * hatb0 * hata2 * hatb2 + hata3 * hatb0 * hatb1 + hata2 * hatb12 + hatb22;
  for (int i = 0; i < 5; i++) {
    xx[i] = 0;
    for (int j = 0; j < 5; j++) xx[i] += AA_adj[i][j] * bb[j];
    xx[i] *= detAA_inv;
  }
  r1 = xx[0];
  r2 = xx[1];
  s0 = xx[2];
  s1 = xx[3];
  s2 = xx[4];
  t0 = beta;
  t1 = beta * ao1[Index];
  t2 = beta * ao2[Index];
  y_measure[Index][0] = Angle_measure[Index] * RadToDeg;
  u_compute[Index][0] = -ao1[Index] * u_compute[Index][1] - ao2[Index] * u_compute[Index][2]
                        + t0 * y_desire[Index][0]  + t1 * y_desire[Index][1]  + t2 * y_desire[Index][2]
                        - s0 * y_measure[Index][0] - s1 * y_measure[Index][1] - s2 * y_measure[Index][2]
                        + (ao1[Index] - r1) * u_actual[Index][1] + (ao2[Index] - r2) * u_actual[Index][2];
  u_actual[Index][0] = constrain( u_compute[Index][0], -FeedBackMax[Index], FeedBackMax[Index]);
  STR_flag = Index + 1;
  return u_actual[Index][0];
}

void STR_estimator(uint8_t IndexAngle) {
  uint8_t Index = IndexAngle;
  float varphi[6] = { -y_measure[Index][1], -y_measure[Index][2], -y_measure[Index][3], u_actual[Index][1], u_actual[Index][2], u_actual[Index][3]};
  float e = Angle_measure[Index] * RadToDeg;
  float alpha = lambda[Index];
  float K[6];
  float temp[6][6];
  float P_o[6][6];
  // Step 1: e = y - hattheta*varphi
  for (int i = 0; i < 6; i++) {
    e -= hattheta[Index][i] * varphi[i];
  }
  // Step 2: alpha = lambda + varphi'*P*varphi
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) alpha += varphi[i] * P[Index][i][j] * varphi[j];
  }
  // Step 3: K = P*varphi/alpha and hattheta = hattheta_o + K*e
  for (int i = 0; i < 6; i++) {
    K[i] = 0;
    for (int j = 0; j < 6; j++) K[i] += P[Index][i][j] * varphi[j];
    K[i] = K[i] / alpha;
    hattheta[Index][i] += K[i] * e;
  }
  // Step 4: temp = I-K*varphi'
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) temp[i][j] = 1 - K[i] * varphi[j];
      else temp[i][j] = -K[i] * varphi[j];
      P_o[i][j] = P[Index][i][j];
    }
  }
  // Step 5: P = (I-K*varphi')*P_o/lambda = temp*P_o/lambda
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P[Index][i][j] = 0;
      for (int k = 0; k < 6; k++) P[Index][i][j] += temp[i][k] * P_o[k][j];
      P[Index][i][j] = P[Index][i][j] / lambda[Index];
    }
  }
  a1[Index] = hattheta[Index][0];
  a2[Index] = hattheta[Index][1];
  a3[Index] = hattheta[Index][2];
  b0[Index] = hattheta[Index][3];
  b1[Index] = hattheta[Index][4];
  b2[Index] = hattheta[Index][5];
}
#endif

void Controller_updatedata() {
#if defined(PID_v2)
  for (int i = 0; i < 3; i++) {
    eTerm_inner_old[i] = eTerm_inner[i];
  }
#elif defined(PID_v3)
  for (int i = 0; i < 3; i++) {
    eTerm_old[i] = eTerm[i];
    DTerm_old[i] = DTerm[i];
  }
#endif
#ifdef STR_v1
  if (STR_flag == 0) {
  }
  else {
    uint8_t Index = STR_Index;
    STR_estimator(Index);
    y_measure[Index][3] = y_measure[Index][2];
    y_measure[Index][2] = y_measure[Index][1];
    y_measure[Index][1] = y_measure[Index][0];
    u_compute[Index][3] = u_compute[Index][2];
    u_compute[Index][2] = u_compute[Index][1];
    u_compute[Index][1] = u_compute[Index][0];
    u_actual[Index][3]  = u_actual[Index][2];
    u_actual[Index][2]  = u_actual[Index][1];
#if defined(QuadP)
    switch (Index) {
      case 0:
        u_actual[Index][1] = (-PWM_out[1] + PWM_out[3]) / 2;
        break;
      case 1:
        u_actual[Index][1] = ( PWM_out[0] - PWM_out[2]) / 2;
        break;
      case 2:
        u_actual[Index][1] = (-PWM_out[0] + PWM_out[1] - PWM_out[2] + PWM_out[3]) / 4;
        break;
    }
#elif defined(QuadX)
    switch (Index) {
      case 0:
        u_actual[Index][1] = (PWM_out[0] - PWM_out[1] - PWM_out[2] + PWM_out[3]) / 4;
        break;
      case 1:
        u_actual[Index][1] = (PWM_out[0] + PWM_out[1] - PWM_out[2] - PWM_out[3]) / 4;
        break;
      case 2:
        u_actual[Index][1] = (PWM_out[0] - PWM_out[1] + PWM_out[2] - PWM_out[3]) / 4;
        break;
    }
#endif
    STR_flag = 0;
  }
#endif
}
