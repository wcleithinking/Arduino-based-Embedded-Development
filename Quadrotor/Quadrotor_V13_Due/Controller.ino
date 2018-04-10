// Feed Control of Vehicle, from Controllers
float FeedForward[4]  = {0, 0, 0, PWM_MIN};
float FeedBack[4]     = {0, 0, 0, 0};
float FeedAll[4]      = {0, 0, 0, PWM_MIN};
float FeedBackMax[4]  = {50, 50, 100, PWM_MAX};
float STR_FeedBack;

void Controller() {
#ifdef FEEDFORWARD
  // set the desired angles
  Angle_desire[IndexRoll]  = constrain(map(RC[IndexRoll], Pulse_MIN, Pulse_MAX, Roll_MIN, Roll_MAX), Roll_MIN, Roll_MAX);
  Angle_desire[IndexPitch] = constrain(map(RC[IndexPitch], Pulse_MIN, Pulse_MAX, Pitch_MIN, Pitch_MAX), Pitch_MIN, Pitch_MAX);
  // Angle_desire[IndexYaw]   = constrain(map(RC[IndexYaw], Pulse_MIN, Pulse_MAX, Yaw_MIN, Yaw_MAX), Yaw_MIN, Yaw_MAX);
  for (int i = 0; i < 3; i++) {
    if ((1490 <= RC[i]) && (RC[i] <= 1510)) Angle_desire[i] = 0;
  }
#endif
  // feedforward
  FeedForward[IndexRoll]      = 0;
  FeedForward[IndexPitch]     = 0;
  FeedForward[IndexYaw]       = 0;
  FeedForward[IndexAltitude]  = constrain(map(RC[IndexAltitude], Pulse_MIN, Pulse_MAX, PWM_MIN, PWM_MAX), PWM_MIN, PWM_MAX);
  // feedback
#ifdef STR_v1
  time_Index = STR_Index + 1;
  time_current[time_Index] = millis();
  if (time_current[time_Index] >= time_previous[time_Index] + STR_period) {
#ifdef DEBUG
    time_diff[time_Index] = time_current[time_Index] - time_previous[time_Index];
#endif
    time_previous[time_Index] = time_current[time_Index];
    STR_FeedBack  = STR_controller(STR_Index);
  }
  switch (STR_Index) {
    // STR_Roll
    case 0:
      FeedBack[IndexRoll]   = STR_FeedBack;
      FeedBack[IndexPitch]  = PID_controller(IndexPitch);
      FeedBack[IndexYaw]    = PID_controller(IndexYaw);
      break;
    // STR_Pitch
    case 1:
      FeedBack[IndexRoll]  = PID_controller(IndexRoll);
      FeedBack[IndexPitch] = STR_FeedBack;
      FeedBack[IndexYaw]   = PID_controller(IndexYaw);
      break;
    // STR_Yaw
    case 2:
      FeedBack[IndexRoll]   = PID_controller(IndexRoll);
      FeedBack[IndexPitch]  = PID_controller(IndexPitch);
      FeedBack[IndexYaw]    = STR_FeedBack;
      break;
  }
#else
  FeedBack[IndexRoll]   = PID_controller(IndexRoll);
  FeedBack[IndexPitch]  = PID_controller(IndexPitch);
  FeedBack[IndexYaw]    = PID_controller(IndexYaw);
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
float Kp_outer = 1.8, Ki_outer = 0.08, Kd_outer = 4.8;
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

/*
   STR Variables
*/
#ifdef STR_v1
// prior information
float a1[3] = { -1.15, -1.15, -1.15};
float a2[3] = {  0.15,  0.15,  0.15};
float b0[3] = {  0.05,  0.05,  0.05};
float b1[3] = {  0.03,  0.03,  0.03};
// performance specification
//                    roll      pitch     yaw(0.04s)
float am1[3]    = { -1.72101, -1.72101, -1.72101};
float am2[3]    = {  0.75578,  0.75578,  0.75578};
float ao1[3]    = {        0,        0,        0};
float lambda[3] = {     0.98,     0.98,     0.98};
float Pmax[3]   = {       20,       20,       20};
// outer loop
float Kp_STR_outer = 1.5, Ki_STR_outer = 0.2, Kd_STR_outer = 0;
float eTerm_STR_outer[3] = {0, 0, 0};
float PTerm_STR_outer[3] = {0, 0, 0};
float ITerm_STR_outer[3] = {0, 0, 0};
float DTerm_STR_outer[3] = {0, 0, 0};
float eTerm_STR_inner[3] = {0, 0, 0};
// inner loop
uint8_t STR_flag = 0;
float y_desire[3] = {0, 0, 0};
float y_measure[3] = {0, 0, 0};
float u_compute[3] = {0, 0, 0};
float u_actual[3] = {0, 0, 0};
float hattheta[4] = {a1[STR_Index],  a2[STR_Index],  b0[STR_Index],  b1[STR_Index]};
float P[4][4] = {
  Pmax[STR_Index], 0, 0, 0,
  0, Pmax[STR_Index], 0, 0,
  0, 0, Pmax[STR_Index], 0,
  0, 0, 0, Pmax[STR_Index]
};

float STR_controller(uint8_t IndexAngle) {
  uint8_t Index = IndexAngle;
  float STR_out;
  float r1, s0, s1, t0, t1;
  float beta, detAA_inv, AA_adj[3][3], bb[3], xx[3];
  float hata1 = a1[Index], hata2 = a2[Index], hatb0 = b0[Index], hatb1 = b1[Index];
  // solve the Diophantine Equation: AR + BS = A_c = A_m*A_o
  beta = (1 + am1[Index] + am2[Index]) / (hatb0 + hatb1);
  bb[0] = am1[Index] + ao1[Index] - hata1;
  bb[1] = am2[Index] + ao1[Index] * am1[Index] - hata2;
  bb[2] = ao1[Index] * am2[Index] ;
  detAA_inv = 1.0 / (hatb1 * hatb1 - hata1 * hatb0 * hatb1 + hata2 * hatb0 * hatb0);
  AA_adj[0][0] =  hatb1 * hatb1;
  AA_adj[0][1] = -hatb0 * hatb1;
  AA_adj[0][2] =  hatb0 * hatb0;
  AA_adj[1][0] =  hata2 * hatb0 - hata1 * hatb1;
  AA_adj[1][1] =  hatb1;
  AA_adj[1][2] = -hatb0;
  AA_adj[2][0] = -hata2 * hatb1;
  AA_adj[2][1] =  hata2 * hatb0;
  AA_adj[2][2] =  hatb1 - hata1 * hatb0;
  for (int i = 0; i < 3; i++) {
    xx[i] = 0;
    for (int j = 0; j < 3; j++) xx[i] += AA_adj[i][j] * bb[j];
    xx[i] *= detAA_inv;
  }
  r1 = xx[0];
  s0 = xx[2];
  s1 = xx[3];
  t0 = beta;
  t1 = beta * ao1[Index];
  eTerm_STR_outer[Index]  = Angle_desire[Index] - Angle_measure[Index] * RadToDeg;
  // eTerm_STR_inner[Index]  = Rate_desire[Index]  - Rate_measure[Index]  * RadToDeg;
  PTerm_STR_outer[Index]  = Kp_STR_outer * eTerm_STR_outer[Index];
  ITerm_STR_outer[Index] += Ki_STR_outer * eTerm_STR_outer[Index] * STR_period * 0.001;
  // DTerm_STR_outer[Index]  = Kd_STR_outer * eTerm_STR_inner[Index];
  y_desire[0]  = PTerm_STR_outer[Index] + ITerm_STR_outer[Index]; // + DTerm_STR_outer[Index];
  y_measure[0] = Rate_measure[Index] * RadToDeg;
  u_compute[0] = -ao1[Index] * u_compute[1]
                 + t0 * y_desire[0]  + t1 * y_desire[1]
                 - s0 * y_measure[0] - s1 * y_measure[1]
                 + (ao1[Index] - r1) * u_actual[1];
  u_actual[0] = constrain( u_compute[0], -FeedBackMax[Index], FeedBackMax[Index]);
  STR_flag = Index + 1;
  STR_out  = u_actual[0];
  return STR_out;
}

void STR_estimator(uint8_t IndexAngle) {
  uint8_t Index = IndexAngle;
  float varphi[4] = { -y_measure[1], -y_measure[2], u_actual[1], u_actual[2]};
  float e = Rate_measure[Index] * RadToDeg;
  float alpha = lambda[Index];
  float K[4];
  float temp[4][4];
  float P_o[4][4];
  // Step 1: e = y - hattheta*varphi
  for (int i = 0; i < 4; i++) {
    e -= hattheta[i] * varphi[i];
  }
  //e = e / (1 + abs(e));
  // Step 2: alpha = lambda + varphi'*P*varphi
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) alpha += varphi[i] * P[i][j] * varphi[j];
  }
  // Step 3: K = P*varphi/alpha and hattheta = hattheta_o + K*e
  for (int i = 0; i < 4; i++) {
    K[i] = 0;
    for (int j = 0; j < 4; j++) K[i] += P[i][j] * varphi[j];
    K[i] = K[i] / alpha;
    hattheta[i] += K[i] * e;
  }
  // Step 4: temp = I-K*varphi'
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == j) temp[i][j] = 1 - K[i] * varphi[j];
      else temp[i][j] = -K[i] * varphi[j];
      P_o[i][j] = P[i][j];
    }
  }
  // Step 5: P = (I-K*varphi')*P_o/lambda = temp*P_o/lambda
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P[i][j] = 0;
      for (int k = 0; k < 4; k++) P[i][j] += temp[i][k] * P_o[k][j];
      P[i][j] = P[i][j] / lambda[Index];
    }
  }
  a1[Index] = hattheta[0];
  a2[Index] = hattheta[1];
  b0[Index] = hattheta[2];
  b1[Index] = hattheta[3];
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
    uint8_t Index = STR_flag - 1;
    STR_estimator(Index);
    y_measure[2] = y_measure[1];
    y_measure[1] = y_measure[0];
    u_compute[2] = u_compute[1];
    u_compute[1] = u_compute[0];
    u_actual[2]  = u_actual[1];
#if defined(QuadP)
    switch (Index) {
      case 0:
        u_actual[1] = (-PWM_out[1] + PWM_out[3]) / 2;
        break;
      case 1:
        u_actual[1] = ( PWM_out[0] - PWM_out[2]) / 2;
        break;
      case 2:
        u_actual[1] = (-PWM_out[0] + PWM_out[1] - PWM_out[2] + PWM_out[3]) / 4;
        break;
    }
#elif defined(QuadX)
    switch (Index) {
      case 0:
        u_actual[1] = (PWM_out[0] - PWM_out[1] - PWM_out[2] + PWM_out[3]) / 4;
        break;
      case 1:
        u_actual[1] = (PWM_out[0] + PWM_out[1] - PWM_out[2] - PWM_out[3]) / 4;
        break;
      case 2:
        u_actual[1] = (PWM_out[0] - PWM_out[1] + PWM_out[2] - PWM_out[3]) / 4;
        break;
    }
#endif
    STR_flag = 0;
  }
#endif
}

void Controller_cleardata() {
  for (int i = 0; i < 3; i++) {
    y_desire[i]  = 0;
    y_measure[i] = 0;
    u_compute[i] = 0;
    u_actual[i] = 0;
  }
}

