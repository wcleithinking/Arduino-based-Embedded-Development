/****************************************************************************/
/*
  PID Variables
*/
/****************************************************************************/
#if defined(PID_v1)
float Kp_outer = 1.8, Ki_outer = 0.02, Kd_outer = 4.8;
float eTerm_outer[3] = {0, 0, 0};
float PTerm_outer[3] = {0, 0, 0};
float ITerm_outer[3] = {0, 0, 0};
float DTerm_outer[3] = {0, 0, 0};
float eTerm_inner[3] = {0, 0, 0};
#elif defined(PID_v2)
float Kp_outer = 1.8, Ki_outer = 0.02, Kd_outer = 4.8;
float eTerm_outer[3] = {0, 0, 0};
float PTerm_outer[3] = {0, 0, 0};
float ITerm_outer[3] = {0, 0, 0};
float DTerm_outer[3] = {0, 0, 0};
float Kp_inner = 1.8, Ki_inner = 0.02, Kd_inner = 4.8;
float eTerm_inner[3] = {0, 0, 0};
float PTerm_inner[3] = {0, 0, 0};
float ITerm_inner[3] = {0, 0, 0};
float DTerm_inner[3] = {0, 0, 0};
float eTerm_inner_old[3] = {0, 0, 0};
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
  eTerm_outer[Index]  = Angle_eulerdesire[Index] - Angle_newestimate[Index] * RadToDeg;
  eTerm_inner[Index]  = Rate_middledesire[Index]  - Rate_threemeasure[Index]  * RadToDeg;
  PTerm_outer[Index]  = Kp_outer * eTerm_outer[Index];
  ITerm_outer[Index] += Ki_outer * eTerm_outer[Index] * dt;
  DTerm_outer[Index]  = Kd_outer * eTerm_inner[Index];
  PID_out = PTerm_outer[Index] + ITerm_outer[Index] + DTerm_outer[Index];
#elif defined(PID_v2)
  eTerm_outer[Index]  = Angle_eulerdesire[Index] - Angle_newestimate[Index] * RadToDeg;
  eTerm_inner[Index]  = Rate_middledesire[Index]  - Rate_threemeasure[Index]  * RadToDeg;
  PTerm_outer[Index]  = Kp_outer * eTerm_outer[Index];
  ITerm_outer[Index] += Ki_outer * eTerm_outer[Index] * dt;
  DTerm_outer[Index]  = Kd_outer * eTerm_inner[Index];
  Rate_middledesire[Index]  = PTerm_outer[Index] + ITerm_outer[Index] + DTerm_outer[Index];
  Rate_middledesire[Index]  = constrain(Rate_middledesire[Index], -FeedBackMax[Index], FeedBackMax[Index]);
  eTerm_inner[Index]  = Rate_middledesire[Index]  - Rate_threemeasure[Index]  * RadToDeg;
  eTerm_innerinner[Index] = (eTerm_inner[Index] - eTerm_inner_old[Index]) / dt;
  PTerm_inner[Index]  = Kp_inner * eTerm_inner[Index];
  ITerm_inner[Index] += Ki_inner * eTerm_inner[Index] * dt;
  DTerm_inner[Index]  = Kd_inner * eTerm_innerinner[Index];
  PID_out = PTerm_inner[Index] + ITerm_inner[Index] + DTerm_inner[Index];
#elif defined(PID_v3)
  eTerm[Index]  = Angle_eulerdesire[Index] - Angle_newestimate[Index] * RadToDeg;
  PTerm[Index]  = K[Index] * eTerm[Index];
  DTerm[Index]  = K[Index] * (exp(-dt * N[Index] / Td[Index]) * DTerm_old[Index] + N[Index] * (eTerm[Index] - eTerm_old[Index]));
  PID_out       = PTerm[Index] + ITerm[Index] + DTerm[Index];
  ITerm[Index]  = K[Index] * (ITerm[Index] + (dt / Ti[Index]) * eTerm[Index]);
#endif
  PID_out = constrain(PID_out, -FeedBackMax[Index], FeedBackMax[Index]);
  return PID_out;
}

void PID_updatedata() {
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
}

void PID_reset() {
#if defined(PID_v1)
  for (int i = 0; i < 3; i++) {
    ITerm_outer[i] = 0;
  }
#elif defined (PID_v2)
  for (int i = 0; i < 3 ; i++) {
    ITerm_outer[i] = 0;
    ITerm_inner[i] = 0;
  }
#elif defined(PID_v3)
  for (int i = 0; i < 3; i++) {
    ITerm[i] = 0;
  }
#endif
}

