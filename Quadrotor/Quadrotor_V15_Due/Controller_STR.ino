/****************************************************************************/
/*
   STR Variables
*/
/****************************************************************************/
#ifdef STR_v1

bool STR_flag = 0;
// tf coefficients
float a1 = 1;
float a2 = 1;
float b0 = 1;
float b1 = 1;
// outer loop
float Kp_STR_outer = 1.5, Ki_STR_outer = 0.01, Kd_STR_outer = 0;
float eTerm_STR_outer[3] = {0, 0, 0};
float PTerm_STR_outer[3] = {0, 0, 0};
float ITerm_STR_outer[3] = {0, 0, 0};
float DTerm_STR_outer[3] = {0, 0, 0};
float eTerm_STR_inner[3] = {0, 0, 0};

float STR_controller(uint8_t IndexAngle) {
  STR_flag = 1;
  uint8_t Index = IndexAngle;
  float STR_out;
  eTerm_STR_outer[Index]  = Angle_eulerdesire[Index] - Angle_newestimate[Index] * RadToDeg;
  eTerm_STR_inner[Index]  = Rate_middledesire[Index]  - Rate_threemeasure[Index]  * RadToDeg;
  PTerm_STR_outer[Index]  = Kp_STR_outer * eTerm_STR_outer[Index];
  ITerm_STR_outer[Index] += Ki_STR_outer * eTerm_STR_outer[Index] * STR_period / 1000;
  DTerm_STR_outer[Index]  = Kd_STR_outer * eTerm_STR_inner[Index];
  float output_desired = PTerm_STR_outer[Index] + ITerm_STR_outer[Index] + DTerm_STR_outer[Index];
  float output_measure = Rate_threemeasure[Index] * RadToDeg;
  Rate_middledesire[Index] = output_desired;
  mySTR.update_output(output_desired, output_measure);
  mySTR.run_estimator(&a1, &a2, &b0, &b1);
  mySTR.run_controller(&STR_out, FeedBackMax[Index]);
  return STR_out;
}

void STR_updatedata() {
  if (STR_flag == 1) {
    float u_actual;
#if defined(QuadP)
    switch (STR_Index) {
      case 0:
        u_actual = (-PWM_out[1] + PWM_out[3]) / 2;
        break;
      case 1:
        u_actual = ( PWM_out[0] - PWM_out[2]) / 2;
        break;
      case 2:
        u_actual = (-PWM_out[0] + PWM_out[1] - PWM_out[2] + PWM_out[3]) / 4;
        break;
    }
#elif defined(QuadX)
    switch (STR_Index) {
      case 0:
        u_actual = (PWM_out[0] - PWM_out[1] - PWM_out[2] + PWM_out[3]) / 4;
        break;
      case 1:
        u_actual = (PWM_out[0] + PWM_out[1] - PWM_out[2] - PWM_out[3]) / 4;
        break;
      case 2:
        u_actual = (PWM_out[0] - PWM_out[1] + PWM_out[2] - PWM_out[3]) / 4;
        break;
    }
#endif
    mySTR.get_u_actual(u_actual);
    STR_flag = 0;
  }
}

void STR_reset() {
  for (int i = 0; i < 3; i++) {
    ITerm_STR_outer[i] = 0;
  }
  mySTR.resetdata();
}

#endif
