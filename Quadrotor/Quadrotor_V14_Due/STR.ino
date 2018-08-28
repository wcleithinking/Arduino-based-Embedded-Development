/*
   STR Variables
*/
#ifdef STR_v1
// outer loop
float Kp_STR_outer = 1.5, Ki_STR_outer = 0.01, Kd_STR_outer = 0;
float eTerm_STR_outer[3] = {0, 0, 0};
float PTerm_STR_outer[3] = {0, 0, 0};
float ITerm_STR_outer[3] = {0, 0, 0};
float DTerm_STR_outer[3] = {0, 0, 0};
float eTerm_STR_inner[3] = {0, 0, 0};

float STR_controller(uint8_t IndexAngle) {
  uint8_t Index = IndexAngle;
  float STR_out;
  STR_flag = Index + 1;
  eTerm_STR_outer[Index]  = Angle_desire[Index] - Angle_measure[Index] * RadToDeg;
  eTerm_STR_inner[Index]  = Rate_desire[Index]  - Rate_measure[Index]  * RadToDeg;
  PTerm_STR_outer[Index]  = Kp_STR_outer * eTerm_STR_outer[Index];
  ITerm_STR_outer[Index] += Ki_STR_outer * eTerm_STR_outer[Index] * STR_period / 1000;
  DTerm_STR_outer[Index]  = Kd_STR_outer * eTerm_STR_inner[Index];
  float output_desired = PTerm_STR_outer[Index] + ITerm_STR_outer[Index] + DTerm_STR_outer[Index];
  float output_measure = Rate_measure[Index] * RadToDeg;
  Rate_desire[Index] = output_desired;
  mySTR.update_output(output_desired, output_measure);
  mySTR.run_estimator(&a1, &a2, &b0, &b1);
  mySTR.run_controller(&STR_out, FeedBackMax[Index]);
  return STR_out;
}

void STR_reset() {
  for (int i = 0; i < 3; i++) {
    ITerm_STR_outer[i] = 0;
  }
  mySTR.resetdata();
}
#endif
