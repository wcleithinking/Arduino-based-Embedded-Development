float FeedForward[4]  = {0, 0, 0, PWM_MIN};
float FeedBack[4]     = {0, 0, 0, 0};
float FeedAll[4]      = {0, 0, 0, PWM_MIN};
float FeedBackMax[4]  = {50, 50, 100, PWM_MAX};
float STR_FeedBack = 0;

void Controller_run() {
#ifdef FEEDDESIRE
  Angle_eulerdesire[IndexRoll]  = constrain(map(RC[IndexRoll], Pulse_MIN, Pulse_MAX, Roll_MIN, Roll_MAX), Roll_MIN, Roll_MAX);
  Angle_eulerdesire[IndexPitch] = constrain(map(RC[IndexPitch], Pulse_MIN, Pulse_MAX, Pitch_MIN, Pitch_MAX), Pitch_MIN, Pitch_MAX);
  Angle_eulerdesire[IndexYaw]   = constrain(map(RC[IndexYaw], Pulse_MIN, Pulse_MAX, Yaw_MIN, Yaw_MAX), Yaw_MIN, Yaw_MAX);
  for (int i = 0; i < 2; i++) {
    if ((1490 <= RC[i]) && (RC[i] <= 1510)) Angle_eulerdesire[i] = 0;
  }
  if ((1500 <= RC[2]) && (RC[2] <= 1600)) Angle_eulerdesire[2] = 0;
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
    case 0:
      FeedBack[IndexRoll]   = STR_FeedBack;
      FeedBack[IndexPitch]  = PID_controller(IndexPitch);
      FeedBack[IndexYaw]    = PID_controller(IndexYaw);
      break;
    case 1:
      FeedBack[IndexRoll]  = PID_controller(IndexRoll);
      FeedBack[IndexPitch] = STR_FeedBack;
      FeedBack[IndexYaw]   = PID_controller(IndexYaw);
      break;
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
  // all
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

void Controller_updatedata() {
  PID_updatedata();
#ifdef STR_v1
  STR_updatedata();
#endif
}

void Controller_cleardata() {
  PID_reset();
#ifdef STR_v1
  STR_reset();
#endif
}

