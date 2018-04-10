double hattheta1[6] = {1, 1, 1, 1, 1, 1};
double hattheta2[6] = {1, 1, 1, 1, 1, 1};
double hattheta3[6] = {1, 1, 1, 1, 1, 1};
double hattheta4[6] = {1, 1, 1, 1, 1, 1};
double P1[6][6] = {100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100};
double P2[6][6] = {100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100};
double P3[6][6] = {100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100};
double P4[6][6] = {100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100};
const double lambda1 = 0.98;
const double lambda2 = 0.98;
const double lambda3 = 0.98;
const double lambda4 = 0.98;

void dynamics_identify() {
  estimator1();
  estimator2();
  estimator3();
  estimator4();
}
// estimator1 for roll dynamic
void estimator1() {
  double varphi1[6] = { -roll_o, -roll_oo, -roll_ooo, u_o[0], u_oo[0], u_ooo[0] };
  double e1 = roll;
  double r1 = lambda1;
  double K1[6];
  double temp[6][6];
  double P1_o[6][6];
  // e = y - hattheta*varphi
  for (int i = 0; i < 6; i++) e1 -= hattheta1[i] * varphi1[i];
  // r = lambda + varphi'*P*varphi
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      r1 += varphi1[i] * P1[i][j] * varphi1[j];
    }
  }
  // K = P*varphi/r
  // hattheta = hattheta_o + K*e
  for (int i = 0; i < 6; i++) {
    K1[i] = 0;
    for (int j = 0; j < 6; j++) K1[i] += P1[i][j] * varphi1[j];
    K1[i] = K1[i] / r1;
    hattheta1[i] += K1[i] * e1;
  }
  // temp = I-K*varphi'
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) temp[i][j] = 1 - K1[i] * varphi1[j];
      else temp[i][j] = -K1[i] * varphi1[j];
      P1_o[i][j] = P1[i][j];
    }
  }
  // P = (I-K*varphi')*P_o/lambda
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P1[i][j] = 0;
      for (int k = 0; k < 6; k++) P1[i][j] += temp[i][k] * P1_o[k][j];
      P1[i][j] = P1[i][j] / lambda1;
    }
  }
  hata1[0] = hattheta1[0];
  hata2[0] = hattheta1[1];
  hata3[0] = hattheta1[2];
  hatb0[0] = hattheta1[3];
  hatb1[0] = hattheta1[4];
  hatb2[0] = hattheta1[5];
}

// estimator2 for pitch dynamic
void estimator2() {
  double varphi2[6] = { -pitch_o, -pitch_oo, -pitch_ooo, u_o[1], u_oo[1], u_ooo[1] };
  double e2 = pitch;
  double r2 = lambda2;
  double K2[6];
  double temp[6][6];
  double P2_o[6][6];
  // e = y - hattheta*varphi
  for (int i = 0; i < 6; i++) e2 -= hattheta2[i] * varphi2[i];
  // r = lambda + varphi'*P*varphi
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      r2 += varphi2[i] * P2[i][j] * varphi2[j];
    }
  }
  // K = P*varphi/r
  // hattheta = hattheta_o + K*e
  for (int i = 0; i < 6; i++) {
    K2[i] = 0;
    for (int j = 0; j < 6; j++) K2[i] += P2[i][j] * varphi2[j];
    K2[i] = K2[i] / r2;
    hattheta2[i] += K2[i] * e2;
  }
  // temp = I-K*varphi'
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) temp[i][j] = 1 - K2[i] * varphi2[j];
      else temp[i][j] = -K2[i] * varphi2[j];
      P2_o[i][j] = P2[i][j];
    }
  }
  // P = (I-K*varphi')*P_o/lambda
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P2[i][j] = 0;
      for (int k = 0; k < 6; k++) P2[i][j] += temp[i][k] * P2_o[k][j];
      P2[i][j] = P2[i][j] / lambda2;
    }
  }
  hata1[1] = hattheta2[0];
  hata2[1] = hattheta2[1];
  hata3[1] = hattheta2[2];
  hatb0[1] = hattheta2[3];
  hatb1[1] = hattheta2[4];
  hatb2[1] = hattheta2[5];
}

// estimator3 for yaw dynamic
void estimator3() {
  double varphi3[6] = { -yaw_o, -yaw_oo, -yaw_ooo, u_o[2], u_oo[2], u_ooo[2] };
  double e3 = yaw;
  double r3 = lambda3;
  double K3[6];
  double temp[6][6];
  double P3_o[6][6];
  // e = y - hattheta*varphi
  for (int i = 0; i < 6; i++) e3 -= hattheta3[i] * varphi3[i];
  // r = lambda + varphi'*P*varphi
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      r3 += varphi3[i] * P3[i][j] * varphi3[j];
    }
  }
  // K = P*varphi/r
  // hattheta = hattheta_o + K*e
  for (int i = 0; i < 6; i++) {
    K3[i] = 0;
    for (int j = 0; j < 6; j++) K3[i] += P3[i][j] * varphi3[j];
    K3[i] = K3[i] / r3;
    hattheta3[i] += K3[i] * e3;
  }
  // temp = I-K*varphi'
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) temp[i][j] = 1 - K3[i] * varphi3[j];
      else temp[i][j] = -K3[i] * varphi3[j];
      P3_o[i][j] = P3[i][j];
    }
  }
  // P = (I-K*varphi')*P_o/lambda
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P3[i][j] = 0;
      for (int k = 0; k < 6; k++) P3[i][j] += temp[i][k] * P3_o[k][j];
      P3[i][j] = P3[i][j] / lambda3;
    }
  }
  hata1[2] = hattheta3[0];
  hata2[2] = hattheta3[1];
  hata3[2] = hattheta3[2];
  hatb0[2] = hattheta3[3];
  hatb1[2] = hattheta3[4];
  hatb2[2] = hattheta3[5];
}

// estimator4 for altitude dynamic
void estimator4() {
  double varphi4[6] = { -altitude_o, -altitude_oo, -altitude_ooo, u_o[3], u_oo[3], u_ooo[3] };
  double e4 = altitude;
  double r4 = lambda4;
  double K4[6];
  double temp[6][6];
  double P4_o[6][6];
  // e = y - hattheta*varphi
  for (int i = 0; i < 6; i++) e4 -= hattheta4[i] * varphi4[i];
  // r = lambda + varphi'*P*varphi
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      r4 += varphi4[i] * P4[i][j] * varphi4[j];
    }
  }
  // K = P*varphi/r
  // hattheta = hattheta_o + K*e
  for (int i = 0; i < 6; i++) {
    K4[i] = 0;
    for (int j = 0; j < 6; j++) K4[i] += P4[i][j] * varphi4[j];
    K4[i] = K4[i] / r4;
    hattheta4[i] += K4[i] * e4;
  }
  // temp = I-K*varphi'
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) temp[i][j] = 1 - K4[i] * varphi4[j];
      else temp[i][j] = -K4[i] * varphi4[j];
      P4_o[i][j] = P4[i][j];
    }
  }
  // P = (I-K*varphi')*P_o/lambda
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P4[i][j] = 0;
      for (int k = 0; k < 6; k++) P4[i][j] += temp[i][k] * P4_o[k][j];
      P4[i][j] = P4[i][j] / lambda4;
    }
  }
  hata1[3] = hattheta4[0];
  hata2[3] = hattheta4[1];
  hata3[3] = hattheta4[2];
  hatb0[3] = hattheta4[3];
  hatb1[3] = hattheta4[4];
  hatb2[3] = hattheta4[5];
}
