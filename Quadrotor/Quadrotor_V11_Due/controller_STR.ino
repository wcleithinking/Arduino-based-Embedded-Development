#ifdef STR_YAW
float hattheta[6] = {hata1, hata2, hata3, hatb0, hatb1, hatb2};
float P[6][6] = {100, 0, 0, 0, 0, 0,
                 0, 100, 0, 0, 0, 0,
                 0, 0, 100, 0, 0, 0,
                 0, 0, 0, 100, 0, 0,
                 0, 0, 0, 0, 100, 0,
                 0, 0, 0, 0, 0, 100
                };
float lambda = 0.98;
void STR_estimator_yaw() {
  float varphi[6] = { -yaw_o, -yaw_oo, -yaw_ooo, u_yaw_o, u_yaw_oo, u_yaw_ooo};
  float e = yaw * RadToDeg;
  float alpha = lambda;
  float K[6];
  float temp[6][6];
  float P_o[6][6];
  // Step 1: e = y - hattheta*varphi
  for (int i = 0; i < 6; i++) {
    e -= hattheta[i] * varphi[i];
  }
  // Step 2: alpha = lambda + varphi'*P*varphi
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) alpha += varphi[i] * P[i][j] * varphi[j];
  }
  // Step 3: K = P*varphi/alpha and hattheta = hattheta_o + K*e
  for (int i = 0; i < 6; i++) {
    K[i] = 0;
    for (int j = 0; j < 6; j++) K[i] += P[i][j] * varphi[j];
    K[i] = K[i] / alpha;
    hattheta[i] += K[i] * e;
  }
  // Step 4: temp = I-K*varphi'
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) temp[i][j] = 1 - K[i] * varphi[j];
      else temp[i][j] = -K[i] * varphi[j];
      P_o[i][j] = P[i][j];
    }
  }
  // Step 5: P = (I-K*varphi')*P_o/lambda = temp*P_o/lambda
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P[i][j] = 0;
      for (int k = 0; k < 6; k++) P[i][j] += temp[i][k] * P_o[k][j];
      P[i][j] = P[i][j] / lambda;
    }
  }
  hata1 = hattheta[0];
  hata2 = hattheta[1];
  hata3 = hattheta[2];
  hatb0 = hattheta[3];
  hatb1 = hattheta[4];
  hatb2 = hattheta[5];
}

void STR_controller_yaw() {
  float r1, r2, s0, s1, s2, t0, t1, t2;
  float beta, detA_inv, A_adj[5][5], b[5], x[5];
  float hata12 = hata1 * hata1;
  float hata13 = hata12 * hata1;
  float hata22 = hata2 * hata2;
  float hata23 = hata22 * hata2;
  float hata32 = hata3 * hata3;
  float hata33 = hata32 * hata3;
  float hatb02 = hatb0 * hatb0;
  float hatb03 = hatb02 * hatb0;
  float hatb12 = hatb1 * hatb1;
  float hatb13 = hatb12 * hatb1;
  float hatb22 = hatb2 * hatb2;
  float hatb23 = hatb22 * hatb2;
  // solve the Diophantine Equation: AR + BS = A_c = A_m*A_o
  beta = (1 + am1 + am2 + am3) / (hatb0 + hatb1 + hatb2);
  b[0] = am1 + ao1 - hata1;
  b[1] = am2 + ao1 * am1 + ao2 - hata2;
  b[2] = am3 + ao1 * am2 + ao2 * am1 - hata3;
  b[3] = ao1 * am3 + ao2 * am2;
  b[4] = ao2 * am3;
  detA_inv = 1.0 / (hata12 * hatb0 * hatb22 - 2 * hata1 * hatb02 * hata3 * hatb2 - hata1 * hatb0 * hata2 * hatb1 * hatb2 + hata1 * hatb0 * hatb12 * hata3
                    - hata1 * hatb1 * hatb22 + hatb03 * hata32 + hatb02 * hata22 * hatb2 - hatb02 * hata2 * hatb1 * hata3 - 2 * hatb0 * hata2 * hatb22
                    + 3 * hatb0 * hatb1 * hata3 * hatb2 + hata2 * hatb12 * hatb2 - hatb13 * hata3 + hatb23);
  A_adj[0][0] = - hata3 * hatb13 + hata2 * hatb12 * hatb2 - hata1 * hatb1 * hatb22 + 2 * hatb0 * hata3 * hatb1 * hatb2 + hatb23 - hatb0 * hata2 * hatb22;
  A_adj[0][1] = - hata3 * hatb02 * hatb2 + hata3 * hatb0 * hatb12 - hata2 * hatb0 * hatb1 * hatb2 + hata1 * hatb0 * hatb22;
  A_adj[0][2] =   hata2 * hatb02 * hatb2 - hatb1 * hata3 * hatb02 - hatb0 * hatb22;
  A_adj[0][3] =   hata3 * hatb03 - hata1 * hatb2 * hatb02 + hatb1 * hatb2 * hatb0;
  A_adj[0][4] = - hata2 * hatb03 + hata1 * hatb02 * hatb1 + hatb2 * hatb02 - hatb0 * hatb12;
  A_adj[1][0] = - hata3 * hatb12 * hatb2 + hata2 * hatb1 * hatb22 - hata1 * hatb23 + hatb0 * hata3 * hatb22;
  A_adj[1][1] =   hatb23 - hatb0 * hata2 * hatb22 + hatb0 * hatb1 * hata3 * hatb2;
  A_adj[1][2] = - hata3 * hatb02 * hatb2 + hata1 * hatb0 * hatb22 - hatb1 * hatb22;
  A_adj[1][3] =   hata2 * hatb02 * hatb2 - hata1 * hatb0 * hatb1 * hatb2 - hatb0 * hatb22 + hatb12 * hatb2;
  A_adj[1][4] =   hata3 * hatb03 - hata2 * hatb02 * hatb1 - hata1 * hatb2 * hatb02 + hata1 * hatb0 * hatb12 + 2 * hatb2 * hatb0 * hatb1 - hatb13;
  A_adj[2][0] =   hata12 * hatb22 - 2 * hata1 * hatb0 * hata3 * hatb2 - hata1 * hata2 * hatb1 * hatb2 + hata1 * hatb12 * hata3 + hatb02 * hata32
                  + hatb0 * hata22 * hatb2 - hatb0 * hata2 * hatb1 * hata3 - hata2 * hatb22 + hatb1 * hata3 * hatb2;
  A_adj[2][1] = - hata3 * hatb12 + hata2 * hatb1 * hatb2 - hata1 * hatb22 + hatb0 * hata3 * hatb2;
  A_adj[2][2] =   hatb22 - hatb0 * hata2 * hatb2 + hatb0 * hatb1 * hata3;
  A_adj[2][3] = - hata3 * hatb02 + hata1 * hatb2 * hatb0 - hatb1 * hatb2;
  A_adj[2][4] =   hata2 * hatb02 - hata1 * hatb0 * hatb1 - hatb2 * hatb0 + hatb12;
  A_adj[3][0] = - hata22 * hatb1 * hatb2 + hata2 * hatb12 * hata3 + hata1 * hata2 * hatb22 - hatb0 * hatb1 * hata32 - hata3 * hatb22;
  A_adj[3][1] =   hatb02 * hata32 + hatb0 * hata22 * hatb2 - hatb1 * hatb0 * hata2 * hata3 - hata1 * hatb0 * hata3 * hatb2 - hata2 * hatb22
                  + hatb1 * hata3 * hatb2;
  A_adj[3][2] =   hatb0 * hata3 * hatb2 - hatb12 * hata3 + hata2 * hatb1 * hatb2 - hata1 * hatb0 * hata2 * hatb2 + hata1 * hatb0 * hatb1 * hata3;
  A_adj[3][3] =   hata12 * hatb0 * hatb2 - hata3 * hata1 * hatb02 - hatb1 * hata1 * hatb2 - hata2 * hatb0 * hatb2 + hatb1 * hata3 * hatb0 + hatb22;
  A_adj[3][4] = - hata12 * hatb0 * hatb1 + hata2 * hata1 * hatb02 + hata1 * hatb12 - hata3 * hatb02 - hatb2 * hatb1;
  A_adj[4][0] =   hatb12 * hata32 - hata2 * hatb1 * hata3 * hatb2 - hatb0 * hata32 * hatb2 + hata1 * hata3 * hatb22;
  A_adj[4][1] = - hatb0 * hatb1 * hata32 - hata3 * hatb22 + hatb0 * hata2 * hata3 * hatb2;
  A_adj[4][2] =   hatb02 * hata32 - hata1 * hatb2 * hatb0 * hata3 + hatb1 * hatb2 * hata3;
  A_adj[4][3] = - hata2 * hata3 * hatb02 + hata1 * hata3 * hatb0 * hatb1 + hata3 * hatb2 * hatb0 - hata3 * hatb12;
  A_adj[4][4] =   hata12 * hatb0 * hatb2 - hata3 * hata1 * hatb02 - hata1 * hatb0 * hata2 * hatb1 - hata1 * hatb1 * hatb2 + hatb02 * hata22
                  - 2 * hatb0 * hata2 * hatb2 + hata3 * hatb0 * hatb1 + hata2 * hatb12 + hatb22;
  for (int i = 0; i < 5; i++) {
    x[i] = 0;
    for (int j = 0; j < 5; j++) x[i] += A_adj[i][j] * b[j];
    x[i] *= detA_inv;
  }
  r1 = x[0];
  r2 = x[1];
  s0 = x[2];
  s1 = x[3];
  s2 = x[4];
  t0 = beta;
  t1 = beta * ao1;
  t2 = beta * ao2;
  v_yaw = -ao1 * v_yaw_o - ao2 * v_yaw_oo + t0 * yaw_d + t1 * yaw_d_o + t2 * yaw_d_oo - s0 * yaw * RadToDeg - s1 * yaw_o - s2 * yaw_oo
          + (ao1 - r1) * u_yaw_o + (ao2 - r2) * u_yaw_oo;
  yaw_feedback = constrain( v_yaw, -2 * umax, 2 * umax);
  STR_YAW_flag = 1;
}

void STR_YAW_updatedata() {
  if (STR_YAW_flag == 1) {
    STR_estimator_yaw();
    yaw_ooo = yaw_oo;
    yaw_oo = yaw_o;
    yaw_o = yaw * RadToDeg;
    v_yaw_oo = v_yaw_o;
    v_yaw_o = v_yaw;
    u_yaw_ooo = u_yaw_oo;
    u_yaw_oo = u_yaw_o;
    u_yaw_o = (-PWM_out[0] + PWM_out[1] - PWM_out[2] + PWM_out[3]) / 4;
    STR_YAW_flag = 0;
  }
}
#endif
