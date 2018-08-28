#include "STR.h"

STR::STR() {
	am1 = 1;
	am2 = 1;
	bm0 = 1;
	bm1 = 1;
	ao1 = 0;
	for (int i = 0; i < 4; i++) {
		if (i < 3) {
			y_desired[i] = 0;
			y_measure[i] = 0;
			v_compute[i] = 0;
			u_compute[i] = 0;
		}
		hattheta[i] = 1;
		for (int j = 0; j < 4; j++) {
			if (i==j) P[i][j] = 50;
			else P[i][j] = 0;
		}
	}
}

void STR::init_estimator(float forgetting_factor, float maxP, float a1_guess, float a2_guess, float b0_guess, float b1_guess) {
	lambda = forgetting_factor;
	Pmax = maxP;
	for (int i=0; i<4;i++) P[i][i] = Pmax;
	a1_prior = a1_guess;
	a2_prior = a2_guess;
	b0_prior = b0_guess;
	b1_prior = b1_guess;
}	

void STR::init_controller(float xi, float omega_0, float mu, float sample_period) {
	float omega = omega_0 * sqrt(1 - xi * xi);
	float alpha = exp(-xi * omega_0 * sample_period);
	float beta  = cos(omega * sample_period);
	float gamma = sin(omega * sample_period);
	float temp  = xi * omega_0 / omega * gamma;
	am1 = -2 * alpha * beta;
	am2 = alpha * alpha;
	bm0 = 1 - alpha * (beta + temp);
	bm1 = am2 + alpha * (temp - beta);
	ao1 = mu;
}

void STR::update_output(float output_desired, float output_measure) {
	y_desired[2] = y_desired[1];
	y_desired[1] = y_desired[0];
	y_desired[0] = output_desired;
	y_measure[2] = y_measure[1];
	y_measure[1] = y_measure[0];
	y_measure[0] = output_measure;
	v_compute[2] = v_compute[1];
	v_compute[1] = v_compute[0];
	// wait the controller to compute v
	u_compute[2] = u_compute[1];
	u_compute[1] = u_compute[0];
	// wait the controller to compute u
}

void STR::run_estimator(float *a1, float *a2, float *b0, float *b1) {
	float varphi[4] = {-y_measure[1], -y_measure[2], u_compute[1], u_compute[2]};
	float e = y_measure[0];
	float alpha = lambda;
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
		for (int j = 0; j < 4; j++) {
			K[i] += P[i][j] * varphi[j];
		}
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
				P[i][j] = P[i][j] / lambda;
		}
	}
	*a1 = hattheta[0];
	*a2 = hattheta[1];
	*b0 = hattheta[2];
	*b1 = hattheta[3];

}

void STR::run_controller(float *input_compute, float input_max) {
	float r1, s0, s1, t0, t1;
	float detAA_inv, AA_adj[3][3], bb[3], xx[3];
	float hata1 = hattheta[0];
	float hata2 = hattheta[1];
	float hatb0 = hattheta[2];
	float hatb1 = hattheta[3];
	float beta = (1 + am1 + am2) / (hatb0 + hatb1);
	bb[0] = am1 + ao1 - hata1;
	bb[1] = am2 + ao1 * am1 - hata2;
	bb[2] = ao1 * am2;
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
	t1 = beta * ao1;
	v_compute[0] = 	-ao1 * v_compute[1]
					+ t0 * y_desired[0] + t1 * y_desired[1]
					- s0 * y_measure[0] - s1 * y_measure[1]
					+ (ao1 - r1) * u_compute[1];
	u_compute[0] = constrain( v_compute[0], -input_max, input_max);
	*input_compute  = u_compute[0];
}

void STR::get_u_actual(float u_actual) {
	u_compute[0] = u_actual;
}

void STR::resetdata() {
	for (int i = 0; i < 4; i++) {
		if (i < 3) {
			y_desired[i] = 0;
			y_measure[i] = 0;
			v_compute[i] = 0;
			u_compute[i] = 0;
		}
		for (int j = 0; j < 4; j++) {
			if (i==j) P[i][j] = Pmax;
			else P[i][j] = 0;
		}
	}
	hattheta[0] = a1_prior;
	hattheta[1] = a2_prior;
	hattheta[2] = b0_prior;
	hattheta[3] = b1_prior;
}