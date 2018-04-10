#ifndef _STR_H_
#define _STR_H_

#include <Arduino.h>

class STR {
public:
	STR();
	void init_estimator(float forgetting_factor, float Pmax, float a1_guess, float a2_guess, float b0_guess, float b1_guess);
	void init_controller(float xi, float omega_0, float mu, float sample_period);
	void update_output(float output_desired, float output_measure);
	void run_estimator(float *a1, float *a2, float *b0, float *b1);
	void run_controller(float *input_compute, float input_max);
	void get_u_actual(float u_actual);
	void resetdata();
private:
	float am1;
	float am2;
	float bm0;
	float bm1;
	float ao1;
	float y_desired[3];
	float y_measure[3];
	float v_compute[3];
	float u_compute[3];
	float lambda = 1;
	float P[4][4];
	float Pmax;
	float hattheta[4];
	float a1_prior;
	float a2_prior;
	float b0_prior;
	float b1_prior;
};

#endif