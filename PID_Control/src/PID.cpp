#include "PID.h"
#include <iostream>
#include <cmath>
#include <limits>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	dbgPrint = true;

	//assign coefficients
	PID::Kp = Kp;
	PID::Kd = Kd;
	PID::Ki = Ki;

	//initialize individual errors to 0 to start with
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	min_error = std::numeric_limits<double>::max();
	max_error = 0;
	cte = 0;
	cte_t_1 = 0;
	loop_counter = 0;
	dp = { 0.1*Kp, 0.1*Kd, 0.1*Ki };
	validation_steps = 20;
	dwell_steps = 5;
	twiddle_increase = false;
	twiddle_decrease = false;
	idx_param = 2;

}

void PID::UpdateError(double cte) {

	p_error = cte;
	i_error += cte;
	d_error = cte - cte_t_1;
	cte_t_1 = cte;
	


}

double PID::TotalError(bool print, bool twiddle) {

	if ((loop_counter % (validation_steps + dwell_steps) > dwell_steps)){
		total_error += pow(cte, 2);
		cout << "total_error : " << total_error << endl;
	}
	
	if (twiddle && (loop_counter % (validation_steps + dwell_steps) == 0)){
		std::cout << "Iteration : " << loop_counter << " |  Total Error : " << total_error << " |  Best : " << min_error << std::endl;
		std::cout << "p_error : " << p_error << " |  i_error : " << i_error << " |  d_error : " << d_error << std::endl;

		if (total_error < min_error) {
			std::cout << "Correcting... \n" << std::endl;
			min_error = total_error;
			if (loop_counter != validation_steps + dwell_steps){
				dp[idx_param] *= 1.1;
			}

			idx_param = (idx_param + 1) % 3;
			twiddle_increase = twiddle_decrease = false;

		}

		if (!twiddle_increase && !twiddle_decrease) {
			//First, try increasing
			switch (idx_param){
			case 0:
				Kp = Kp + dp[idx_param];
				break;
			case 1:
				Kd = Kd + dp[idx_param];
				break;
			case 2:
				Ki = Ki + dp[idx_param];
				break;
			default:
				break;
			}
			twiddle_increase = true;
		}
		else if (twiddle_increase && !twiddle_decrease) {
			//Then, try decreasing
			switch (idx_param) {
			case 0:
				Kp = Kp - 2 * dp[idx_param];
				break;
			case 1:
				Kd = Kd - 2 * dp[idx_param];
				break;
			case 2:
				Ki = Ki - 2 * dp[idx_param];
				break;
			default:
				break;
			}
			twiddle_decrease = true;
		}
		else {
			//First, try increasing
			switch (idx_param) {
			case 0:
				Kp = Kp - 2 * dp[idx_param];
				break;
			case 1:
				Kd = Kd - 2 * dp[idx_param];
				break;
			case 2:
				Ki = Ki - 2 * dp[idx_param];
				break;
			default:
				break;
			}
			dp[idx_param] *= 0.9;
			idx_param = (idx_param + 1) % 3;
			twiddle_increase = twiddle_decrease = false;
		}
		
		cout << "loop_counter : " << loop_counter << endl;
		cout << "new parameters" << endl;
		cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;

	}
	total_error = 0;
	loop_counter++;
	//cout << "loop_counter : " << loop_counter << endl;
	return total_error;

}


