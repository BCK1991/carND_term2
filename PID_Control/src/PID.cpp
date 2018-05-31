#include "PID.h"
#include <iostream>
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
	validation_steps = 100;
	dwell_steps = 1000
	twiddle_increase = false;
	twiddle_decrease = false;

}

void PID::UpdateError(double cte) {

	p_error = cte;
	i_error += cte;
	d_error = cte - cte_t_1;
	cte_t_1 = cte;
	


}

double PID::TotalError(bool print, bool twiddle) {

	if ((loop_counter % (validation_steps + dwell_steps) > 0))
		total_error = += pow(cte, 2);
	
	if (twiddle && (loop_counter % (validation_steps + dwell_steps) == 0)){
		std::cout << "Iteration : " << loop_counter << " |  Total Error : " << total_error << " |  Best : " << min_error << std::endl;
		std::cout << "p_error : " << p_error << " |  i_error : " << i_error << " |  d_error : " << d_error << std::endl;

		if (total_error < min_error) {
			std::cout << "Correcting... \n" << std::endl;
			min_error = total_error;
			if (step != validation_steps + dwell_steps){
				dp[param_index] *= 1.1;
			}

			param_index = (param_index + 1) % 3;
			twiddle_increase = twiddle_decrease = false;

		}

		if (!twiddle_increase && !tried_decrease) {
			//First, try increasing
			switch (param_index){
			case 0:
				Kp = Kp + dp[param_index];
				break;
			case 1:
				Kd = Kd + dp[param_index];
				break;
			case 2:
				Ki = Ki + dp[param_index];
				break;
			default:
				break;
			}
			twiddle_increase = true;
		}
		else if (twiddle_increase && !twiddle_decrease) {
			//Then, try decreasing
			switch (param_index) {
			case 0:
				Kp = Kp - 2 * dp[param_index];
				break;
			case 1:
				Kd = Kd - 2 * dp[param_index];
				break;
			case 2:
				Ki = Ki - 2 * dp[param_index];
				break;
			default:
				break;
			}
			twiddle_decrease = true;
		}
		else (twiddle_increase && twiddle_decrease) {
			//First, try increasing
			switch (param_index) {
			case 0:
				Kp = Kp - 2 * dp[param_index];
				break;
			case 1:
				Kd = Kd - 2 * dp[param_index];
				break;
			case 2:
				Ki = Ki - 2 * dp[param_index];
				break;
			default:
				break;
			}
			dp[param_index] *= 0.9;
			param_index = (param_index + 1) % 3;
			twiddle_increase = twiddle_decrease = false;
		}
		total_error = 0;
		cout << "new parameters" << endl;
		cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;

	}
	
	loop_counter

	return total_error;

}


