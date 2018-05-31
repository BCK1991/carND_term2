#include "PID.h"

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
	min_error = 99999;
	max_error = 0;
	cte = 0;
	cte_t_1 = 0;
	loop_counter = 0;

}

void PID::UpdateError(double cte) {

	p_error = cte;
	i_error = i_error + cte;
	d_error = cte - cte_t_1;
	loop_counter++;


}

double PID::TotalError(bool print) {

	total_error = p_error * Kp + i_error * Ki + d_error * Kd;

	if (total_error < min_error)
		min_error = total_error;
	if (print)
		std::cout << 'Iteration : ' << loop_counter << 'Total Error : ' << total_error << 'Best : ' << min_error << endl;
	return total_error;

}

