#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//determining the offsets/incides for fg - the vector of the constraints (steps of N per contraint)
size_t x_offset = 0;
size_t y_offset = x_offset + N;
size_t psi_offset = y_offset + N;
size_t v_offset = psi_offset + N;
size_t cte_offset = v_offset + N;
size_t epsilon_offset = cte_offset + N;
size_t delta_offset = epsilon_offset + N;
size_t a_offset = delta_offset + N - 1;

//array to hold all coefficients related to the cost
int cost_coeff[8] = {1,3000,3000,5,5,700,200,10}; 

//reference car speed to keep the car running (preventing the situtation error = 0 and car stops)
double v_reference = 70.0;

class FG_eval {
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars) {
		// TODO: implement MPC
		// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.

		//reserve fg[0] for cost
		fg[0] = 0;

		//**Define all cost functions

		//Reference state cost (from Udacity lectures)
		// the weights are tuned based on trial and error
		for (int i = 0; i < N; i++) {
			fg[0] += cost_coeff[0] * CppAD::pow(vars[v_offset + i] - v_reference, 2);
			fg[0] += cost_coeff[1] * CppAD::pow(vars[cte_offset + i], 2);
			fg[0] += cost_coeff[2] * CppAD::pow(vars[epsilon_offset + i], 2);

		}
		// Actuators cost (from Udacity lectures + cross cost function)
		for (int i = 0; i < N - 1; i++) {
			fg[0] += cost_coeff[3] * CppAD::pow(vars[delta_offset + i], 2);
			fg[0] += cost_coeff[4] * CppAD::pow(vars[a_offset + i], 2);
			fg[0] += cost_coeff[5] * CppAD::pow(vars[delta_offset + i] * vars[v_offset + i], 2); //cross cost functions to avoid problems while cornering
		}

		//Differential actuator costs (from Udacity lectures)
		for (int i = 0; i < N - 2; i++) {
			fg[0] += cost_coeff[6] * CppAD::pow(vars[delta_offset + i + 1] - vars[delta_offset + i], 2);
			fg[0] += cost_coeff[7] * CppAD::pow(vars[a_offset + i + 1] - vars[a_offset + i], 2);
		}

		//next state on time
		for (int t = 1; t < N; t++) {
			AD<double> x1 = vars[x_offset + t];
			AD<double> x0 = vars[x_offset + t - 1];
			AD<double> y1 = vars[y_offset + t];
			AD<double> y0 = vars[y_offset + t - 1];
			AD<double> psi1 = vars[psi_offset + t];
			AD<double> psi0 = vars[psi_offset + t - 1];
			AD<double> v1 = vars[v_offset + t];
			AD<double> v0 = vars[v_offset + t - 1];
			AD<double> cte1 = vars[cte_offset + t];
			AD<double> cte0 = vars[cte_offset + t - 1];
			AD<double> epsi1 = vars[epsilon_offset + t];
			AD<double> epsi0 = vars[epsilon_offset + t - 1];

			AD<double> a = vars[a_offset + t - 1];
			AD<double> delta = vars[delta_offset + t - 1];
			if (t > 1) {   // use previous actuations (to account for latency)
				a = vars[a_offset + t - 2];
				delta = vars[delta_offset + t - 2];
			}
			AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
			//AD<double> f0 = coeffs[0] + coeffs[1] * x0;
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
			//AD<double> psides0 = CppAD::atan(coeffs[1]);
			//**Define all contraints

			//initial contraints
			fg[1 + x_offset] = vars[x_offset];
			fg[1 + y_offset] = vars[y_offset];
			fg[1 + psi_offset] = vars[psi_offset];
			fg[1 + v_offset] = vars[v_offset];
			fg[1 + cte_offset] = vars[cte_offset];
			fg[1 + epsilon_offset] = vars[epsilon_offset];

			//Model contraints
			fg[1 + x_offset + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_offset + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_offset + t] = psi1 - (psi0 - v0 / Lf * delta * dt);
			fg[1 + v_offset + t] = v1 - (v0 + a * dt);
			fg[1 + cte_offset + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[1 + epsilon_offset + t] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta * dt);

		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsilon = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  vars[x_offset] = x;
  vars[y_offset] = y;
  vars[psi_offset] = psi;
  vars[v_offset] = v;
  vars[cte_offset] = cte;
  vars[epsilon_offset] = epsilon;

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_offset; i++) {
	  vars_lowerbound[i] = -1.0e19;
	  vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_offset; i < a_offset; i++) {
	  vars_lowerbound[i] = -0.436332;
	  vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_offset; i < n_vars; i++) {
	  vars_lowerbound[i] = -1.0;
	  vars_upperbound[i] = 1.0;
  }

  constraints_lowerbound[x_offset] = x;
  constraints_lowerbound[y_offset] = y;
  constraints_lowerbound[psi_offset] = psi;
  constraints_lowerbound[v_offset] = v;
  constraints_lowerbound[cte_offset] = cte;
  constraints_lowerbound[epsilon_offset] = epsilon;

  constraints_upperbound[x_offset] = x;
  constraints_upperbound[y_offset] = y;
  constraints_upperbound[psi_offset] = psi;
  constraints_upperbound[v_offset] = v;
  constraints_upperbound[cte_offset] = cte;
  constraints_upperbound[epsilon_offset] = epsilon;
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  vector<double> return_sol;

  return_sol.push_back(solution.x[delta_offset]);
  return_sol.push_back(solution.x[a_offset]);

  for (int i = 0; i < N - 1; i++) {
	  return_sol.push_back(solution.x[x_offset + i + 1]);
	  return_sol.push_back(solution.x[y_offset + i + 1]);
  }

  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return return_sol;
}
