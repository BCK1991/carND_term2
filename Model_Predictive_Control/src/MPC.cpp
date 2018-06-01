#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 0;
double dt = 0;

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
size_t a_offset = delta_offset + N;

//array to hold all coefficients related to the cost
int cost_coeff[8] = {1,3000,2000,5,5,500,250,10}; 

//reference car speed to keep the car running (preventing the situtation error = 0 and car stops)
double v_reference = 30.0;

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


	  //**Define all contraints

	  //iti
	  fg[1 + x_offset] = vars[x_offset];
	  fg[1 + y_offset] = vars[y_offset];
	  fg[1 + psi_offset] = vars[psi_offset];
	  fg[1 + v_offset] = vars[v_offset];
	  fg[1 + cte_offset] = vars[cte_offset];
	  fg[1 + epsilon_offset] = vars[epsilon_offset];

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 0;
  // TODO: Set the number of constraints
  size_t n_constraints = 0;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

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
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
