#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;

// TODO: Set the timestep length and duration


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

	  fg[0] = 0.0;

	  //costs
	  for (unsigned int i = 0; i < N; ++i)
	  {
		  //costs for cte, epsi
		  const auto cte = vars[ID_FIRST_CTE + i];
		  const auto cte_2 = cte * cte;

		  const auto epsi = vars[ID_FIRST_EPSI + i];
		  const auto epsi_2 = epsi * epsi;

		  //costs for target velocity
		  const auto v = vars[ID_FIRST_V + i] - TARGET_VELOCITY;
		  const auto v_2 = v * v;

		  fg[0] += (weight_cte * cte_2 + weight_epsi * epsi_2 + weight_v * v_2);
	  }

	  for (unsigned int i = 0; i < N - 1; ++i)
	  {
		  //costs for delta and a
		  const auto delta = vars[ID_FIRST_DELTA + i];
		  const auto delta_2 = delta * delta;

		  const auto a = vars[ID_FIRST_A + i];
		  const auto a_2 = a * a;

		  fg[0] += (weight_delta*delta_2 + weight_a * a_2);

		  // cost for delta and v
		  fg[0] += 100 * CppAD::pow((vars[ID_FIRST_DELTA + i] * vars[ID_FIRST_V]),2);
	  }

	  for (unsigned int i = 0; i < N - 2; i++)
	  {
		 //costs for delta and a changes
		  const auto delta_diff = vars[ID_FIRST_DELTA + i + 1] - vars[ID_FIRST_DELTA + i];
		  const auto delta_diff_2 = delta_diff * delta_diff;

		  const auto a_diff = vars[ID_FIRST_A + i + 1] - vars[ID_FIRST_A + i];
		  const auto a_diff_2 = a_diff * a_diff;

		  fg[0] += (weight_delta_diff*delta_diff_2 + weight_a_diff * a_diff_2);
	  }

	  //constraints
	  fg[ID_FIRST_X + 1] = vars[ID_FIRST_X];
	  fg[ID_FIRST_Y + 1] = vars[ID_FIRST_Y];
	  fg[ID_FIRST_PSI + 1] = vars[ID_FIRST_PSI];
	  fg[ID_FIRST_V + 1] = vars[ID_FIRST_V];
	  fg[ID_FIRST_CTE + 1] = vars[ID_FIRST_CTE];
	  fg[ID_FIRST_EPSI + 1] = vars[ID_FIRST_EPSI];

	  for (unsigned int i = 0; i < N - 1; i++)
	  {
		  //current states and steering & acceleration/deceleration
		  const auto x0 = vars[ID_FIRST_X + i];
		  const auto y0 = vars[ID_FIRST_Y + i];
		  const auto psi0 = vars[ID_FIRST_PSI + i];
		  const auto v0 = vars[ID_FIRST_V + i];
		  const auto cte0 = vars[ID_FIRST_CTE + i];
		  const auto epsi0 = vars[ID_FIRST_EPSI + i];
		  const auto delta0 = vars[ID_FIRST_DELTA + i];
		  const auto a0 = vars[ID_FIRST_A + i];

		  //trinomial fitting: y_dest=f(x0) and psi_dest=f'(x0)
		  const auto x0_2 = x0 * x0;
		  const auto x0_3 = x0_2 * x0;

		  const auto y_dest = coeffs[3] * x0_3 + coeffs[2] * x0_2 + coeffs[1] * x0+coeffs[0];
		  const auto psi_dest = CppAD::atan(3 * coeffs[3] * x0_2 + 2 * coeffs[2] * x0 + coeffs[1]);

		  //next state by MPC(kinematic model)
		  const auto x1_mpc = x0 + v0 * CppAD::cos(psi0)*dt;
		  const auto y1_mpc = y0 + v0 * CppAD::sin(psi0)*dt;
		  const auto psi1_mpc = psi0 + v0 * (-delta0) / Lf * dt;
		  const auto v1_mpc = v0 + a0 * dt;
		  const auto cte1_mpc = y_dest - y0 + v0 * CppAD::sin(epsi0)*dt;
		  const auto epsi1_mpc = psi0 - psi_dest + v0 * (-delta0)/ Lf * dt;

		  //next state
		  const auto x1 = vars[ID_FIRST_X + i+1];
		  const auto y1 = vars[ID_FIRST_Y + i+1];
		  const auto psi1 = vars[ID_FIRST_PSI + i+1];
		  const auto v1 = vars[ID_FIRST_V + i+1];
		  const auto cte1 = vars[ID_FIRST_CTE + i+1];
		  const auto epsi1 = vars[ID_FIRST_EPSI + i+1];

		  //constraints
		  fg[ID_FIRST_X + i + 2] = x1 - x1_mpc;
		  fg[ID_FIRST_Y + i + 2] = y1 - y1_mpc;
		  fg[ID_FIRST_PSI + i + 2] = psi1 - psi1_mpc;
		  fg[ID_FIRST_V + i + 2] = v1 - v1_mpc;
		  fg[ID_FIRST_CTE + i + 2] = cte1 - cte1_mpc;
		  fg[ID_FIRST_EPSI + i + 2] = epsi1 - epsi1_mpc;

		  //cout << "contraints: " << i << endl;
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

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  // size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  

  Dvector vars(N_STATES_ACTUATIONS);
  for (unsigned int i = 0; i < N_STATES_ACTUATIONS; i++) {
    vars[i] = 0;
  }

  vars[ID_FIRST_X] = x;
  vars[ID_FIRST_Y] = y;
  vars[ID_FIRST_PSI] = psi;
  vars[ID_FIRST_V] = v;
  vars[ID_FIRST_CTE] = cte;
  vars[ID_FIRST_EPSI] = epsi;

  Dvector vars_lowerbound(N_STATES_ACTUATIONS);
  Dvector vars_upperbound(N_STATES_ACTUATIONS);
  // TODO: Set lower and upper limits for variables.
  //x, y, psi, v is set to number that computer can handle
  for (int i = 0; i < ID_FIRST_DELTA; ++i)
  {
	  vars_lowerbound[i] = -1.0e19;
	  vars_upperbound[i] = 1.0e19;

	 // cout << "x, y psi, v: " << i << endl;
  }

  //delta<-[-0.7,+0.7]
  for (int i = ID_FIRST_DELTA; i < ID_FIRST_A; ++i)
  {
	  vars_lowerbound[i] = -0.43633;
	  vars_upperbound[i] = 0.43633;

	  // cout << "delta: " << i << endl;
  }

  // a<-[-0.7,1]
  for (unsigned int i = ID_FIRST_A; i < N_STATES_ACTUATIONS; ++i)
  {
	  vars_lowerbound[i] = -0.7;
	  vars_upperbound[i] = 1.0;

	  // cout << "a: " << i << endl;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(N_STATES);
  Dvector constraints_upperbound(N_STATES);
  for (unsigned int i = 0; i < N_STATES; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;

	// cout << "contraints_lowupper: " << i << endl;
  }

  constraints_lowerbound[ID_FIRST_X] = x;
  constraints_lowerbound[ID_FIRST_Y] = y;
  constraints_lowerbound[ID_FIRST_PSI] = psi;
  constraints_lowerbound[ID_FIRST_V] = v;
  constraints_lowerbound[ID_FIRST_CTE] = cte;
  constraints_lowerbound[ID_FIRST_EPSI] = epsi;

  constraints_upperbound[ID_FIRST_X] = x;
  constraints_upperbound[ID_FIRST_Y] = y;
  constraints_upperbound[ID_FIRST_PSI] = psi;
  constraints_upperbound[ID_FIRST_V] = v;
  constraints_upperbound[ID_FIRST_CTE] = cte;
  constraints_upperbound[ID_FIRST_EPSI] = epsi;

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

  steer = solution.x[ID_FIRST_DELTA];
  throttle = solution.x[ID_FIRST_A];

  x_mpc = {};
  y_mpc = {};

  for (unsigned int i = 0; i < N; ++i)
  {
	  double x = solution.x[ID_FIRST_X + i];
	  double y = solution.x[ID_FIRST_Y + i];

	  x_mpc.push_back(x);
	  y_mpc.push_back(y);

  }

  return {};
}
