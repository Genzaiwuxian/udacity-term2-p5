#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <vector>

using namespace std;

size_t N_h = 10;

//the first ID of each instances
const int ID_FIRST_X = 0;
const int ID_FIRST_Y = ID_FIRST_X + N_h;
const int ID_FIRST_PSI = ID_FIRST_Y + N_h;
const int ID_FIRST_V = ID_FIRST_PSI + N_h;
const int ID_FIRST_CTE = ID_FIRST_V + N_h;
const int ID_FIRST_EPSI = ID_FIRST_CTE + N_h;
const int ID_FIRST_DELTA = ID_FIRST_EPSI + N_h;
const int ID_FIRST_A = ID_FIRST_DELTA + N_h;

//target velocity
const double TARGET_VELOCITY = 60.0;

//weights for each constraints
const double weight_cte = 1.0;
const double weight_epsi = 1.0;
const double weight_v = 1.0;
const double weight_delta = 1.0;
const double weight_a = 1.0;
const double weight_delta_diff = 1.0;
const double weight_a_diff = 1.0;

//numbers of states(6: x, y, psi, v, cte, epsi) + steering&acc/deceleration (2: delta, a)
const int N_STATES_ACTUATIONS = N_h * 6 + (N_h - 1) * 2;
const int N_STATES = N_h * 6;


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double steer;
  double throttle;

  std::vector<double> x_mpc;
  std::vector<double> y_mpc;

};

#endif /* MPC_H */
