#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <vector>

using namespace std;

// size_t N_h = 10;

//the first ID of each instances
const int ID_FIRST_X = 0;
const int ID_FIRST_Y = ID_FIRST_X + 10;
const int ID_FIRST_PSI = ID_FIRST_Y + 10;
const int ID_FIRST_V = ID_FIRST_PSI + 10;
const int ID_FIRST_CTE = ID_FIRST_V + 10;
const int ID_FIRST_EPSI = ID_FIRST_CTE + 10;
const int ID_FIRST_DELTA = ID_FIRST_EPSI + 10;
const int ID_FIRST_A = ID_FIRST_DELTA + 10;

//target velocity
const double TARGET_VELOCITY = 60.0;

//weights for each constraints
const double weight_cte = 1500.0;
const double weight_epsi = 1500.0;
const double weight_v = 1.0;
const double weight_delta = 10.0;
const double weight_a = 10.0;
const double weight_delta_diff = 150.0;
const double weight_a_diff = 15.0;

//numbers of states(6: x, y, psi, v, cte, epsi) + steering&acc/deceleration (2: delta, a)
const int N_STATES_ACTUATIONS = 10 * 6 + (10 - 1) * 2;
const int N_STATES = 10 * 6;


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
