#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <vector>

using namespace std;

const size_t N=10;
const double dt=0.1;

// size_t N_h = 10;

//the first ID of each instances
const size_t ID_FIRST_X = 0;
const size_t ID_FIRST_Y = ID_FIRST_X + N;
const size_t ID_FIRST_PSI = ID_FIRST_Y + N;
const size_t ID_FIRST_V = ID_FIRST_PSI + N;
const size_t ID_FIRST_CTE = ID_FIRST_V + N;
const size_t ID_FIRST_EPSI = ID_FIRST_CTE + N;
const size_t ID_FIRST_DELTA = ID_FIRST_EPSI + N;
const size_t ID_FIRST_A = ID_FIRST_DELTA + N-1;

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
const auto N_STATES_ACTUATIONS = N * 6 + (N - 1) * 2;
const auto N_STATES = N * 6;


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
