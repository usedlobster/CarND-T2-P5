#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// DONE: Set the timestep length and duration,

const int N = 12 	    ;
const double dt = 0.06   ;

const double ref_velocity = 90.0 ;


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



class MPC {
  public:
    MPC();

    enum start : int {  x = 0, y = 1*N, psi = 2*N, v = 3*N, cte = 4*N, epsi=5*N, delta=6*N, a = 7*N - 1 }  ;

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
