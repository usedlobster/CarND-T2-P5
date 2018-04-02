#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//
// UL: Sorry, we have moved constants N , dt , Lf , ref_velocity into MPC.h
//

class FG_eval {
  public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) {
        this->coeffs = coeffs;
    }
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        // DONE: implement MPC
        //
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.


        // UL: calculate the cost of are current solutions ( vars )


        fg[0] = 0;

        // UL : Add basic linear costs.
        //
        for (int t = 0; t < N; t++) {
            fg[0] += 476.0 * ( N-t ) * CppAD::pow(vars[MPC::start::cte  + t], 2);
            fg[0] += 476.0 * ( N-t ) * CppAD::pow(vars[MPC::start::epsi + t], 2);
            fg[0] += CppAD::pow(vars[MPC::start::v + t] - ref_velocity, 2);
        }

        // UL : Make sure we are not changing acctuators - if we can help it .
        //
        for (int t = 0; t < N - 1; t++) {
            fg[0] +=    2.0 * (N-t) * CppAD::pow(vars[MPC::start::delta + t], 2);
            fg[0] +=    2.0 * (N-t) * CppAD::pow(vars[MPC::start::a + t], 2);
        }

        // UL : Try to limit the rate of change of acctuators
        //
        for (int t = 0; t < N - 2; t++) {
            fg[0] +=   100.0 * CppAD::pow(vars[ MPC::start::delta + t + 1] - vars[MPC::start::delta + t], 2);
            fg[0] +=    10.0 * CppAD::pow(vars[ MPC::start::a + t + 1] - vars[MPC::start::a + t], 2);
        }

        AD<double> x0 	 = fg[ 1 + MPC::start::x 	] = vars[ MPC::start::x    ] ;
        AD<double> y0 	 = fg[ 1 + MPC::start::y 	] = vars[ MPC::start::y    ] ;
        AD<double> psi0  = fg[ 1 + MPC::start::psi 	] = vars[ MPC::start::psi  ] ;
        AD<double> v0 	 = fg[ 1 + MPC::start::v	] = vars[ MPC::start::v    ] ;
        AD<double> cte0  = fg[ 1 + MPC::start::cte 	] = vars[ MPC::start::cte  ] ;
        AD<double> epsi0 = fg[ 1 + MPC::start::epsi	] = vars[ MPC::start::epsi ] ;

        for ( size_t t = 0 ; t < N - 1 ; t++ ) {
            // vars at t+1
            AD<double> x1 	 = vars[ t + 1 + MPC::start::x    ] ;
            AD<double> y1 	 = vars[ t + 1 + MPC::start::y    ] ;
            AD<double> psi1  = vars[ t + 1 + MPC::start::psi  ] ;
            AD<double> v1 	 = vars[ t + 1 + MPC::start::v    ] ;
            AD<double> cte1  = vars[ t + 1 + MPC::start::cte  ] ;
            AD<double> epsi1 = vars[ t + 1 + MPC::start::epsi ] ;

            AD<double> delta0 = vars[ MPC::start::delta + t  ];
            AD<double> a0 	  = vars[ MPC::start::a + t ];

            // AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0 ;
            AD<double> f0 = ((coeffs[3] * x0 + coeffs[2] ) * x0 + coeffs[1] ) * x0 + coeffs[0] ;
            //AD<double> psides0 = CppAD::atan( coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3]* x0 * x0  ));
            AD<double> psides0 = CppAD::atan(( 3.0*coeffs[3] * x0 + 2.0*coeffs[2] ) * x0 + coeffs[1] ) ;



            fg[ 2 + t + MPC::start::x 	 ] = x1    - ( x0 + v0 * CppAD::cos(psi0) * dt ) ;
            fg[ 2 + t + MPC::start::y 	 ] = y1    - ( y0 + v0 * CppAD::sin(psi0) * dt ) ;
            fg[ 2 + t + MPC::start::psi  ] = psi1  - ( psi0 - v0 * delta0 / Lf  * dt ) ;
            fg[ 2 + t + MPC::start::v	 ] = v1    - ( v0 + a0 * dt ) ;
            fg[ 2 + t + MPC::start::cte  ] = cte1  - (( f0 - y0 ) + ( v0 * CppAD::sin(epsi0) * dt )) ;
            fg[ 2 + t + MPC::start::epsi ] = epsi1 - (( psi0 - psides0 ) - v0 * delta0 / Lf * dt ) ;

            // x0 state will be x1 state on next iteration.

            x0 	  = x1 ;
            y0 	  = y1 ;
            psi0  = psi1 ;
            v0 	  = v1 ;
            cte0  = cte1 ;
            epsi0 = epsi1 ;

        }

    }






};


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    // bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // Set the number of model variables (includes both states and inputs).

    // we assume the state size is 6 , and there are two control inputs

    const size_t n_vars = ( 6 * N ) +  2 * ( N - 1 );
    const size_t n_constraints = ( 6 * N ) ;

    Dvector vars( n_vars ) ;
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);


    // UL : setup vars / and lower - upper bounds

    for ( size_t i=0; i<n_vars; i++) {
        vars[i] = 0.0 ;

        if ( i < MPC::start::delta ) {
            // for [ 0..delta-1 ]
            vars_lowerbound[i] = -1e16 ;
            vars_upperbound[i] =  1e16 ;
        } else if ( i < MPC::start::a  ) {
            // for [ delta .. a-1 ]
            vars_lowerbound[i] = -0.4363325 * Lf ;
            vars_upperbound[i] =  0.4363325 * Lf ;
        } else {
            // for [ a .. n_vars ]
            vars_lowerbound[i] = -1.0 ;
            vars_upperbound[i] =  1.0 ;
        }
    }

    // std::cout << vars_lowerbound << std::endl ;
    // std::cout << vars_upperbound << std::endl ;

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    // set all contraints to 0 initially
    for ( size_t i=0; i< n_constraints; i++ )
        constraints_lowerbound[i] = constraints_upperbound[i] = 0.0 ;

    // set start state.

    vars[ MPC::start::x    ] = state[0] ;
    vars[ MPC::start::y    ] = state[1] ;
    vars[ MPC::start::psi  ] = state[2] ;
    vars[ MPC::start::v    ] = state[3] ;
    vars[ MPC::start::cte  ] = state[4] ;
    vars[ MPC::start::epsi ] = state[5] ;

    // set upper / lower bounds constraints ...

    constraints_lowerbound[ MPC::start::x ]    = constraints_upperbound[ MPC::start::x    ] =  state[0] ;
    constraints_lowerbound[ MPC::start::y ]    = constraints_upperbound[ MPC::start::y    ] =  state[1] ;
    constraints_lowerbound[ MPC::start::psi  ] = constraints_upperbound[ MPC::start::psi  ] =  state[2] ;
    constraints_lowerbound[ MPC::start::v    ] = constraints_upperbound[ MPC::start::v    ] =  state[3] ;
    constraints_lowerbound[ MPC::start::cte  ] = constraints_upperbound[ MPC::start::cte  ] =  state[4] ;
    constraints_lowerbound[ MPC::start::epsi ] = constraints_upperbound[ MPC::start::epsi ] =  state[5] ;

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
    // ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    // UL : We should check the status of the solution
    // but if it didn't solve what would we do ?


    vector<double> result ;

    result.push_back( solution.x[ MPC::start::delta ] ) ;
    result.push_back( solution.x[ MPC::start::a ] ) ;


    for (int i = 0; i < N-1; i++) {
        result.push_back(solution.x[MPC::start::x + i ]);
        result.push_back(solution.x[MPC::start::y + i ]);
    }


    return result ;
}
