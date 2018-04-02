# Model Predictive Control Project
### CarND-Controls-MPC, Self-Driving Car Engineer Nanodegree Program

This project uses MPC to control the steering and acceleration of a simulated car.

---
## The Model

From an initial given state, the MPC predicts N states ahead. The solver tries various different actuator inputs, and runs a cost function against each of the N states.
The actuator inputs that produce the least value cost, are used as inputs to the  car.

#### State 
 
The model we used has the following state variables. 

    * x : the car x-position 
    * y : the car y-position
    * ψ : psi - the vehicle heading 
    * v : the velocity
    * cte : the cross track error [distance from track ]
    * eψ : error psi - the angle  difference between current heading and track.

#### Actuators [ δ , a ]

The actuators we use are. Where each is limited to the range +/- 1 .

    * δ : delta - (the steering angle) [-1,+1 ]
    * a : a - ( combined throttle and brake pedals ). [-1,+1 ]

#### Update equations 

The model uses the following equations to get the next state [ x' , y' , ψ' , v' , cte' , eψ' ],  from the current state [ x , y , ψ , v , cte , eψ ] after dt seconds.

	* x' = x + v*cos( ψ )*dt 
	* y' = y + v*sin( ψ )*dt
	* ψ' = ψ - v*( δ / Lf )∗dt
	* v' = v + a ∗ dt
	* cte' = ( f( x ) - y ) + v * sin( eψ ) * dt
	* eψ' = ( ψ - tan( f'(x) ) - v * ( δ / Lf ) * dt

the function f(x) - is a polynomial giving the path the car should be following , and f'(x) is the derivative.

#### Cost function

The cost function is a weighted sum of the error =  ( desired - actual  )^2 .

The cost function we use is critical as it guides the best values for [ δ , a ] to use . We don't want the model thinking it can just ramp out the velocity without consequence to going off course, 
similarly we don't want the solver to thinking going at 0 speed is the answer.

We therefore implemented the following cost function like this :-

        fg[0] = 0;
        for (int t = 0; t < N; t++) {
            fg[0] += 476.0 * ( N-t ) * CppAD::pow(vars[MPC::start::cte  + t], 2);
            fg[0] += 476.0 * ( N-t ) * CppAD::pow(vars[MPC::start::epsi + t], 2);
            fg[0] += CppAD::pow(vars[MPC::start::v + t] - ref_velocity, 2);
        }
        for (int t = 0; t < N - 1; t++) {
            fg[0] +=    2.0 * (N-t) * CppAD::pow(vars[MPC::start::delta + t], 2);
            fg[0] +=    2.0 * (N-t) * CppAD::pow(vars[MPC::start::a + t], 2);
        }
        for (int t = 0; t < N - 2; t++) {
            fg[0] +=   100.0 * CppAD::pow(vars[ MPC::start::delta + t + 1] - vars[MPC::start::delta + t], 2);
            fg[0] +=    10.0 * CppAD::pow(vars[ MPC::start::a + t + 1] - vars[MPC::start::a + t], 2);

The cost function rewards very small cte & epsi more heavily than the velocity. We use the weight 476 * ( N -  t ) to prefer reaching the optimum cte and epsi earlier.

We also favour the δ & a terms to be 0 , this makes sense for  δ, 

To prevent  δ & a actuators from changing we also weight so that they prefer to be constant thought the N steps.


## Timestep Length and Elapsed Duration ( N & dt )

The MPC uses N states at dt second intervals. 

The choice of N & dt is critical to performance. The product of N*dt gives us the total time ahead the predictions  are run.

Predicting too far into the future is a bit of a waste, and since conditions can change rapidly would probably be a waste. However to small a value and  upcoming changes ( such as a sharp bend ) may not get incorporated in-time.

With N < 8 the solver had trouble fitting a solution, with N > 30 the solution was taking too long causing erratic behaviour of the car.

We eventually settled for N = 12 , and dt = 0.06.  



## Polynomial Fitting and MPC Preprocessing

The world position way-points from the simulator, are translated to the location and orientation of the car. This is done by translating the way-points to the car , and then rotating them clockwise. This makes some of the mathematics easier, as now the car is travelling along the x-axis  , with the y axis increasing on the left. 

So  the car now has an initial position of (0,0) and a heading of 0 relative to the way-points. Fitting a polynomial to the translated way-points, we can create a function f(x) where the result is the y position of the track ( relative to the car at 0 ). This means the initial cross-track error is simply f(0) , and initial gradient  is f'(0) and angle is atan( f'(0)) . 
   


## Model Predictive Control with Latency

To overcome the effect of latency, we simply predicted the state 100ms forward of the start time , using the update equations of the model. fs


## The vehicle must successfully drive a lap around the track.
fs
See video.




