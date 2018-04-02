 # Model Predictive Control Project
CarND-Controls-MPC, Self-Driving Car Engineer Nanodegree Program

This project uses an MPC to drive a car around a simulated track, by calculating the required steering angle and throttle setting.

---
## 1. MPC Model

The MPC model is given a conditional state , a set of update equations ( that work on the state and actuator inputs ) and a cost function. The job of the MPC solver is to find the set of inputs that   minimize the cost function. The inputs that produced the minimal cost solution found, are the inputs to the simulator.

#### State [ x , y , ψ , v , cte , eψ ]
 
The model we used had the following state variables.

	- x : the car x-position 
	- y : the car y-position
	- ψ : psi - the vehicle heading 
	- v : the velociy
	- cte : the cross track error
	- eψ : error psi 

#### Actuators [ δ , a ]

The actuators we use are.

	- δ (the steering angle) [-1,+1 ]
	- a ( combined throttle and brake pedals). [-1,+1 ]

#### Update equations (Vehicle Dynamics)

The new state variables after dt seconds can be computed by the following update equations.

	- x' = x + v*cos(ψ)*dt 
	- y' = y + v*sin(ψ)*dt
	- ψ' = ψ - v*( δ/Lf )∗dt
	- v' = v + a ∗ dt
	- cte' = ( f(x) - y ) + v * sin( eψ ) * dt
	- eψ' = ( ψ - atan( f'(x) ) - v * ( δ/Lf ) * dt

the function f(x) - is the the current path we should be following.

#### MPC Setup:
1. Define the length of the trajectory, N, and duration of each timestep, dt.
    * See below for definitions of N, dt, and discussion on parameter tuning.
* Define vehicle dynamics and actuator limitations along with other constraints.
    * See the state, actuators and update equations above.
* Define the cost function.
    * Cost in this MPC increases with: (sece `MPC.cpp` lines 80-101)
        * Difference from reference state (cross-track error, orientation and velocity)
        * Use of actuators (steering angle and acceleration)
        * Value gap between sequential actuators (change in steering angle and change in acceleration).
    * We take the deviations and square them to penalise over and under-shooting equally.
        * This may not be optimal.
    * Each factor mentioned abov contributed to the cost in different proportions. We did this by multiplying the squared deviations by weights unique to each factor.

#### MPC Loop:
1. We **pass the current state** as the initial state to the model predictive controller.
* We call the optimization solver. Given the initial state, the solver will ***return the vector of control inputs that minimizes the cost function**. The solver we'll use is called Ipopt.
* We **apply the first control input to the vehicle**.
* Back to 1.

*Reference: Setup and Loop description taken from Udacity's Model Predictive Control lesson.*p


#### N and dt
* N is the number of timesteps the model predicts ahead. As N increases, the model predicts further ahead.
* dt is the length of each timestep. As dt decreases, the model re-evaluates its actuators more frequently. This may give more accurate predictions, but will use more computational power. If we keep N constant, the time horizon over which we predict ahead also decreases as dt decreases.

#### Tuning N and dt
* I started with (N, dt) = (10, 0.1) (arbitrary starting point). The green panth would often curve to the right or left near the end, so I tried increasing N so the model would try to fit more of the upcoming path and would be penalised more if it curved off erratically after 10 steps.
* Increasing N to 15 improved the fit and made the vehicle drive smoother. Would increasing N further improve performance?
* Increasing N to 20 (dt = 0.1) made the vehicle weave more (drive less steadily) especially after the first turn. 
    * The weaving was exacerbated with N = 50 - the vehicle couldn't even stay on the track for five seconds. 
* Increasing dt to 0.2 (N = 10) made the vehicle too slow to respond to changes in lane curvature. E.g. when it reached the first turn, it only started steering left when it was nearly off the track. This delayed response is expected because it re-evaluates the model less frequently. 
* Decreasing dt to 0.05 made the vehicle drive in a really jerky way.
* So I chose N = 15, dt = 0.1.
* It would be better to test variations in N and dt more rigorously and test different combinations of N, dt and the contributions of e.g. cross-track error to cost. 
* It would also be good to discuss variations in N and dt without holding N or dt fixed at 10 and 0.1 respectively.

### Latency
* If we don't add latency, the car will be steering and accelerating/braking based on what the model thinks it should've done 100ms ago. The car will respond too slowly to changes in its position and thus changes in cost. 
	* It may not start steering e.g. left when it goes round a curve, leading it to veer off the track. 
	* Likewise, it may continue steering even when the path stops curving. 
	* The faster the vehicle speed, the worse the effects of latency that is unaccounted for.
* **Implementation**: We used the kinematic model to predict the state 100ms ahead of time and then feed that predicted state into the MPC solver.
	* We chose 100ms because that's the duration of the latency. That is, we try to predict where the car will be when our instructions reach the car so the steering angle and throttle will be appropriate for when our instructions reach the car (100ms later).
	* The code can be found in `main.cpp`.

### Other comments
* 




