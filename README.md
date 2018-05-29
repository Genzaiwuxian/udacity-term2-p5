# udacity-term2-p5

# Model Predictive Control
Model predictive control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints.

In this projects, some elements of MPC are:
- states: 
  x: vehicle location of x axis
  
  y: vehicle location of y axis
  
  psi: vehicle angle
  
  v: vehicle speed
  
  cte: cross trajactory error, dy bettwen vehicle location and desination curve
  
  epsi: error of psi between gradient of polyfit curve and vehicle angle

- actuators:
  steer: vehicle steering angle between ±0.43rad (±25°)
  
  throttle: accleration or decelleration pedal, between [-1, +1]
 
 - update equation
  the mpc project use bicycle model as follow:
  - x_t = x_t + v_t * cos(ψ_t) *dt
  - y_t+1 = y_t + v_t * sin(ψ_t) * dt
  - ψ_t+1 = ψ_t + (v_t/Lf)*δ * dt
  - v_t+1 = v_t + a_t * dt
  - cte_t+1= cte_t + v_t * sin(eψ_t) * dt
  - eψ_t+1 = eψ_t +(v_t/Lf) * δ_t * dt
  where: δ is distance x axis between middle point of vehicle and front axis, usually we can calculate by make vehicle run as a cycle to get this value.

# N & dt
N is the number of timesteps in the horizon. dt is how much time elapses between actuations.
N, dt, and T are hyperparameters need to tune for each model predictive controller you build. However, there are some general guidelines. T should be as large as possible, while dt should be as small as possible.
- N: 
- dt: choose 0.1, it equals to latency, mpc can solve latency problem by input the next dt states into solver, so dt=latency will be convinent
- N: choose a large N will make calculation more accuacy as we understanding, however, large N will lead to more calculation time for computer, also for a high speed vehicle, a large N will not suitable for vehicle will not consider too far away.

# pre-processing
- waypoints: as waypoints is global coordinate, however, our mpc is based on vehicle coordinate, and it's more convinent to calcuate cte in vehicle coordinate, so it's better to change waypoints from global coordinate to vehicle coordinate;

- steer_value: mpc solver use rad/s, and the range is [-25, +25], but our simulator only accept steer_value [-1, 1]. so the input for steer_value= mpc.steer/deg2rad(25);

- speed: input speed is miles/h, and it is change into m/s by devided 0.44073;

# latency
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system.
This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system
A contributing factor to latency is actuator dynamics. for example, the latency is 0.1s, for mpc, it can calculate the next states after 0.1s and input those states instead of t-0.1s states. That will be more accuracy, the solver can calculate based on the more accuacary vehicle states.


