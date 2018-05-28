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
 
 - 
