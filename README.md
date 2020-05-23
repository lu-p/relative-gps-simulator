# relative-gps-simulator
simulation of the process for satellite relative position estimate via GPS sensor in Matlab/Simulink.

This simulator aims to evaluate the capabilities of a GPS sensor to estimate relative position of a satellite on orbit. You have to provide your satellite trajectories in ECI as input (in my case, I got it from GMAT simulations) and estimate the noise in your GPS sensor. For the whole simulation time, this simulator estimates follower position relative to the leader satellite by solving GPS nonlinear system using Multivariate Newton’s method. It also provides absolute and relative errors with reference to the true input trajectories. See “details.pdf” for details.

It may seem a bit confusing to do some work to generate an output giving the exact information you have to provide as input… but it’s all ok. This script can be used to build more complex simulators dealing with sensor fusion or noise correction. For example, you can add a Kalman filter to the current output values to estimate its capabilities to improve position estimates. You can also use Kalman filter for sensor fusion to merge position estimate from different navigation sensors.

What you have to do:

• Provide your trajectory and velocity data in “ECI_state_leader.txt”, "ECI_state_follower.txt", "LVLH_state_follower.txt" (data format is suggested in the first rows of the files),

• Set GPS noise in "rgps_initialization.m",

• Launch “rgps_initialization.m”,

• Enjoy results.
