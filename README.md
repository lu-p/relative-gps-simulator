# relative-gps-simulator
simulation of the process for satellite relative position estimate via GPS sensor in Matlab/Simulink.

This simulator aims to evaluate the capabilities of a GPS sensor to estimate position of a follower satellite relative  to a leader satellite  on  low  Earth  orbit.  You  have  to  provide  leader  and  follower  trajectories  and velocities in ECI as input and to estimate the amount of noise in your GPS sensor.
For the whole simulation time, this simulator provides relative position estimate and error on the true input trajectories. 

It may seem a bit confusing to do some work to generate an output containing the exact information you have to provide as input… but it’s all ok. This script can be used to build more complex simulators dealing with sensor fusion or noise correction.

What you have to do:

• Provide your trajectory and velocity data in “ECI_state_leader.txt”, "ECI_state_follower.txt" (data format is suggested in the first rows of the files),

• Set GPS noise in "rgps_initialization.m",

• Launch “rgps_initialization.m”,

• Enjoy results.


See "details.pdf" for details.
