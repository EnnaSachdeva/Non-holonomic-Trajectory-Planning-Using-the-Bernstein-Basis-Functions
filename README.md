# Non-holonomic-Trajectory-Planning-Using-the-Bernstein-Basis-Functions

To generate a kinematically feasible trajectory for a differential drive robot in an obstacle-free environment, it uses the
Bernstein basis method to plan smooth trajectories.

The implementation has to be validated in the following scenarios.
• Under-constrained
• Exactly-constrained
• Over-constrained

It uses position and velocity constraints of the state of a
non-holonomic robot, in a 2D environment, at time t_0 = 0,t_c = 5 and t_f = 10 seconds to determine
the weights WX_i for i=0...5. To put strict constraints on the initial and the final positions of the
robot, the optimization has been applied only to solve for weights WX_1 to WX_4 while keeping the
weight parameters WX_0 and WX_5 fixed as X0 and Xf, respectively. This is achieved by solving
equations corresponding to X_0 and X_f, separately using substitution method, rather than solving
along with other constraints (as A = BW). 
