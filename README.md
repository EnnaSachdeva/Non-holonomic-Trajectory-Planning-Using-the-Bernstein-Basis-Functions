# Non-holonomic-Trajectory-Planning-Using-the-Bernstein-Basis-Functions

To generate a kinematically feasible trajectory for a differential drive robot in an obstacle-free environment, it uses the
Bernstein basis method to plan smooth trajectories.

The implementation has to be validated in the following scenarios.
• Under-constrained
• Exactly-constrained
• Over-constrained

We have used position and velocity constraints of the state of a
non-holonomic robot, in a 2D environment, at time t0 = 0,tc = 5 and tf = 10 seconds to determine
the weights WXi for i=0...5. To put strict constraints on the initial and the final positions of the
robot, the optimization has been applied only to solve for weights WX1 to WX4 while keeping the
weight parameters WX0 and WX5 fixed as X0 and Xf, respectively. This is achieved by solving
equations corresponding to X0 and Xf, separately using substitution method, rather than solving
along with other constraints (as A = BW). 
