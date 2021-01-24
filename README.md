## capstone 

Low Mars Orbit (LMO) propagator written in the completion of the final dynamics capstone mission of the Spacecraft Dynamics & Control specialisation (University of Colorado, Coursera).

Key Features:
 - Propagation in time: iplemented as simple Euler method or Runge Kutta 4.
 - Proportional-Derivative (PD) control implementation on Modified Rodriguez Parameter (MRP) attitude representation. Simple PD scheme can be extended. More control schemes to come.
 
 Note: MRPs as an attitude representation are well suited to control applications as they linearise well, and by switching between original & shadow set (describing short and long rotations respectively), it is possible to avoid complications with singularities inherent in other 3 parameter attitude descriptions. MRP implementation comes from [here](https://github.com/kierandwn/attitude/blob/main/include/mrp.h).

Developments to come:
 - More control schemes & time integration schemes available, configurable.
 - Record solutions to Coursera assignments in test cases.
 - Check validity of config on start: if invalid, end gracefully.
 - Sensor & actuator models & Kalman estimation in-the-loop.

NOTE: the repository is meant simply to house the solution I produced to completed the Coursera Capstone mission, but I've added configurability into the tool so it can also house other pieces of code used to complete the assignments. In the future, I will push all abstractions to my template project for dynamic propagation, [propagate](https://github.com/kierandwn/propagate), so I can use this for projects to come.
