# dynamics

Dynamics module (time-integration solvers) for the dynamics capstone mission to mars.  

Configuration for this module is described in global YAML configuration, under the `control` node, e.g.

```
solver:
  integration_scheme: rk4
  timestep: 0.01
  start_time: 0.0
  end_time: 120.0
  channel_names:
    - sigma_0
    - sigma_1
    - sigma_2
    - omega_0
    - omega_1
    - omega_2
  states_are_angles: [false, false, false, true, true, true]
  angles_are_deg: true
  initial_states:
    - 0.3
    - -0.4
    - 0.5 
    - 1.0
    - 1.75
    - -2.2
  state_ndim : 6
  control_in_lockstep: true
  control_update_rate: 100
```

| YAML Parameter      	| Description                                                                                                                       	| Imported to type 	|
|---------------------	|-----------------------------------------------------------------------------------------------------------------------------------	|------------------	|
| integration_scheme  	| Time integration scheme: `rk4` or `euler`.                                                                                        	| string           	|
| timestep            	| Incremental timestep in integration scheme.                                                                                       	| double           	|
| start_time          	| Time to begin simulation.                                                                                                         	| double           	|
| end_time            	| Time to end simulation.                                                                                                           	| double           	|
| channel_names       	| List of string identifiers for each state element.                                                                                	| vector<string>   	|
| states_are_angles   	| Whether each state element is an angular (measured in degrees or radians) value.                                                  	| vector<bool>     	|
| angles_are_deg      	| Whether angular states (above) are represented in degrees.                                                                        	| bool             	|
| initial_states      	| State vector at time `start_time`.                                                                                                	| vector<double>   	|
| state_ndim          	| Number of dimensions in state vector (for now, hardcoded 6, has no effect).                                                       	| int              	|
| control_in_lockstep 	| Whether control is computed in same update loop as dynamics propagation, or run on a separate thread.                             	| bool             	|
| control_update_rate 	| Frequency (Hz) to update control inputs at. Note, if in lockstep, will update on the next timestep after 1/frequency is exceeded. 	| int              	|


## Time Integration schemes

A number of schemes are available to propagate the system dynamics forward in time:
- 4th-order Runge Kutta implemented as described [here](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods).
- Euler method (simplest time integration scheme) implemented as described [here](https://en.wikipedia.org/wiki/Euler_method).


## Future work
 - Add more time-integration schemes.
 - Implement control update on separate thread.
 - Add support for systems for ndim != 6.

