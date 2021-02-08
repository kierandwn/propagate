# control

Control module for the dynamics capstone mission to mars. Includes support for a number of controller configurations, as described below:

Configuration for this module is described in global YAML configuration, under the `control` node, e.g.

```
control:
  scheme:
    type: simple_pd
    gain:
      - proportional: 5.
      - derivative: 10.
      - integral: .01
    gyroscopics: false
    feedforward_term: false
    integral: false
  reference_state: orbit
  external_torques:
    type: constant
    value: [0., 0., 0.]
```

This indicates that a simple proportional-derivative controller should be used -with the specified gains- without including an integral term. The reference state to be tracked is the `orbit` trajectory: as specified in the capstone mission brief.

## Control schemes

### Simple Proportional Derivative

This control computes the control torques to be applied given the attitude error, ![attitude_error](https://latex.codecogs.com/svg.latex?\vec{\sigma}_{B/R}), and rate error, ![rate_error](https://latex.codecogs.com/svg.latex?\vec{\omega}_{B/R}), which represent the divergence between the current state (attitudes & rates) and the desired reference states.

![simple_pd](https://latex.codecogs.com/svg.latex?\Large&space;\vec{u}&space;=&space;K&space;\vec{\sigma}_{B/R}&space;+&space;\left[&space;P&space;\right]&space;\vec{\omega}_{B/R})

The magnitudes of gains ![proportional_gain](https://latex.codecogs.com/svg.latex?K) (proportional) & ![rate_gain](https://latex.codecogs.com/svg.latex?\left[P\right]) (derivative) determine how strongly errors from reference should be countered. This has an impact on control performance, i.e. how well does the control bring the system to track its target, and actuator saturation, i.e. how much of the available control authority is used to complete a command (is it exceeded?). Limitations in actuation will be included at a later stage. By convention, the rate gain is expressed as a matrix which is determined as identity multiplied the derivative gain - this reduces the dimensionality of gain selection task but for so long as these matrices (the same argument applies for proportional gain), stability is ensured.

The simple proportional control is the most simple control scheme, but there are terms that can be added to improve performance:  

**Gyroscopics term**  

This term counters the gyroscopic effects of a rotating body:  

![gyro](https://latex.codecogs.com/svg.latex?\Large&space;\vec{u}&space;=&space;K&space;\vec{\sigma}_{B/R}&space;+&space;\left[&space;P&space;\right]&space;\vec{\omega}_{B/R}&space;+&space;\vec{\omega}_{B/N}&space;\times&space;\left[I\right]\vec{\omega}_{B/N})

This control term can be configured on/off with the boolean attribute in the scheme parameters of the config:

```
    gyroscopics: true
```

**Feed-forward term**

This term accounts for the fact that the reference trajectory is time-varying and itself has momentum:  

![feedforward](https://latex.codecogs.com/svg.latex?\Large&space;\vec{u}&space;=&space;K&space;\vec{\sigma}_{B/R}&space;+&space;\left[&space;P&space;\right]&space;\vec{\omega}_{B/R}&space;+&space;\left[I\right]\left(\dot{\vec{\omega}}_{B/R}&space;-&space;\vec{\omega}_{B/N}&space;\times&space;\vec{\omega}_{B/R}\right))

This control term can be configured on/off with the boolean attribute in the scheme parameters of the config:

```
    feedforward: true
```


**Integral term**

The integral term tries to reduce steady state errors in the response by acting upon the integral of the error channel in time: it becomes more powerful the longer the system remains perturbed from the reference. This should be used with care: it can make the system slow to react to high frequency perturbation.  

![feedforward](https://latex.codecogs.com/svg.latex?\Large&space;\vec{u}&space;=&space;K&space;\vec{\sigma}_{B/R}&space;+&space;\left[&space;P&space;\right]&space;\vec{\omega}_{B/R}&space;-\left[P\right]\left[K_I\right]\vec{z})

where:  

![z](https://latex.codecogs.com/svg.latex?\Large&space;\vec{z}&space;=&space;\int^t_t_0&space;\left(K\vec{\sigma}_{B/R}&space;+\left[I\right]\dot{\vec{\omega}}_{B/R}\right)&space;dt)


This control term can be configured on/off with the boolean attribute in the scheme parameters of the config:

```
    integral: true
```

And the parameter ![integral_gain](https://latex.codecogs.com/svg.latex?K_I) is configured:

```
    gain:
      - integral: .01
```


**Combined**

The full control sceheme looks as follows:

![complete](https://latex.codecogs.com/svg.latex?%5Cbegin%7Balign*%7D%20%5Cvec%7Bu%7D%26space%3B%26amp%3B%3D%26space%3BK%26space%3B%5Cvec%7B%5Csigma%7D_%7BB/R%7D%26space%3B+%26space%3B%5Cleft%5B%26space%3BP%26space%3B%5Cright%5D%26space%3B%5Cvec%7B%5Comega%7D_%7BB/R%7D%26space%3B-%5Cleft%5BP%5Cright%5D%5Cleft%5BK_I%5Cright%5D%5Cvec%7Bz%7D%26space%3B+%26space%3B%5Cleft%5BI%5Cright%5D%5Cleft%28%5Cdot%7B%5Cvec%7B%5Comega%7D%7D_%7BB/R%7D%26space%3B-%26space%3B%5Cvec%7B%5Comega%7D_%7BB/N%7D%26space%3B%5Ctimes%26space%3B%5Cvec%7B%5Comega%7D_%7BB/R%7D%5Cright%29%26space%3B%5C%5C%26space%3B%26amp%3B+%26space%3B%5Cvec%7B%5Comega%7D_%7BB/N%7D%26space%3B%5Ctimes%26space%3B%5Cleft%5BI%5Cright%5D%5Cvec%7B%5Comega%7D_%7BB/N%7D%20%5Cend%7Balign*%7D)  

![z](https://latex.codecogs.com/svg.latex?\vec{z}&space;=&space;\int^t_t_0&space;\left(K\vec{\sigma}_{B/R}&space;+\left[I\right]\dot{\vec{\omega}}_{B/R}\right)&space;dt)

**External Toqrues**

It is also possible to account for external torques in the control which, for now, is limited to a constant decribed in the body frame but will soon be possible to calculate dynamically. The external torques are simply subtracted from the calculated control torque vector.


## Future work
 - Include linear closed-loop dynamics control.
 - External torques computed from system state.
 - Include actuation models.
 - Increase dimensionality of K & P gains.

