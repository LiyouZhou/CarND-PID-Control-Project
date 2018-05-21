# CarND-Controls-PID Project

The goal of the project is to implement a PID controller in c++ which can drive a simulated vehicle around a simulated track. The simulator will provide the controller with the cross track error (CTE) and the velocity (mph). The controller needs to set the appropriate steering angle for the vehicle.

## Reflection

### Implementation of PID controller

The idea of a PID controller is to output a control signal which is the weighted sum of 3 error signals: the proportional error, the integral error and the differential error. [This code](src/PID.cpp#L36-L38) shows the calculation of the error. [This code](src/PID.cpp#L48) shows the weighted summation. All 3 errors are derived from the cross talk error (CTE) which is the position of the car relative to the middle of the lane.

**The proportional error** is proportional to the CTE, it is here to ensure that the wheel always turn towards the middle of the lane.

**The differential error** is the difference of the current CTE and the CTE from a previous time step. This term cause the car to steer less towards the middle of the lane if CTE is already decreasing. This has the effect of dampening oscillation caused by the proportional term.

**The integral error** is the sum of all CTE in all previous time steps. Proportional and differential term can act against each other and causes the system to stabilize at non-zero error. In that case the integral term will grow with time forcing the system towards zero error. However if the CTE is large the integral term may grow very large very quickly and takes very long time to unwind. During which time the integral term dominates the control signal and cause the system to react slowly and become unstable. To solve this problem, the value of the integral term is bounded. Here is the [code](src/PID.cpp#L41-L42) that implements this **anti windup measure**.

### Tuning of PID parameters

Tuning is achieved with a combination of manual tuning and twiddle algorithm.

#### The Twiddle algorithm

The algorithm is implemented [here](src/twiddle.cpp). The basic idea is to perturb each PID weights and measure if an improvement is made to the control process. The amount of perturbation is increased if an improvement is made and decreased otherwise.

**The Goodness** is an indicator for the performance of the controller. [Here](src/main.cpp#L108) it is defined as the accumulated magnitude of CTE divided by the distance travelled by the car on the road. It penalizes large CTEs and encourages traveling long distances without leaving the road.

Although the twiddle algorithm is simple to implement and work for certain usecases, I found it to fail because of the following reasons.
1. The algorithm assumes a deterministic system. There is a certain degree of stochasticity in the simulator, one set of parameter fail during tuning and hence rejected by the algorithm might perform better in most cases than the other ones.
1. The algorithm assumes parameters are independent. In fact the PID parameters might co-vary hence tuning them individually one by one result in confusing outcomes.
1. The algorithm assumes the parameter space concave.It is very prone to being stuck in local minimums.

Hence a probability based optimization algorithm such as simulated annealing or genetic algorithm might fair better in this task. However the large number of trails required to run these algorithms might prove to be time consuming when running the simulator in real time.

#### Speed controller

To help the vehicle deal with difficult corners a few simple rules are implemented to control the throttle and hence the speed. The code can be found [here](src/main.cpp#L123-L132).

1. If speed is less than target speed, apply full throttle.
1. If CTE error grow rapidly, apply half break.
1. If car nearing edge of the road, apply full break.

#### Running the code

The compiled binary takes a number of parameters
```sh
# Running PID directly
./pid <Kp> <Ki> <Kd> <target_speed>
# Running Twiddle tuning
./pid <Initial Kp> <Initial Ki> <Initial Kd> <target_speed>
      <Initial perturb amount dKp> <Initial perturb amount dKi>
      <Initial perturb amount dKd>
      <the ratio of decrease of perturb amount>
      <Final Tolerance>
      <Number of time steps to run at each iteration>
```

## Result

This [videos](https://youtu.be/efuUPviqgTo) shows the twiddle algorithm running. You can see the car gradually start to follow the road successfully. Due to time concerns, the algorithm is only run over a small section of the road which was mostly straight. Hence the tuned parameters did not fair well during corners, hence extra manual tuning is done and the final parameters are:
Kp = 0.06, Ki = 0.01, Kd = 0.4.

![Car drive around the track](https://media.giphy.com/media/3baTNQklNxdoyjjSgE/giphy.gif)
Full video can be found [here](https://youtu.be/fndTk8xGfPQ). The vehicle is able to drive around the track without leaving the road but had to slow down significantly during corners. The PID controller fails significantly during corners as it deals with corners the same way as it deals with straight bits.