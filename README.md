# Nonlinear Model Predictive Control with actuator delays.
Self-Driving Car Engineer Nanodegree Program
This project is to implement in C++ Model Predictive Control to drive the car around the track. The program uses a simple Global Kinematic Model with tuned parameters to achieve the maximal speed. The aim of this project is to develop a nonlinear model predictive controller (NMPC) to steer a car in a simulator.

---
## Project description 
### The Model

The  model used in this project is a kinematic bicycle model described in the class. The coorinate system was transformed to be relative to the state of the carbefore passed into the car, and this transformation is performed by using the function `transform_points` in `main.cpp`. The model descripted 6 vehicle state elements, x position, y position, heading psi, speed, cross tracking error and heading error, and all related equations are listed as below.
```
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
where, `x,y` are the positions of the car, `psi` is the heading direction, `v` is velocity, `cte` is the cross track error and `epsi` is the heading error. `Lf` is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability. 

### Timestep Length and Elapsed Duration

Model prediction is based on how many timesteps (prediction horizon) projected into future. The short predection is good at quick response, but suffers from the instabilities and increasing latency, while the long prediction timesteps lead to average the output and result in smoother controls. Based on the computing power and results, I tested value of `N` and `dt` which can steers the car smoothly around the track with the velocities around 70 mph. The values used in this project are 12 timesteps and for each step it took around 0.05 sec which yeilds 12*0.05 = 0.6 sec into future, which is enough to overcome the 0.1 sec latency. 

### Polynomial Fitting and MPC Preprocessing
All calculations are performed in the vehicle relative coordinate system. So first of all, waypoints data was transformed into the vehicle space and a 3d order polynomial was fitted to the data. Therefore, the waypoints are obtained in the frame of the vehicle. The transformation between coordinate systems is implemented in `transformGlobalToLocal`. The transformation used is 
```
 X' =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
 Y' =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
```
where `X',Y'` are coordinates in the vehicle coordinate system. 

After the transform, we get the waypoints in vehicle coordinates. The calculation of cte in vehicle coordinates is simpler because car always at the (0.0) and psi is 0. Thus the initial state of the car in the vhicle coordinate is `state << 0, 0, 0, v, cte, epsi`.
### Model Predictive Control with Latency

Before feeding the result back to the simulator a 100ms latency delay was introduced. The latency was introduced to simulate the real delay of physical actuation for self driving car. If the delays are not accounted for oscillations can occur which can futher result in the sampled NMPC problem. The latency here is taken into accounts by constraining the controls to the values of the previous interation for the duration of latency. Hence, the optimal trajectory is computed starting from the time after the latency period. This will lead to the situation the actuations are forced to remain at their previous values for the time of the latency. This is implemented in `MPC::Solve` as below.

```  
  for (int i = delta_start; i < delta_start + latency_ind; i++) {
    vars_lowerbound[i] = delta_prev;
    vars_upperbound[i] = delta_prev;
  }
 ... 
  
  for (int i = a_start; i < a_start+latency_ind; i++) {
    vars_lowerbound[i] = a_prev;
    vars_upperbound[i] = a_prev;
  }
```

So the value fed to the simulator is the first freely varying control parameter of the optimal trajectory:
```
          // compute the optimal trajectory          
          Solution sol = mpc.Solve(state, coeffs);

          double steer_value = sol.Delta.at(latency_ind);
          double throttle_value= sol.A.at(latency_ind);
```
The cost function parameters were tuned by trail and error method. All these parameters are stored in the `src/MPC.h` file. The increased weight on steering changes btween adjacent time intervals is the most important to achieve the smooth trajectories. Overall the drive around this simulation track was smoother compared to the PID controller. 

The resulted gif is as below.

<table style="width:100%">
  <tr>
    <th>
      <p align="center">
       <img src="./img/final.gif" alt="Overview" width="50%">
      </p>
    </th>
  </tr>
  </table>


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
