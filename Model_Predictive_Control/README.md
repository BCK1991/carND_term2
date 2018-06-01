# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

This project aims to develop a C++ based implementation of the Model Predictive Control system that would control a car in simulation environment to finish a track without causing any unexpected behaviour that would risk a human driver.
The system uses kinematic model to model the vehicle at hand. 

[//]: # (Image References)

[image1]: ./Kinematic_Model_Eq.JPG "Kinematic_Model_Eq"
[image2]: ./cte_eq.JPG "cte_eq"
[image3]: ./psi_eq.JPG "psi_eq"

## Student describes their model in detail. This includes the state, actuators and update equations.

The variables of the kinematic model are listed as follows:  Position (x,y), heading (ψ) and velocity (v) based on the lectures. A state vector would look : [x,y,ψ,v]

Two actuators are given as steering angle (δ) and throttle(also covering braking) (a). The range of δ is [-25 25] and the throttle is [-1 1]. This is important to use these as constraints at a later point. The actuators vector would look like : [δ,a]

To predict the next state of the vehicle, we used the kinematic model and simple laws of the motion. Followings equations are taken as the model equations for the implementation (from lectures):

![alt text][image1]

As the control parameter to track the error, cross track error (cte) and orientation error (ψ) are used. The cost functions are derived as follows in the lectures:

![alt text][image2]
![alt text][image3]

## Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The dt value was set to 0.1 to align with the latency of the control system. This was just an initial value to start the trial-and-error. Based on the initial dt = 0.1, I tried multiple N (from 5 to 50) to observe the behaviour of the car. When the number of steps are high, the throughput time of the system goes low which is not a desired situation in a real time situation, that is why I tried to minimum value that would yield a plausible result. the values starting from 10 provided valid results.
The final decision was made after going through the Udacity forum and reading similar topics as dt, N = (0.1 10).

The real challenge was to select the best coefficients for the cost functions. After multiple iterations, I realized without the coefficients, it is almost impossible to reach to a stable result (by only playing with dt, N). The initial values are taken from Udacity forum discussions, after trying multiple set of values, I started tweaking them by trial-and-error. 

## A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The waypoints (locations) are transformed into car's perspective to reduce the complexity of the polynomial fitting function since x and y becomes 0.

## The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

The actuations from the previous step are applied one step later; and the equation updates took place with the later step taken into account. In simple terms, the car started looking 'ahead' and acting accordingly.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
